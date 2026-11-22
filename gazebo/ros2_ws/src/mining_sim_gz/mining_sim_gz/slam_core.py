"""LiDAR odometry + pose-graph SLAM (ROS-independent). Port of src/slam/SlamEstimator.js.

  1. predict with the (noisy) odometry increment
  2. align the sweep to a submap of the last keyframes (point-to-line ICP + odometry prior)
  3. keyframes are linked by odometry edges; revisits are verified by scan matching and become loop-closure edges
  4. a pose-graph optimiser straightens the whole trajectory

Differences from the JS version: the loop-closure search is coarse-to-fine (cheap ICP from a grid of offsets, then
refinement of the best few) so it keeps up with a real-time ROS node.
"""
import math

import numpy as np

from .pose_graph import optimize_pose_graph
from .scan_matcher import build_reference, icp
from .se2 import between, compose, wrap

DEFAULTS = dict(
    # scan-matching odometry on/off. Off = odometry + keyframes + loop closure + pose graph only (see gazebo/README: ICP is not yet robust on Gazebo scans)
    scan_matching=True,
    keyframe_distance=1.5, keyframe_angle=0.4, submap_keyframes=3,
    loop_min_path_gap=30.0, loop_search_radius=7.0, loop_min_fitness=0.55, loop_min_info=1500.0,
    # odometry prior of the scan matcher: sigma_t = sigma_t0 + sigma_t_per_m * step length, sigma_r in rad (tighten for a good gyro)
    prior_sigma_t0=0.15, prior_sigma_t_per_m=0.0, prior_sigma_r=0.03,
    # along the direction of motion (where long smooth tunnels give the scan no information) odometry is trusted much more; None = isotropic
    prior_sigma_along0=None, prior_sigma_along_per_m=0.05,
    loop_max_correction=5.0, loop_retry_every=8, loop_max_per_keyframe=2, loop_offsets=(-3.0, -1.5, 0.0, 1.5, 3.0),
)


def _to_world(pose, pts):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return np.stack([c * pts[:, 0] - s * pts[:, 1] + pose[0], s * pts[:, 0] + c * pts[:, 1] + pose[1]], axis=1)


class Keyframe:
    __slots__ = ("pose", "pts", "t", "path", "last_tried", "truth")

    def __init__(self, pose, pts, t, path, truth=None):
        self.pose, self.pts, self.t, self.path, self.last_tried, self.truth = pose, pts, t, path, -99, truth


class Sample:
    __slots__ = ("t", "kf", "rel", "odom", "truth", "fit")

    def __init__(self, t, kf, rel, odom, truth, fit):
        self.t, self.kf, self.rel, self.odom, self.truth, self.fit = t, kf, rel, odom, truth, fit


class SlamEstimator:
    def __init__(self, **overrides):
        self.cfg = {**DEFAULTS, **overrides}
        self.reset()

    # ------------------------------------------------------------------ state
    def reset(self):
        self.est = None
        self.odom_pose = None
        self.kfs = []
        self.odo_edges = []
        self.loop_edges = []
        self.loop_closures = []
        self.samples = []
        self.submap = None
        self.path_len = 0.0
        self.rejected = 0
        self.version = 0

    def init(self, pose):
        """Declare the (known) start pose; in GPS-denied operation the start frame is the map origin."""
        self.reset()
        self.est = np.array(pose, dtype=float)
        self.odom_pose = self.est.copy()

    @property
    def pose(self):
        return None if self.est is None else self.est.copy()

    # ------------------------------------------------------------------ update
    def update(self, odom_delta, pts, t, truth=None):
        """odom_delta: relative motion since the previous call in the robot frame (x, y, th). pts: (m,2) sensor-frame 2D points.
        `truth` (x, y, yaw) is only stored for error metrics, never used for matching."""
        pts = np.asarray(pts, dtype=float).reshape(-1, 2)
        d = np.asarray(odom_delta, dtype=float)
        self.path_len += math.hypot(d[0], d[1])
        self.odom_pose = compose(self.odom_pose, d)
        pred = compose(self.est, d)
        fit = 0.0
        if self.cfg["scan_matching"] and self.submap is not None and len(pts) > 40:
            c = self.cfg
            sig_t = c["prior_sigma_t0"] + c["prior_sigma_t_per_m"] * math.hypot(d[0], d[1])
            prior = dict(pose=pred, wT=1 / sig_t ** 2, wR=1 / c["prior_sigma_r"] ** 2)
            step = math.hypot(d[0], d[1])
            if c["prior_sigma_along0"] is not None and step > 0.02:
                heading = self.est[2] + math.atan2(d[1], d[0])
                prior["dir"] = (math.cos(heading), math.sin(heading))
                prior["wT_along"] = 1 / (c["prior_sigma_along0"] + c["prior_sigma_along_per_m"] * step) ** 2
            r = icp(self.submap, pts, pred, prior=prior, iters=25)
            fit = r.fitness
            self.est = r.pose if r.fitness > 0.3 else pred
        else:
            self.est = pred
        c = self.cfg
        last = self.kfs[-1] if self.kfs else None
        if last is None or math.hypot(self.est[0] - last.pose[0], self.est[1] - last.pose[1]) > c["keyframe_distance"] \
                or abs(wrap(self.est[2] - last.pose[2])) > c["keyframe_angle"]:
            self._add_keyframe(pts, t, truth)
        anchor = len(self.kfs) - 1
        self.samples.append(Sample(t, anchor, between(self.kfs[anchor].pose, self.est), self.odom_pose.copy(),
                                   None if truth is None else np.array(truth, dtype=float), fit))
        self.version += 1

    def _rebuild_submap(self):
        from_ = max(0, len(self.kfs) - self.cfg["submap_keyframes"])
        world = np.vstack([_to_world(k.pose, k.pts) for k in self.kfs[from_:]])
        self.submap = build_reference(world)

    def _add_keyframe(self, pts, t, truth):
        k = len(self.kfs)
        if k > 0:
            prev = self.kfs[k - 1]
            self.odo_edges.append((k - 1, k, between(prev.pose, self.est), np.array([1 / 0.05 ** 2, 1 / 0.05 ** 2, 1 / 0.01 ** 2])))
        self.kfs.append(Keyframe(self.est.copy(), pts.copy(), t, self.path_len, None if truth is None else np.array(truth, dtype=float)))
        self._rebuild_submap()
        if k > 0:
            self._try_loop_closure(k)

    # ------------------------------------------------------------------ loop closure
    def _try_loop_closure(self, k):
        c = self.cfg
        cur = self.kfs[k]
        cands = []
        for i in range(k):
            if cur.path - self.kfs[i].path < c["loop_min_path_gap"] or k - self.kfs[i].last_tried < c["loop_retry_every"]:
                continue
            dist = math.hypot(cur.pose[0] - self.kfs[i].pose[0], cur.pose[1] - self.kfs[i].pose[1])
            if dist < c["loop_search_radius"]:
                cands.append((dist, i))
        cands.sort()
        for _, i in cands[: c["loop_max_per_keyframe"]]:
            self.kfs[i].last_tried = k
            ref = build_reference(self.kfs[i].pts)
            start = between(self.kfs[i].pose, cur.pose)
            coarse_src = cur.pts[:: max(1, len(cur.pts) // 150)]
            coarse = []
            for ox in c["loop_offsets"]:
                for oy in c["loop_offsets"]:
                    for oth in (-0.08, 0.0, 0.08):
                        init = (start[0] + ox, start[1] + oy, start[2] + oth)
                        r = icp(ref, coarse_src, init, iters=6, gate=0.5)
                        coarse.append((r.fitness, init))
            coarse.sort(key=lambda q: -q[0])
            results = []
            for _, init in coarse[:8]:
                r = icp(ref, cur.pts, init, iters=14, gate=0.5)
                if r.fitness >= c["loop_min_fitness"] * 0.8:
                    results.append(r)
            results.sort(key=lambda r: -r.fitness)
            best = results[0] if results else None
            if best is None or best.fitness < c["loop_min_fitness"] or best.hmin < c["loop_min_info"]:
                self.rejected += 1
                continue
            corr = math.hypot(best.pose[0] - start[0], best.pose[1] - start[1])
            ambiguous = any(
                r.fitness >= 0.9 * best.fitness
                and (math.hypot(r.pose[0] - best.pose[0], r.pose[1] - best.pose[1]) > 0.6 or abs(wrap(r.pose[2] - best.pose[2])) > 0.1)
                for r in results)
            if corr > c["loop_max_correction"] or ambiguous:
                self.rejected += 1
                continue
            self.loop_edges.append((i, k, best.pose.copy(), np.array([1 / 0.08 ** 2, 1 / 0.08 ** 2, 1 / 0.03 ** 2])))
            self.loop_closures.append(dict(frm=i, to=k, fitness=best.fitness, correction=corr, info=best.hmin, t=cur.t))
            opt = optimize_pose_graph([q.pose for q in self.kfs], self.odo_edges + self.loop_edges)
            for q, p in enumerate(opt):
                self.kfs[q].pose = p
            self.est = self.kfs[k].pose.copy()
            self._rebuild_submap()
            self.version += 1
            return

    # ------------------------------------------------------------------ outputs
    def sample_pose(self, i):
        s = self.samples[i]
        return compose(self.kfs[s.kf].pose, s.rel)

    def trajectory(self, stride=1):
        return np.array([self.sample_pose(i)[:2] for i in range(0, len(self.samples), stride)]).reshape(-1, 2)

    def odometry_trajectory(self, stride=1):
        return np.array([s.odom[:2] for s in self.samples[::stride]]).reshape(-1, 2)

    def truth_trajectory(self, stride=1):
        return np.array([s.truth[:2] for s in self.samples[::stride] if s.truth is not None]).reshape(-1, 2)

    def metrics(self, window=50):
        """ATE (RMSE, m) of the SLAM estimate and of odometry alone (needs `truth`), RPE over `window` sweeps (10 s at 5 Hz)."""
        n = len(self.samples)
        if n == 0 or self.samples[0].truth is None:
            return dict(samples=n, keyframes=len(self.kfs), loop_closures=len(self.loop_closures), rejected_loops=self.rejected)
        poses = [self.sample_pose(i) for i in range(n)]
        se = so = re = ro = 0.0
        rn = 0
        for i, s in enumerate(self.samples):
            se += (poses[i][0] - s.truth[0]) ** 2 + (poses[i][1] - s.truth[1]) ** 2
            so += (s.odom[0] - s.truth[0]) ** 2 + (s.odom[1] - s.truth[1]) ** 2
            if i >= window:
                a = self.samples[i - window]
                dt = between(a.truth, s.truth)
                de = between(poses[i - window], poses[i])
                dod = between(a.odom, s.odom)
                re += (de[0] - dt[0]) ** 2 + (de[1] - dt[1]) ** 2
                ro += (dod[0] - dt[0]) ** 2 + (dod[1] - dt[1]) ** 2
                rn += 1
        e = self.samples[-1]
        return dict(
            samples=n, ate=math.sqrt(se / n), ate_odom=math.sqrt(so / n),
            rpe=math.sqrt(re / rn) if rn else 0.0, rpe_odom=math.sqrt(ro / rn) if rn else 0.0,
            keyframes=len(self.kfs), loop_closures=len(self.loop_closures), rejected_loops=self.rejected,
            final_error=math.hypot(poses[-1][0] - e.truth[0], poses[-1][1] - e.truth[1]),
            final_error_odom=math.hypot(e.odom[0] - e.truth[0], e.odom[1] - e.truth[1]),
        )


def odometry_noise_model(truth_delta, rng, scale_error=0.02, yaw_scale=0.03, yaw_bias_per_m=0.0008, trans_sigma=0.015, yaw_sigma=0.004):
    """Same simulated wheel-odometry error model as the JS estimator (scale error, turn slip, yaw bias, random walk)."""
    dist = math.hypot(truth_delta[0], truth_delta[1])
    sig = trans_sigma * math.sqrt(dist + 1e-9)
    out = np.array([
        truth_delta[0] * (1 + scale_error) + rng.normal() * sig,
        truth_delta[1] + rng.normal() * sig * 0.5,
        truth_delta[2] * (1 + yaw_scale) + yaw_bias_per_m * dist + rng.normal() * yaw_sigma * math.sqrt(dist + abs(truth_delta[2]) + 1e-9),
    ])
    out[2] = wrap(out[2])
    return out
