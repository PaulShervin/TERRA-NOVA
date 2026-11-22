// LiDAR-odometry + pose-graph SLAM running in "shadow mode" next to the simulator:
//   1. wheel odometry is SIMULATED from the true motion plus realistic errors (scale error, yaw bias, random walk)
//   2. every sweep is aligned (point-to-line ICP + odometry prior) to a local submap of the last keyframes
//   3. keyframes are linked by odometry edges; revisits are detected by scan matching and added as loop-closure edges
//   4. a pose-graph optimiser straightens the whole trajectory
// The simulator's TRUE pose is only used to (a) generate the noisy odometry increments and (b) compute error metrics.
// The estimator never reads the true pose for matching.
import { between, compose, createRng, wrap } from './se2.js';
import { buildReference, icp } from './ScanMatcher.js';
import { optimizePoseGraph } from './PoseGraph.js';

const DEFAULTS = {
  seed: 1,
  odom: { scaleError: 0.02, yawScale: 0.03, yawBiasPerMetre: 0.0008, transSigma: 0.015, yawSigma: 0.004 },
  band: { minRelZ: 0.25, maxRelZ: 2.0 }, // returns used for the 2D match (above the floor, below the ceiling)
  keyframe: { distance: 1.5, angle: 0.4 },
  submapKeyframes: 3,
  loop: { minPathGap: 30, searchRadius: 7, minFitness: 0.55, minInfo: 1500, maxCorrection: 5, retryEvery: 8, maxPerKeyframe: 2, offsets: [-3, -1.5, 0, 1.5, 3] },
  keepScans: false, // keep per-sweep sensor-frame points (evaluation scripts)
};

/** Sensor-frame 2D points from a sweep (rays carry world directions, so rotate back by the sensor yaw). */
export function extractScanPoints(rays, yaw, band, maxPoints = 700) {
  const c = Math.cos(-yaw), s = Math.sin(-yaw), idx = [];
  for (let i = 0; i < rays.length; i++) {
    const r = rays[i];
    if (!r.hit || r.floorLike || r.cat === 'ceiling' || r.relZ < band.minRelZ || r.relZ > band.maxRelZ || r.range > 20) continue;
    idx.push(i);
  }
  const step = Math.max(1, Math.ceil(idx.length / maxPoints)), out = new Float32Array(2 * Math.ceil(idx.length / step));
  let n = 0;
  for (let k = 0; k < idx.length; k += step) {
    const r = rays[idx[k]], wx = r.dx * r.range, wy = r.dy * r.range;
    out[2 * n] = c * wx - s * wy; out[2 * n + 1] = s * wx + c * wy; n++;
  }
  return out.subarray(0, 2 * n);
}

const toWorld = (pose, pts) => {
  const c = Math.cos(pose[2]), s = Math.sin(pose[2]), out = new Float32Array(pts.length);
  for (let i = 0; i < pts.length; i += 2) { out[i] = c * pts[i] - s * pts[i + 1] + pose[0]; out[i + 1] = s * pts[i] + c * pts[i + 1] + pose[1]; }
  return out;
};
const concat = (arrs) => { const out = new Float32Array(arrs.reduce((n, a) => n + a.length, 0)); let o = 0; for (const a of arrs) { out.set(a, o); o += a.length; } return out; };

export function createSlamEstimator(options = {}) {
  const cfg = { ...DEFAULTS, ...options, odom: { ...DEFAULTS.odom, ...options.odom }, loop: { ...DEFAULTS.loop, ...options.loop } };
  const rng = createRng(cfg.seed);
  let est, odomPose, prevTruth, kfs, odoEdges, loopEdges, loopClosures, samples, submap, pathLen, rejected, version;

  function reset() {
    est = odomPose = prevTruth = null; kfs = []; odoEdges = []; loopEdges = []; loopClosures = []; samples = []; submap = null; pathLen = 0; rejected = 0; version = 0;
  }
  reset();

  function noisyOdometry(truth) {
    const d = between(prevTruth, truth), dist = Math.hypot(d[0], d[1]), o = cfg.odom;
    const sig = o.transSigma * Math.sqrt(dist + 1e-9);
    const out = [d[0] * (1 + o.scaleError) + rng.gauss() * sig, d[1] + rng.gauss() * sig * 0.5,
      d[2] * (1 + o.yawScale) + o.yawBiasPerMetre * dist + rng.gauss() * o.yawSigma * Math.sqrt(dist + Math.abs(d[2]) + 1e-9)];
    out[2] = wrap(out[2]);
    return out;
  }

  function rebuildSubmap() {
    const from = Math.max(0, kfs.length - cfg.submapKeyframes);
    submap = buildReference(concat(kfs.slice(from).map((k) => toWorld(k.pose, k.pts))));
  }

  function addKeyframe(pts, t, tp) {
    const k = kfs.length;
    if (k > 0) {
      const prev = kfs[k - 1];
      odoEdges.push({ i: k - 1, j: k, z: between(prev.pose, est), info: [1 / 0.05 ** 2, 1 / 0.05 ** 2, 1 / 0.01 ** 2] });
    }
    kfs.push({ pose: est.slice(), pts: pts.slice(), t, path: pathLen, lastTried: -99, truth: tp }); // truth: display only
    rebuildSubmap();
    if (k > 0) tryLoopClosure(k);
  }

  function tryLoopClosure(k) {
    const L = cfg.loop, cur = kfs[k], cands = [];
    for (let i = 0; i < k; i++) {
      if (cur.path - kfs[i].path < L.minPathGap || k - kfs[i].lastTried < L.retryEvery) continue;
      const d = Math.hypot(cur.pose[0] - kfs[i].pose[0], cur.pose[1] - kfs[i].pose[1]);
      if (d < L.searchRadius) cands.push({ i, d });
    }
    cands.sort((a, b) => a.d - b.d);
    for (const { i } of cands.slice(0, L.maxPerKeyframe)) {
      kfs[i].lastTried = k;
      const ref = buildReference(kfs[i].pts);
      const start = between(kfs[i].pose, cur.pose), results = [];
      for (const ox of L.offsets) for (const oy of L.offsets) for (const oth of [-0.08, 0, 0.08]) {
        const r = icp(ref, cur.pts, [start[0] + ox, start[1] + oy, start[2] + oth], { iters: 14, gate: 0.5 });
        if (r.fitness >= L.minFitness * 0.8) results.push(r);
      }
      results.sort((a, b) => b.fitness - a.fitness);
      const best = results[0];
      if (!best || best.fitness < L.minFitness || best.hmin < L.minInfo) { rejected++; continue; }
      const corr = Math.hypot(best.pose[0] - start[0], best.pose[1] - start[1]);
      const ambiguous = results.some((r) => r.fitness >= 0.9 * best.fitness && (Math.hypot(r.pose[0] - best.pose[0], r.pose[1] - best.pose[1]) > 0.6 || Math.abs(wrap(r.pose[2] - best.pose[2])) > 0.1));
      if (corr > L.maxCorrection || ambiguous) { rejected++; continue; }
      loopEdges.push({ i, j: k, z: best.pose, info: [1 / 0.08 ** 2, 1 / 0.08 ** 2, 1 / 0.03 ** 2] });
      loopClosures.push({ from: i, to: k, fitness: best.fitness, correction: corr, info: best.hmin, t: cur.t });
      const opt = optimizePoseGraph(kfs.map((q) => q.pose), [...odoEdges, ...loopEdges]);
      opt.forEach((p, q) => { kfs[q].pose = p; });
      est = kfs[k].pose.slice();
      rebuildSubmap();
      version++;
      return;
    }
  }

  return {
    /** Declare the (known) start pose. In GPS-denied operation the start frame is the map origin. */
    init(truth) {
      reset();
      est = [truth.x, truth.y, truth.yaw]; odomPose = est.slice(); prevTruth = est.slice();
    },
    /** Feed one sweep. `truth` is only used to synthesise odometry increments and for the error metrics. */
    update(truth, rays, t) {
      const tp = [truth.x, truth.y, truth.yaw];
      if (!est) this.init(truth);
      const d = noisyOdometry(tp);
      pathLen += Math.hypot(d[0], d[1]);
      odomPose = compose(odomPose, d);
      const pred = compose(est, d);
      const pts = extractScanPoints(rays, truth.yaw, cfg.band);
      let matchFit = 0;
      if (submap && pts.length > 40) {
        const r = icp(submap, pts, pred, { prior: { pose: pred, wT: 1 / 0.15 ** 2, wR: 1 / 0.03 ** 2 }, iters: 25 });
        matchFit = r.fitness;
        est = r.fitness > 0.3 ? r.pose : pred;
      } else est = pred;
      prevTruth = tp;
      const last = kfs[kfs.length - 1];
      if (!last || Math.hypot(est[0] - last.pose[0], est[1] - last.pose[1]) > cfg.keyframe.distance || Math.abs(wrap(est[2] - last.pose[2])) > cfg.keyframe.angle) addKeyframe(pts, t, tp);
      const anchor = kfs.length - 1;
      const sample = { t, kf: anchor, rel: between(kfs[anchor].pose, est), truth: tp, odom: odomPose.slice(), fit: matchFit };
      if (cfg.keepScans) sample.pts = pts.slice();
      samples.push(sample);
      version++;
    },
    reset() { reset(); },

    /** Current estimate [x, y, th] in the map frame. */
    get pose() { return est ? { x: est[0], y: est[1], yaw: est[2] } : null; },
    get keyframes() { return kfs; },
    get loopClosures() { return loopClosures; },
    get samples() { return samples; },
    get version() { return version; },
    /** Estimated pose of sample i after the latest graph optimisation. */
    samplePose(i) { const s = samples[i]; return compose(kfs[s.kf].pose, s.rel); },
    trajectory(stride = 1) { const out = []; for (let i = 0; i < samples.length; i += stride) { const p = this.samplePose(i); out.push([p[0], p[1]]); } return out; },
    odometryTrajectory(stride = 1) { const out = []; for (let i = 0; i < samples.length; i += stride) out.push([samples[i].odom[0], samples[i].odom[1]]); return out; },
    truthTrajectory(stride = 1) { const out = []; for (let i = 0; i < samples.length; i += stride) out.push([samples[i].truth[0], samples[i].truth[1]]); return out; },

    /** Absolute trajectory error (RMSE, m) of the SLAM estimate and of odometry alone; relative error over 10 s windows. */
    metrics(window = 50) {
      const n = samples.length;
      if (!n) return { samples: 0, ate: 0, ateOdom: 0, rpe: 0, rpeOdom: 0, keyframes: 0, loopClosures: 0, rejectedLoops: 0, finalError: 0, finalErrorOdom: 0 };
      let se = 0, so = 0, re = 0, ro = 0, rn = 0;
      const poses = samples.map((_, i) => this.samplePose(i));
      for (let i = 0; i < n; i++) {
        const s = samples[i];
        se += (poses[i][0] - s.truth[0]) ** 2 + (poses[i][1] - s.truth[1]) ** 2;
        so += (s.odom[0] - s.truth[0]) ** 2 + (s.odom[1] - s.truth[1]) ** 2;
        if (i >= window) {
          const a = samples[i - window], dt = between(a.truth, samples[i].truth);
          const de = between(poses[i - window], poses[i]), dod = between(a.odom, samples[i].odom);
          re += (de[0] - dt[0]) ** 2 + (de[1] - dt[1]) ** 2; ro += (dod[0] - dt[0]) ** 2 + (dod[1] - dt[1]) ** 2; rn++;
        }
      }
      const e = samples[n - 1];
      return {
        samples: n, ate: Math.sqrt(se / n), ateOdom: Math.sqrt(so / n),
        rpe: rn ? Math.sqrt(re / rn) : 0, rpeOdom: rn ? Math.sqrt(ro / rn) : 0,
        keyframes: kfs.length, loopClosures: loopClosures.length, rejectedLoops: rejected,
        finalError: Math.hypot(poses[n - 1][0] - e.truth[0], poses[n - 1][1] - e.truth[1]),
        finalErrorOdom: Math.hypot(e.odom[0] - e.truth[0], e.odom[1] - e.truth[1]),
      };
    },
  };
}
