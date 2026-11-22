"""Occupancy grid, planner, frontier explorer and waypoint controller (ROS-independent).

Port of src/mapping/{OccupancyGrid,Planner,Explorer}.js and src/RobotController.js:
  log-odds grid -> traversable (inflated) mask -> Dijkstra -> frontier clusters -> goal choice -> string-pulled path -> cos^2 controller
"""
import math

import numpy as np
from scipy import ndimage
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra

UNKNOWN, FREE, OCC = 0, 1, 2
L_OCC, L_FREE, L_MIN, L_MAX = 1.0, -0.25, -2.0, 5.0
SQRT2 = math.sqrt(2.0)
N8 = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0), (1, 1, SQRT2), (1, -1, SQRT2), (-1, 1, SQRT2), (-1, -1, SQRT2)]


class OccupancyGrid:
    """Cell (i, j) <-> world (x, y). Arrays are indexed [j, i]."""

    def __init__(self, x_min, x_max, y_min, y_max, res=0.25):
        self.res, self.x_min, self.y_min = res, x_min, y_min
        self.W, self.H = int(round((x_max - x_min) / res)), int(round((y_max - y_min) / res))
        self.log_odds = np.zeros((self.H, self.W), dtype=np.float32)
        self.seen = np.zeros((self.H, self.W), dtype=bool)

    def to_cell(self, x, y):
        return int(math.floor((x - self.x_min) / self.res)), int(math.floor((y - self.y_min) / self.res))

    def to_world(self, i, j):
        return self.x_min + (i + 0.5) * self.res, self.y_min + (j + 0.5) * self.res

    def inside(self, i, j):
        return 0 <= i < self.W and 0 <= j < self.H

    def state(self):
        s = np.zeros((self.H, self.W), dtype=np.uint8)
        s[self.seen & (self.log_odds < -0.35)] = FREE
        s[self.seen & (self.log_odds > 0.85)] = OCC
        return s

    def _bump(self, ii, jj, delta):
        v = self.log_odds[jj, ii] + delta
        self.log_odds[jj, ii] = np.clip(v, L_MIN, L_MAX)
        self.seen[jj, ii] = True

    def integrate(self, origin, end_xy, obstacle):
        """origin (x, y); end_xy (m, 2) ray end points in the map frame; obstacle (m,) bool = the end point is an obstacle return
        (False for floor returns, overhead returns and max-range misses, which only carve free space)."""
        end_xy = np.asarray(end_xy, dtype=float).reshape(-1, 2)
        obstacle = np.asarray(obstacle, dtype=bool)
        if len(end_xy) == 0:
            return
        # pass 1: occupied end cells, once per scan
        oi = np.floor((end_xy[obstacle, 0] - self.x_min) / self.res).astype(int)
        oj = np.floor((end_xy[obstacle, 1] - self.y_min) / self.res).astype(int)
        ok = (oi >= 0) & (oj >= 0) & (oi < self.W) & (oj < self.H)
        occ_keys = np.unique(oj[ok] * self.W + oi[ok])
        # pass 2: free space along every ray (each cell at most once per scan, never over an occupied cell of this scan)
        vec = end_xy - np.asarray(origin, dtype=float)
        dist = np.hypot(vec[:, 0], vec[:, 1])
        reach = np.where(obstacle, dist - 1.5 * self.res, dist)
        step = self.res * 0.5
        n = int(max(reach.max(initial=0.0), 0.0) / step) + 1
        t = np.arange(n) * step
        use = (t[None, :] <= reach[:, None]) & (dist[:, None] > 1e-6)
        frac = np.where(dist[:, None] > 1e-6, t[None, :] / np.maximum(dist[:, None], 1e-6), 0.0)
        px = origin[0] + vec[:, 0:1] * frac
        py = origin[1] + vec[:, 1:2] * frac
        fi = np.floor((px - self.x_min) / self.res).astype(int)
        fj = np.floor((py - self.y_min) / self.res).astype(int)
        good = use & (fi >= 0) & (fj >= 0) & (fi < self.W) & (fj < self.H)
        free_keys = np.unique(fj[good] * self.W + fi[good])
        free_keys = np.setdiff1d(free_keys, occ_keys, assume_unique=True)
        if len(free_keys):
            self._bump(free_keys % self.W, free_keys // self.W, L_FREE)
        if len(occ_keys):
            self._bump(occ_keys % self.W, occ_keys // self.W, L_OCC)


# ---------------------------------------------------------------------------------------------------------- planner
def _disk(radius_cells):
    r = int(radius_cells)
    y, x = np.ogrid[-r:r + 1, -r:r + 1]
    return (x * x + y * y) <= r * r


def build_traversable(grid, radius, keep_clear=None):
    """Known-free cells farther than `radius` metres from any occupied cell."""
    st = grid.state()
    r = max(1, int(round(radius / grid.res)))
    blocked = ndimage.binary_dilation(st == OCC, structure=_disk(r))
    trav = (st == FREE) & ~blocked
    if keep_clear is not None:
        ci, cj = grid.to_cell(*keep_clear)
        rr = int(math.ceil(0.35 / grid.res))
        for dj in range(-rr, rr + 1):
            for di in range(-rr, rr + 1):
                if grid.inside(ci + di, cj + dj) and st[cj + dj, ci + di] != OCC:
                    trav[cj + dj, ci + di] = True
    return trav, st


def dijkstra_grid(trav, res, start):
    """Cost (metres) and predecessor of every cell from `start` (i, j) over traversable cells; 8-connected, no corner cutting."""
    H, W = trav.shape
    idx = np.arange(H * W).reshape(H, W)
    rows, cols, w = [], [], []
    for di, dj, wt in N8:
        j0, j1 = max(0, -dj), min(H, H - dj)
        i0, i1 = max(0, -di), min(W, W - di)
        a = trav[j0:j1, i0:i1]
        b = trav[j0 + dj:j1 + dj, i0 + di:i1 + di]
        ok = a & b
        if di and dj:
            ok &= trav[j0:j1, i0 + di:i1 + di] & trav[j0 + dj:j1 + dj, i0:i1]
        rows.append(idx[j0:j1, i0:i1][ok])
        cols.append(idx[j0 + dj:j1 + dj, i0 + di:i1 + di][ok])
        w.append(np.full(ok.sum(), wt * res))
    graph = csr_matrix((np.concatenate(w), (np.concatenate(rows), np.concatenate(cols))), shape=(H * W, H * W))
    cost, parent = dijkstra(graph, directed=True, indices=start[1] * W + start[0], return_predecessors=True)
    return cost.reshape(H, W), parent.reshape(H, W)


def find_frontiers(trav, state, min_size=5):
    """Traversable cells with an unknown 4-neighbour, grouped into 8-connected clusters -> list of arrays of (j, i)."""
    unk = state == UNKNOWN
    nb = np.zeros_like(unk)
    nb[1:-1, 1:-1] = unk[1:-1, :-2] | unk[1:-1, 2:] | unk[:-2, 1:-1] | unk[2:, 1:-1]
    front = trav & nb
    lab, n = ndimage.label(front, structure=np.ones((3, 3)))
    out = []
    for k in range(1, n + 1):
        cells = np.argwhere(lab == k)
        if len(cells) >= min_size:
            out.append(cells)
    return out


def extract_path(parent, goal, W):
    path, k = [], goal[1] * W + goal[0]
    flat = parent.ravel()
    while k >= 0:
        path.append((k % W, k // W))
        k = flat[k]
    return path[::-1]


def _line_free(trav, a, b):
    n = int(math.ceil(math.hypot(b[0] - a[0], b[1] - a[1]) * 2)) or 1
    for s in range(n + 1):
        i = int(round(a[0] + (b[0] - a[0]) * s / n))
        j = int(round(a[1] + (b[1] - a[1]) * s / n))
        if not trav[j, i]:
            return False
    return True


def smooth_path(trav, cells):
    """String pulling: keep a cell only when the straight line to the next one is blocked."""
    if len(cells) < 3:
        return cells
    out, anchor = [cells[0]], 0
    for i in range(2, len(cells)):
        if not _line_free(trav, cells[anchor], cells[i]):
            out.append(cells[i - 1])
            anchor = i - 1
    out.append(cells[-1])
    return out


# ------------------------------------------------------------------------------------------------------ controller
class WaypointController:
    """err = wrap(atan2(ty-y, tx-x) - yaw); w = clamp(2.8 err, +-1.8); v = v_max cos^2(min(|err|, 90 deg))."""

    def __init__(self, v_max=0.8, w_max=1.8, tol=0.25):
        self.v_max, self.w_max, self.tol = v_max, w_max, tol
        self.route, self.i = [], 0

    def set_route(self, pts):
        self.route, self.i = [tuple(p) for p in pts], 0
        if len(self.route) > 1:
            self.i = 1

    def is_done(self):
        return self.i >= len(self.route)

    def step(self, x, y, yaw):
        while self.i < len(self.route) and math.hypot(self.route[self.i][0] - x, self.route[self.i][1] - y) < self.tol:
            self.i += 1
        if self.i >= len(self.route):
            return 0.0, 0.0
        tx, ty = self.route[self.i]
        err = (math.atan2(ty - y, tx - x) - yaw + math.pi) % (2 * math.pi) - math.pi
        w = max(-self.w_max, min(self.w_max, 2.8 * err))
        v = self.v_max * math.cos(min(abs(err), math.pi / 2)) ** 2
        return v, w


# ------------------------------------------------------------------------------------------------------- explorer
class FrontierExplorer:
    """Yamauchi frontier exploration with goal commitment, blacklisting, an inflation ladder and a 'finish and go home' rule."""

    def __init__(self, grid, controller, home, truth_mask=None, inflations=(0.5, 0.4, 0.3), size_weight=0.12,
                 replan_every=1.5, stuck_check=4.0, blacklist_radius=1.0, min_goal_dist=0.6):
        # blacklist_radius: the JS explorer uses 1.5 m; on the synthetic test world 1.5 m hid a neighbouring frontier next to a doorway
        # (coverage 85 %), 1.0 m gives 100 % on 3 seeds (test/test_explorer_closed_loop.py).
        self.grid, self.ctl, self.home, self.truth = grid, controller, tuple(home), truth_mask
        self.inflations, self.size_weight = inflations, size_weight
        self.replan_every, self.stuck_check, self.blacklist_radius, self.min_goal_dist = replan_every, stuck_check, blacklist_radius, min_goal_dist
        self.t = 0.0
        self.since_replan, self.since_stuck, self.cov_timer = replan_every, 0.0, 1.0
        self.goal, self.goal_since = None, 0.0
        self.finishing, self.cov_best, self.cov_best_at = False, 0.0, 0.0
        self.state = "exploring"            # exploring | returning | complete
        self.path = []
        self.blacklist = []                 # (x, y, until)
        self.coverage, self.travelled, self.frontiers = 0.0, 0.0, 0
        self.last_pos = None
        self.prev = None

    # -- helpers
    def _coverage(self):
        if self.truth is None:
            return float((self.grid.state() == FREE).sum())
        return float(((self.grid.state() == FREE) & self.truth).sum() / max(self.truth.sum(), 1))

    def _nearest_traversable(self, trav, cell):
        for r in range(8):
            for dj in range(-r, r + 1):
                for di in range(-r, r + 1):
                    i, j = cell[0] + di, cell[1] + dj
                    if self.grid.inside(i, j) and trav[j, i]:
                        return i, j
        return None

    def _route_to(self, trav, parent, cell):
        cells = smooth_path(trav, extract_path(parent, cell, self.grid.W))
        self.path = [self.grid.to_world(i, j) for i, j in cells]
        self.ctl.set_route(self.path)

    def _try_plan(self, x, y, inflation):
        g = self.grid
        trav, st = build_traversable(g, inflation, (x, y))
        start = self._nearest_traversable(trav, g.to_cell(x, y))
        if start is None:
            return False
        cost, parent = dijkstra_grid(trav, g.res, start)
        clusters = find_frontiers(trav, st)
        self.frontiers = len(clusters)
        live = [b for b in self.blacklist if b[2] > self.t]
        cands = []
        for cells in clusters:
            costs = cost[cells[:, 0], cells[:, 1]]
            k = int(np.argmin(costs))
            best_cost = float(costs[k])
            if not math.isfinite(best_cost):
                continue
            j, i = cells[k]
            wx, wy = g.to_world(int(i), int(j))
            if any(math.hypot(b[0] - wx, b[1] - wy) < self.blacklist_radius for b in live):
                continue
            score = best_cost - self.size_weight * min(len(cells), 40)
            if self.goal and math.hypot(self.goal[0] - wx, self.goal[1] - wy) < 1.5:
                score -= 60.0                       # commit to the current target
            cands.append((score, best_cost, wx, wy, (int(i), int(j))))
        usable = [c for c in cands if c[1] > self.min_goal_dist]
        pick = None if self.finishing else min(usable or cands, key=lambda c: c[0], default=None)
        if pick is not None:
            self.state = "exploring"
            if not self.goal or math.hypot(self.goal[0] - pick[2], self.goal[1] - pick[3]) > 1.5:
                self.goal_since = self.t
            self.goal = (pick[2], pick[3])
            self._route_to(trav, parent, pick[4])
            return True
        return self._go_home(x, y, trav, cost, parent)

    def _go_home(self, x, y, trav, cost, parent):
        self.state = "returning"
        target = self._nearest_traversable(trav, self.grid.to_cell(*self.home))
        if math.hypot(x - self.home[0], y - self.home[1]) < 0.9:
            self._finish()
            return True
        if target is None or not math.isfinite(cost[target[1], target[0]]):
            return False
        self._route_to(trav, parent, target)
        return True

    def _finish(self):
        self.state, self.goal, self.path = "complete", None, []
        self.ctl.set_route([])

    def _plan(self, x, y):
        if self.goal and math.hypot(x - self.goal[0], y - self.goal[1]) < 1.2:   # reached but the frontier is still there: unresolvable
            self.blacklist.append((self.goal[0], self.goal[1], self.t + 600))
            self.goal = None
        if self.goal and self.t - self.goal_since > 60:                          # taking too long
            self.blacklist.append((self.goal[0], self.goal[1], self.t + 300))
            self.goal = None
        for infl in self.inflations:
            if self._try_plan(x, y, infl):
                return
        self._finish()

    # -- main entry: call at ~5-10 Hz with the pose used for planning
    def update(self, dt, x, y):
        if self.last_pos is None:
            self.last_pos, self.prev = (x, y), (x, y)
        self.t += dt
        self.since_replan += dt
        self.since_stuck += dt
        self.cov_timer += dt
        self.travelled += math.hypot(x - self.prev[0], y - self.prev[1])
        self.prev = (x, y)
        if self.cov_timer >= 1.0:
            self.coverage, self.cov_timer = self._coverage(), 0.0
            if self.coverage > self.cov_best + 0.003:
                self.cov_best, self.cov_best_at = self.coverage, self.t
            elif not self.finishing and self.t - self.cov_best_at > 150:
                self.finishing = True                                           # < 0.3 % new coverage in 150 s
        if self.state == "complete":
            return
        if self.since_stuck >= self.stuck_check:
            moved = math.hypot(x - self.last_pos[0], y - self.last_pos[1])
            if moved < 0.25 and not self.ctl.is_done() and self.goal:
                self.blacklist.append((self.goal[0], self.goal[1], self.t + 60))
                self.since_replan = self.replan_every
            self.last_pos, self.since_stuck = (x, y), 0.0
        if self.since_replan >= self.replan_every or (self.ctl.is_done() and self.since_replan > 0.3):
            self._plan(x, y)
            self.since_replan = 0.0


def integrate_level_cloud(grid, x, y, yaw, level_xyz, sensor_height=0.972, obstacle_min=0.15, obstacle_max=1.7):
    """Integrate one LiDAR sweep (gravity-aligned, sensor frame, see cloud_utils.level_points) into the occupancy grid.

    Obstacles = vertical structure only (cells whose points span > 25 cm in height) between obstacle_min and obstacle_max above the floor,
    so floors, slopes and ceilings never block the robot. Rays that climb above the band only carve free space up to where they leave it."""
    from .cloud_utils import vertical_structure_mask
    if len(level_xyz) == 0:
        return 0
    z = level_xyz[:, 2]
    z_hi = obstacle_max - sensor_height
    c, s = math.cos(yaw), math.sin(yaw)
    origin = np.array([x, y])
    end = np.stack([c * level_xyz[:, 0] - s * level_xyz[:, 1] + x, s * level_xyz[:, 0] + c * level_xyz[:, 1] + y], axis=1)
    obstacle = vertical_structure_mask(level_xyz, z_min=obstacle_min - sensor_height, z_max=z_hi)
    frac = np.ones(len(z))
    above = z > z_hi
    frac[above] = np.clip(z_hi / np.maximum(z[above], 1e-3), 0.0, 1.0)
    end = origin + (end - origin) * frac[:, None]
    grid.integrate((x, y), end, obstacle)
    return int(obstacle.sum())
