"""Closed-loop check of the ported mapping + exploration stack on a synthetic 2D world (no ROS, no Gazebo):
simulated 2D LiDAR -> log-odds grid -> frontiers -> Dijkstra -> controller -> kinematic robot, until the explorer says 'complete'.

Run:  python test/test_explorer_closed_loop.py   (prints coverage / time)   or   python -m pytest test/test_explorer_closed_loop.py
"""
import math
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mining_sim_gz.explorer_core import FrontierExplorer, OccupancyGrid, WaypointController  # noqa: E402

RES_W = 0.05  # world raster resolution


def make_world():
    """30 x 20 m: outer wall, three rooms joined by doorways, two pillars and a dead-end corridor."""
    W, H = int(30 / RES_W), int(20 / RES_W)
    occ = np.zeros((H, W), dtype=bool)
    def box(x0, y0, x1, y1):
        occ[int(y0 / RES_W):int(y1 / RES_W), int(x0 / RES_W):int(x1 / RES_W)] = True
    box(0, 0, 30, 0.3); box(0, 19.7, 30, 20); box(0, 0, 0.3, 20); box(29.7, 0, 30, 20)
    box(10, 0, 10.3, 8); box(10, 11, 10.3, 20)            # wall with a 3 m doorway
    box(20, 0, 20.3, 4); box(20, 7, 20.3, 20)             # second wall, doorway at y 4..7
    box(5, 6, 6, 7); box(14, 12, 15, 13)                  # pillars
    box(22, 10, 29.7, 10.3); box(22, 13, 29.7, 13.3)      # dead-end corridor (y 10.3..13) opening to the west at x=22
    return occ


def cast(occ, x, y, n=360, max_range=12.0, rng=None):
    ang = np.linspace(0, 2 * np.pi, n, endpoint=False)
    t = np.arange(0.1, max_range, RES_W)
    px = x + np.cos(ang)[:, None] * t[None, :]
    py = y + np.sin(ang)[:, None] * t[None, :]
    ix, iy = (px / RES_W).astype(int), (py / RES_W).astype(int)
    inside = (ix >= 0) & (iy >= 0) & (ix < occ.shape[1]) & (iy < occ.shape[0])
    hit = np.zeros_like(inside)
    hit[inside] = occ[iy[inside], ix[inside]]
    hit |= ~inside
    first = np.where(hit.any(axis=1), hit.argmax(axis=1), len(t) - 1)
    hit_any = hit.any(axis=1)
    rr = t[first] + (rng.normal(0, 0.02, n) if rng is not None else 0)
    return np.stack([x + np.cos(ang) * rr, y + np.sin(ang) * rr], axis=1), hit_any


def run(max_time=900.0, seed=3, verbose=False):
    occ, rng = make_world(), np.random.default_rng(seed)
    grid = OccupancyGrid(0, 30, 0, 20, 0.25)
    # ground truth reachable free space (cells at least 0.3 m from any wall) for the coverage metric
    from scipy import ndimage
    dist = ndimage.distance_transform_edt(~occ) * RES_W
    truth = np.zeros((grid.H, grid.W), dtype=bool)
    for j in range(grid.H):
        for i in range(grid.W):
            x, y = grid.to_world(i, j)
            truth[j, i] = dist[int(y / RES_W), int(x / RES_W)] > 0.3
    home = (2.0, 2.0)
    pose = np.array([home[0], home[1], 0.0])
    ctl = WaypointController(v_max=0.8, tol=0.25)
    ex = FrontierExplorer(grid, ctl, home, truth_mask=truth, inflations=(0.5, 0.4, 0.3))
    t, dt, scan_dt, since = 0.0, 0.05, 0.2, 10.0
    collided = 0
    t0 = time.time()
    while t < max_time and ex.state != "complete":
        if since >= scan_dt:
            ends, hit = cast(occ, pose[0], pose[1], rng=rng)
            grid.integrate((pose[0], pose[1]), ends, hit)
            ex.update(scan_dt, pose[0], pose[1])
            since = 0.0
        v, w = ctl.step(*pose)
        pose[2] += w * dt
        nx, ny = pose[0] + v * math.cos(pose[2]) * dt, pose[1] + v * math.sin(pose[2]) * dt
        if dist[int(ny / RES_W), int(nx / RES_W)] < 0.3:
            collided += 1
        else:
            pose[0], pose[1] = nx, ny
        t += dt
        since += dt
        if verbose and int(t / dt) % 400 == 0:
            print(f"t={t:5.0f}s state={ex.state} cov={ex.coverage * 100:4.1f}% frontiers={ex.frontiers} pos=({pose[0]:.1f},{pose[1]:.1f})")
    return dict(time=t, state=ex.state, coverage=ex.coverage, travelled=ex.travelled, collided=collided,
                home_err=math.hypot(pose[0] - home[0], pose[1] - home[1]), wall_clock=time.time() - t0)


def test_explores_and_returns_home():
    r = run()
    assert r["state"] == "complete", r
    assert r["coverage"] > 0.9, r
    assert r["home_err"] < 1.5, r
    assert r["collided"] == 0, r


if __name__ == "__main__":
    print(run(verbose=True))
