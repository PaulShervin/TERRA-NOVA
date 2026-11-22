"""Replay a recording through the occupancy mapper (true poses) and compare with the ground-truth free-space mask.  python analyze_map.py <rec.npz>"""
import sys, json, math, numpy as np
sys.path.insert(0, "../ros2_ws/src/mining_sim_gz")
from mining_sim_gz.cloud_utils import level_points
from mining_sim_gz.explorer_core import OccupancyGrid, integrate_level_cloud, FREE, OCC, build_traversable, find_frontiers, dijkstra_grid
cfg = "../ros2_ws/src/mining_sim_gz/config/"
info = json.load(open(cfg + "cave_info.json")); m = json.load(open(cfg + "truth_cave.json")); b = info["bounds"]
truth = np.fromfile(cfg + "truth_cave.bin", dtype=np.uint8).reshape(m["H"], m["W"]).astype(bool)
d = np.load(sys.argv[1], allow_pickle=True); t, gt, imu, xyz = d["t"], d["gt"], d["imu"], d["xyz"]
g = OccupancyGrid(b["xMin"], b["xMax"], b["yMin"], b["yMax"], m["res"])
for i in range(len(t)):
    integrate_level_cloud(g, gt[i, 0], gt[i, 1], gt[i, 3], level_points(xyz[i].astype(float), tuple(imu[i])))
st = g.state()
seen = (st != 0)
print(f"mapped cells {seen.sum()} | truth-free cells marked FREE {100*(truth&(st==FREE)).sum()/max(1,(truth&seen).sum()):.1f}% of truth cells seen, marked OCC (wrong) {100*(truth&(st==OCC)).sum()/max(1,(truth&seen).sum()):.1f}%")
print(f"obstacle cells outside the drivable mask (walls etc.): {(st==OCC).sum() - (truth&(st==OCC)).sum()} | total OCC {(st==OCC).sum()}")
x, y = gt[-1, 0], gt[-1, 1]
trav, _ = build_traversable(g, 0.5, (x, y))
start = g.to_cell(x, y); cost, par = dijkstra_grid(trav, g.res, start)
cl = find_frontiers(trav, st)
reach = [c for c in cl if np.isfinite(cost[c[:, 0], c[:, 1]]).any()]
print(f"final pose ({x:.1f},{y:.1f}) traversable cells {trav.sum()} reachable {np.isfinite(cost).sum()} | frontier clusters {len(cl)} reachable {len(reach)}")
