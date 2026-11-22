"""Offline analysis of a recording made by record_run.py: SLAM replay on REAL Gazebo clouds + obstacle-map pollution check.
python analyze_rec.py <rec.npz> [old|new]"""
import sys, math, numpy as np
sys.path.insert(0, "../ros2_ws/src/mining_sim_gz")
from mining_sim_gz.cloud_utils import level_points, relative_heights, scan_points_2d, scan_points_2d_vertical, vertical_structure_mask
from mining_sim_gz.se2 import between
from mining_sim_gz.slam_core import SlamEstimator, odometry_noise_model

d = np.load(sys.argv[1], allow_pickle=True)
mode = sys.argv[2] if len(sys.argv) > 2 else "new"
t, gt, od, imu, xyz = d["t"], d["gt"], d["odom"], d["imu"], d["xyz"]
# floor points flagged as obstacles by each method (floor = points within 0.15 m of the lowest 5 % of the cloud per sweep, a rough proxy)
flag_old = flag_new = tot = 0
for i in range(0, len(t), 5):
    lvl = level_points(xyz[i].astype(float), tuple(imu[i]))
    floor_like = lvl[:, 2] < np.percentile(lvl[:, 2], 10) + 0.12
    rel = relative_heights(lvl)
    flag_old += int(((rel >= 0.15) & (rel <= 1.7) & floor_like).sum())
    flag_new += int((vertical_structure_mask(lvl) & floor_like).sum())
    tot += int(floor_like.sum())
print(f"floor-like points wrongly marked as obstacle: old height-band {100*flag_old/tot:.1f}%  new height-difference {100*flag_new/tot:.1f}%")
slam = SlamEstimator(); slam.init(gt[0, [0, 1, 3]])
rng = np.random.default_rng(1); prev = np.zeros(3); noisy = np.zeros(3)
from mining_sim_gz.se2 import compose
prev_ideal = od[0, [0, 1, 3]]
for i in range(len(t)):
    cur = od[i, [0, 1, 3]]
    delta = odometry_noise_model(between(prev_ideal, cur), rng); prev_ideal = cur
    lvl = level_points(xyz[i].astype(float), tuple(imu[i]))
    pts = scan_points_2d(lvl, relative_heights(lvl)) if mode == "old" else scan_points_2d_vertical(lvl)
    slam.update(delta, pts, t[i] - t[0], truth=gt[i, [0, 1, 3]])
m = slam.metrics()
print(f"[{mode}] {len(t)} sweeps: ATE slam {m['ate']:.2f} m vs odom-only {m['ate_odom']:.2f} m | final err slam {m['final_error']:.2f} vs odom {m['final_error_odom']:.2f} | loops {m['loop_closures']} kf {m['keyframes']} | mean match fitness {np.mean([s.fit for s in slam.samples]):.2f}")
