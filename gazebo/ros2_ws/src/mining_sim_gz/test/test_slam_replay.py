"""Cross-language check: replay a JS-recorded run (scripts/dump-slam-log.mjs) through the Python estimator.

Run:  python -m pytest test/test_slam_replay.py        (set SLAM_LOG to the recorded json)
or    python test/test_slam_replay.py <log.json>
The odometry increments and scan points are identical to what the JS estimator saw, so the errors should be comparable.
"""
import json
import os
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mining_sim_gz.se2 import between  # noqa: E402
from mining_sim_gz.slam_core import SlamEstimator  # noqa: E402


def replay(path):
    log = json.loads(Path(path).read_text())
    slam = SlamEstimator()
    slam.init(log["start"])
    prev = np.array(log["start"], dtype=float)
    t0 = time.time()
    for s in log["samples"]:
        odom = np.array(s["odom"])
        slam.update(between(prev, odom), np.array(s["pts"]).reshape(-1, 2), s["t"], truth=s["truth"])
        prev = odom
    return slam.metrics(), log["js"], time.time() - t0, len(log["samples"])


def test_replay_matches_js():
    path = os.environ.get("SLAM_LOG")
    if not path or not Path(path).exists():
        import pytest
        pytest.skip("set SLAM_LOG to a recording made by scripts/dump-slam-log.mjs")
    py, js, _, _ = replay(path)
    assert py["ate"] < py["ate_odom"], "SLAM must beat odometry alone"
    assert py["ate"] < max(1.5 * js["ate"], js["ate"] + 0.6), (py["ate"], js["ate"])


if __name__ == "__main__":
    py, js, secs, n = replay(sys.argv[1])
    print(f"{n} sweeps replayed in {secs:.1f} s ({secs / n * 1000:.0f} ms / sweep)")
    print(f"python: ATE slam {py['ate']:.2f} m  odom {py['ate_odom']:.2f} m | RPE slam {py['rpe']:.2f} odom {py['rpe_odom']:.2f} | "
          f"final slam {py['final_error']:.2f} odom {py['final_error_odom']:.2f} | loops {py['loop_closures']} (rejected {py['rejected_loops']}) kf {py['keyframes']}")
    print(f"js    : ATE slam {js['ate']:.2f} m  odom {js['ateOdom']:.2f} m | RPE slam {js['rpe']:.2f} odom {js['rpeOdom']:.2f} | "
          f"final slam {js['finalError']:.2f} odom {js['finalErrorOdom']:.2f} | loops {js['loopClosures']} kf {js['keyframes']}")
