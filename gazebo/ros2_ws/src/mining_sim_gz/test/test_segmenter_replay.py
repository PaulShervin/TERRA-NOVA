"""Cross-language check of the label-free segmenter: same map (scripts/dump-map-log.mjs), Python vs JS score.
Run: python test/test_segmenter_replay.py <map_log.json>     or     SLAM_MAP_LOG=<json> python -m pytest test/test_segmenter_replay.py
"""
import json
import os
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mining_sim_gz.segmenter import evaluate_segmentation, segment_points  # noqa: E402


def run(path):
    log = json.loads(Path(path).read_text())
    pts = np.array(log["pts"]).reshape(-1, 3)
    t0 = time.time()
    labels, nv = segment_points(pts, np.array(log["viewpoints"]).reshape(-1, 3))
    return evaluate_segmentation(labels, np.array(log["cat"])), log["js"], time.time() - t0, len(pts), nv


def test_segmenter_matches_js():
    path = os.environ.get("SLAM_MAP_LOG")
    if not path or not Path(path).exists():
        import pytest
        pytest.skip("set SLAM_MAP_LOG to a recording made by scripts/dump-map-log.mjs")
    py, js, *_ = run(path)
    assert py["accuracy"] > js["accuracy"] - 0.03, (py["accuracy"], js["accuracy"])


if __name__ == "__main__":
    py, js, secs, n, nv = run(sys.argv[1])
    print(f"{n} points / {nv} voxels in {secs:.2f} s")
    print(f"python: accuracy {py['accuracy'] * 100:.1f}%  mIoU {py['mean_iou'] * 100:.1f}%  " + "  ".join(f"{k} {v['iou'] or 0:.2f}" for k, v in py["per_class"].items()))
    print(f"js    : accuracy {js['accuracy'] * 100:.1f}%  mIoU {js['meanIoU'] * 100:.1f}%  " + "  ".join(f"{c['name']} {c['iou'] or 0:.2f}" for c in js["perClass"]))
