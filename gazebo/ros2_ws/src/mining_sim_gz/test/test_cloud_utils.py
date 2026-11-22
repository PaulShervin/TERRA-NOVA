"""Unit tests for the point-cloud helpers (no ROS needed: a fake message object stands in for sensor_msgs/PointCloud2)."""
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mining_sim_gz.cloud_utils import level_points, pointcloud2_to_xyz, relative_heights, scan_points_2d  # noqa: E402


def fake_cloud(xyz, extra_field=True):
    n = len(xyz)
    dt = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4")] + ([("intensity", "<f4")] if extra_field else []))
    arr = np.zeros(n, dtype=dt)
    arr["x"], arr["y"], arr["z"] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    fields = [SimpleNamespace(name=nm, offset=4 * i, datatype=7, count=1) for i, nm in enumerate(["x", "y", "z"])]
    return SimpleNamespace(fields=fields, width=n, height=1, point_step=dt.itemsize, is_bigendian=False, data=arr.tobytes())


def test_parse_drops_non_finite_and_far_points():
    xyz = np.array([[1, 2, 3], [np.inf, 0, 0], [np.nan, 1, 1], [30, 0, 0], [0, 4, -1.0]])
    out = pointcloud2_to_xyz(fake_cloud(xyz), max_range=25.0)
    assert out.shape == (2, 3)
    assert np.allclose(out, [[1, 2, 3], [0, 4, -1]])


def quat_pitch(angle):
    return (0.0, math.sin(angle / 2), 0.0, math.cos(angle / 2))


def test_level_points_removes_pitch():
    pitch = math.radians(20)
    # a flat floor seen from a robot pitched up 20 degrees: the floor point (3, 0, -1) in the world is rotated into the body frame
    world = np.array([[3.0, 0.0, -1.0], [3.0, 1.0, -1.0]])
    c, s = math.cos(pitch), math.sin(pitch)
    R = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])      # body -> world
    body = world @ R                                        # = (R^T @ p) for each row
    leveled = level_points(body, quat_pitch(pitch))
    assert np.allclose(leveled, world, atol=1e-9)


def test_relative_heights_and_band():
    floor = np.stack([np.linspace(0.1, 0.9, 9), np.full(9, 0.5), np.full(9, -1.0)], axis=1)
    wall = np.array([[0.5, 0.5, -0.5], [0.5, 0.5, 0.2], [0.5, 0.5, 1.5]])
    pts = np.vstack([floor, wall])
    rel = relative_heights(pts)
    assert np.allclose(rel[:9], 0.0) and np.allclose(rel[9:], [0.5, 1.2, 2.5])
    xy = scan_points_2d(pts, rel)                           # band 0.25..2.0 keeps the first two wall points only
    assert len(xy) == 2


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
