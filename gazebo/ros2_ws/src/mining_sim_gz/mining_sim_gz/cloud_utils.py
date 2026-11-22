"""PointCloud2 <-> numpy helpers and the geometry shared by the SLAM / mapping / segmentation nodes (ROS imports are lazy)."""
import math

import numpy as np

_DTYPES = {1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16, 5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64}


def pointcloud2_to_xyz(msg, max_range=None):
    """Finite (x, y, z) points of a sensor_msgs/PointCloud2 as float64 (n, 3), in the sensor frame."""
    offs = {f.name: f for f in msg.fields}
    n = msg.width * msg.height
    if n == 0:
        return np.zeros((0, 3))
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n, msg.point_step)
    cols = []
    for name in ("x", "y", "z"):
        f = offs[name]
        dt = np.dtype(_DTYPES[f.datatype]).newbyteorder(">" if msg.is_bigendian else "<")
        cols.append(buf[:, f.offset:f.offset + dt.itemsize].copy().view(dt).reshape(n).astype(np.float64))
    xyz = np.stack(cols, axis=1)
    ok = np.isfinite(xyz).all(axis=1)
    if max_range is not None:
        ok &= np.linalg.norm(xyz, axis=1) <= max_range
    return xyz[ok]


def make_cloud(header, xyz, rgb=None):
    """Build a PointCloud2 (x, y, z[, rgb]) from (n, 3) points and optional (n, 3) uint8 colours."""
    from sensor_msgs.msg import PointCloud2, PointField
    xyz = np.asarray(xyz, dtype=np.float32).reshape(-1, 3)
    fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate("xyz")]
    if rgb is None:
        data, step = xyz.tobytes(), 12
    else:
        packed = (rgb[:, 0].astype(np.uint32) << 16 | rgb[:, 1].astype(np.uint32) << 8 | rgb[:, 2].astype(np.uint32)).astype(np.uint32)
        arr = np.zeros(len(xyz), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<u4")])
        arr["x"], arr["y"], arr["z"], arr["rgb"] = xyz[:, 0], xyz[:, 1], xyz[:, 2], packed
        fields.append(PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1))
        data, step = arr.tobytes(), 16
    msg = PointCloud2()
    msg.header = header
    msg.height, msg.width, msg.fields = 1, len(xyz), fields
    msg.is_bigendian, msg.point_step, msg.row_step = False, step, step * len(xyz)
    msg.data, msg.is_dense = data, True
    return msg


def quat_to_matrix(x, y, z, w):
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)],
    ])


def level_points(xyz, quat=None):
    """Rotate sensor-frame points into a gravity-aligned frame that keeps the robot heading (x forward).

    quat = IMU orientation (x, y, z, w) of the sensor in the world. Without it the sensor is assumed level.
    Removes the roll / pitch the robot picks up on a slope so floor and wall heights stay meaningful."""
    xyz = np.asarray(xyz, dtype=float).reshape(-1, 3)
    if quat is None:
        return xyz
    R = quat_to_matrix(*quat)
    yaw = math.atan2(R[1, 0], R[0, 0])
    c, s = math.cos(-yaw), math.sin(-yaw)
    Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
    return xyz @ (Rz @ R).T


def relative_heights(level_xyz, cell=1.0):
    """Height of each point above the lowest point of its 1 m x 1 m cell (a floor-height estimate that works on slopes)."""
    if len(level_xyz) == 0:
        return np.zeros(0)
    ij = np.floor(level_xyz[:, :2] / cell).astype(np.int64)
    key = (ij[:, 0] + 100000) * 200003 + (ij[:, 1] + 100000)
    uniq, inv = np.unique(key, return_inverse=True)
    mins = np.full(len(uniq), np.inf)
    np.minimum.at(mins, inv.reshape(-1), level_xyz[:, 2])
    return level_xyz[:, 2] - mins[inv.reshape(-1)]


def scan_points_2d(level_xyz, rel_z, min_rel=0.25, max_rel=2.0, max_range=20.0, max_points=700):
    """2D points (x, y) used for scan matching: above the floor, below the ceiling, not too far, subsampled."""
    r = np.hypot(level_xyz[:, 0], level_xyz[:, 1])
    ok = (rel_z >= min_rel) & (rel_z <= max_rel) & (r <= max_range)
    pts = level_xyz[ok, :2]
    if len(pts) > max_points:
        pts = pts[:: int(math.ceil(len(pts) / max_points))]
    return pts


def vertical_structure_mask(level_xyz, cell=0.25, min_extent=0.25, z_min=-1.2, z_max=0.75):
    """Points that belong to VERTICAL structure (walls, pillars, boulders), found with a height-difference test.

    A 0.25 m x 0.25 m cell counts as vertical structure when the points inside it span more than `min_extent` in height. Floors and
    ceilings never do (even a 30 degree slope spans only ~0.15 m per cell), so this needs no floor-height estimate and works on slopes.
    Only points between z_min and z_max (sensor frame, i.e. below roughly 1.7 m above the floor) are kept: the ceiling is excluded."""
    if len(level_xyz) == 0:
        return np.zeros(0, dtype=bool)
    ij = np.floor(level_xyz[:, :2] / cell).astype(np.int64)
    key = (ij[:, 0] + 100000) * 200003 + (ij[:, 1] + 100000)
    uniq, inv = np.unique(key, return_inverse=True)
    inv = inv.reshape(-1)
    lo = np.full(len(uniq), np.inf)
    hi = np.full(len(uniq), -np.inf)
    zc = np.clip(level_xyz[:, 2], z_min, z_max)
    inband = (level_xyz[:, 2] >= z_min) & (level_xyz[:, 2] <= z_max)
    np.minimum.at(lo, inv[inband], zc[inband])
    np.maximum.at(hi, inv[inband], zc[inband])
    return inband & ((hi - lo)[inv] >= min_extent)


def scan_points_2d_vertical(level_xyz, max_range=20.0, max_points=700, **kw):
    """2D scan-matching points = vertical structure only (see vertical_structure_mask), within range, subsampled."""
    ok = vertical_structure_mask(level_xyz, **kw) & (np.hypot(level_xyz[:, 0], level_xyz[:, 1]) <= max_range)
    pts = level_xyz[ok, :2]
    if len(pts) > max_points:
        pts = pts[:: int(math.ceil(len(pts) / max_points))]
    return pts
