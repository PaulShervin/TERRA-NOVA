"""Ground-truth semantic classes for scoring the label-free segmentation in Gazebo (never used to segment).

Classes: 0 floor, 1 ceiling, 2 wall, 3 object. Floor / ceiling / wall come from the surface normal of the nearest vertex of the exported
cave mesh; object = inside the footprint of a pillar / boulder / stalactite from cave_info.json (same rules as src/cave/CaveLidar.js).
"""
import json

import numpy as np
from scipy.spatial import cKDTree


def load_obj(path):
    """Vertices and vertex normals of an OBJ file (v / vn lines only)."""
    v, vn = [], []
    with open(path) as fh:
        for line in fh:
            if line.startswith("v "):
                v.append(line.split()[1:4])
            elif line.startswith("vn "):
                vn.append(line.split()[1:4])
    return np.array(v, dtype=float), np.array(vn, dtype=float)


class CaveGroundTruth:
    def __init__(self, obj_path, info_path):
        verts, normals = load_obj(obj_path)
        self.tree, self.normals = cKDTree(verts), normals
        self.formations = json.load(open(info_path))["formations"]

    def classify(self, pts):
        pts = np.asarray(pts, dtype=float).reshape(-1, 3)
        _, nearest = self.tree.query(pts)
        nz = self.normals[nearest, 2]
        cls = np.full(len(pts), 2, dtype=np.uint8)
        cls[nz > 0.7] = 0
        cls[nz < -0.5] = 1
        x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
        obj = np.zeros(len(pts), dtype=bool)
        for f in self.formations:
            if f["kind"] == "pillar":
                obj |= (np.hypot(x - f["x"], y - f["y"]) < f["radius"] * 1.4 + 0.35) & (z > f["zFloor"] - 0.5) & (z < f["zFloor"] + f["height"] + 0.5)
            elif f["kind"] == "boulder":
                co, si = np.cos(f["yaw"]), np.sin(f["yaw"])
                dx, dy = x - f["x"], y - f["y"]
                lx, ly, lz = (co * dx + si * dy) / f["radii"][0], (-si * dx + co * dy) / f["radii"][1], (z - f["z"]) / f["radii"][2]
                obj |= np.sqrt(lx * lx + ly * ly + lz * lz) < 1.45
            else:  # stalactite
                obj |= (np.hypot(x - f["x"], y - f["y"]) < f["radius"] * 2.2 + 0.25) & (z > f["zCeil"] - f["length"] - 0.4) & (nz < 0.35)
        cls[obj] = 3
        return cls
