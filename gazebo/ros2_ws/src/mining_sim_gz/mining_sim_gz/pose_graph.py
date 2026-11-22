"""SE(2) pose-graph optimisation (Gauss-Newton with a sparse direct solve). Port of src/slam/PoseGraph.js.

Nodes are keyframe poses (x, y, th); an edge (i, j, z, info) says "j is z away from i" with diagonal information `info`.
"""
import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.linalg import spsolve


def _wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def optimize_pose_graph(poses, edges, iterations=8, anchor_weight=1e6):
    n = len(poses)
    x = np.array(poses, dtype=float).reshape(n, 3).copy()
    x0 = x[0].copy()
    for _ in range(iterations):
        rows, cols, vals = [], [], []
        b = np.zeros(3 * n)

        def put(i, j, block):
            for r in range(3):
                for c in range(3):
                    rows.append(3 * i + r); cols.append(3 * j + c); vals.append(block[r, c])

        for (i, j, z, info) in edges:
            c, s = np.cos(x[i, 2]), np.sin(x[i, 2])
            dx, dy = x[j, 0] - x[i, 0], x[j, 1] - x[i, 1]
            rx, ry = c * dx + s * dy, -s * dx + c * dy
            err = np.array([rx - z[0], ry - z[1], _wrap(x[j, 2] - x[i, 2] - z[2])])
            Ji = np.array([[-c, -s, ry], [s, -c, -rx], [0, 0, -1.0]])
            Jj = np.array([[c, s, 0], [-s, c, 0], [0, 0, 1.0]])
            O = np.diag(info)
            b[3 * i:3 * i + 3] += Ji.T @ O @ err
            b[3 * j:3 * j + 3] += Jj.T @ O @ err
            put(i, i, Ji.T @ O @ Ji); put(j, j, Jj.T @ O @ Jj)
            put(i, j, Ji.T @ O @ Jj); put(j, i, Jj.T @ O @ Ji)
        for k in range(3):
            rows.append(k); cols.append(k); vals.append(anchor_weight)
            b[k] += anchor_weight * (x[0, k] - x0[k])
        H = coo_matrix((vals, (rows, cols)), shape=(3 * n, 3 * n)).tocsc()
        delta = spsolve(H, -b).reshape(n, 3)
        x += delta
        if np.abs(delta).max() < 1e-5:
            break
    x[:, 2] = _wrap(x[:, 2])
    return x
