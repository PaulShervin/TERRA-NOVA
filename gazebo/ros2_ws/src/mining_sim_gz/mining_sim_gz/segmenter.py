"""Label-free semantic segmentation of a LiDAR map from local geometry only (port of src/slam/Segmenter.js).

Per 0.3 m voxel: PCA of the 3x3x3 neighbourhood -> surface normal, oriented toward the nearest sensor position ->
  normal.z > 0.7  floor | < -0.5  ceiling | otherwise wall, and wall voxels whose neighbourhood bulges toward the sensor
  (convex, >= 6 connected voxels) become 'object'. Ground-truth categories are used only to SCORE the result.
"""
import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree

CLASSES = ("floor", "ceiling", "wall", "object")
VOXEL = 0.3
OFFSETS = [(a, b, c) for a in (-1, 0, 1) for b in (-1, 0, 1) for c in (-1, 0, 1)]


def segment_points(pts, viewpoints, min_neighbours=6, floor_nz=0.7, ceiling_nz=-0.5, convexity=-0.02, min_object_voxels=6):
    """pts (n,3) map points; viewpoints (m,3) sensor positions. Returns (labels (n,) in 0..3, number of voxels)."""
    pts = np.asarray(pts, dtype=float).reshape(-1, 3)
    cell = np.floor(pts / VOXEL).astype(np.int64)
    uniq, inv, counts = np.unique(cell, axis=0, return_inverse=True, return_counts=True)
    inv = inv.reshape(-1)
    nv = len(uniq)
    cen = np.zeros((nv, 3))
    np.add.at(cen, inv, pts)
    cen /= counts[:, None]

    # neighbour lookup through a sorted integer key
    base = uniq.min(axis=0) - 2
    dims = uniq.max(axis=0) - base + 3
    def key(c):
        d = c - base
        return (d[..., 0] * dims[1] + d[..., 1]) * dims[2] + d[..., 2]
    keys = key(uniq)
    order = np.argsort(keys)
    sorted_keys = keys[order]
    nb_idx = np.full((len(OFFSETS), nv), -1, dtype=np.int64)
    for o, off in enumerate(OFFSETS):
        k = key(uniq + np.array(off))
        pos = np.searchsorted(sorted_keys, k)
        pos = np.minimum(pos, nv - 1)
        found = sorted_keys[pos] == k
        nb_idx[o, found] = order[pos[found]]
    found = nb_idx >= 0
    wts = np.where(found, counts[np.maximum(nb_idx, 0)], 0).astype(float)          # (27, nv)
    npos = cen[np.maximum(nb_idx, 0)]                                              # (27, nv, 3)
    wsum = wts.sum(axis=0)
    mean = (npos * wts[..., None]).sum(axis=0) / np.maximum(wsum, 1)[:, None]
    dev = npos - mean[None]
    cov = np.einsum("on,oni,onj->nij", wts, dev, dev) / np.maximum(wsum, 1)[:, None, None]
    valid = (wsum >= min_neighbours) & (found.sum(axis=0) >= 4)
    evals, evecs = np.linalg.eigh(cov)                                             # ascending
    nrm = evecs[:, :, 0]                                                           # smallest eigenvalue direction

    # orient toward the nearest sensor position, then convexity
    vp = np.asarray(viewpoints, dtype=float).reshape(-1, 3)
    if len(vp):
        _, nearest = cKDTree(vp).query(cen)
        to_sensor = vp[nearest] - cen
    else:
        to_sensor = np.tile([0.0, 0.0, 1.0], (nv, 1))
    sgn = np.where((nrm * to_sensor).sum(axis=1) >= 0, 1.0, -1.0)
    oz = sgn * nrm[:, 2]
    rel = npos - cen[None]
    cvx_terms = (rel * nrm[None]).sum(axis=2) * sgn[None]
    not_self = found.copy()
    not_self[OFFSETS.index((0, 0, 0))] = False
    cvx = (cvx_terms * not_self).sum(axis=0) / np.maximum(not_self.sum(axis=0), 1)

    cls = np.full(nv, 2, dtype=np.uint8)                                           # default wall
    cls[valid & (oz > floor_nz)] = 0
    cls[valid & (oz < ceiling_nz)] = 1
    cls[valid & (oz >= ceiling_nz) & (oz <= floor_nz) & (cvx < convexity)] = 3

    # object voxels must form a blob of >= min_object_voxels connected voxels
    obj = cls == 3
    if obj.any():
        d = uniq - base
        grid = np.zeros(tuple(dims), dtype=bool)
        grid[d[obj, 0], d[obj, 1], d[obj, 2]] = True
        lab, _ = ndimage.label(grid, structure=np.ones((3, 3, 3)))
        sizes = np.bincount(lab.ravel())
        vox_lab = lab[d[:, 0], d[:, 1], d[:, 2]]
        small = obj & (sizes[vox_lab] < min_object_voxels)
        cls[small] = 2
    return cls[inv], nv


def evaluate_segmentation(pred, truth):
    """Confusion matrix, accuracy, per-class IoU against ground-truth classes (0 floor, 1 ceiling, 2 wall, 3 object)."""
    pred, truth = np.asarray(pred), np.asarray(truth)
    conf = np.zeros((4, 4), dtype=np.int64)
    np.add.at(conf, (truth, pred), 1)
    out = {}
    for c, name in enumerate(CLASSES):
        tp, fn, fp = conf[c, c], conf[c].sum() - conf[c, c], conf[:, c].sum() - conf[c, c]
        out[name] = dict(support=int(tp + fn), iou=float(tp / (tp + fn + fp)) if tp + fn + fp else None)
    ious = [v["iou"] for v in out.values() if v["iou"] is not None and v["support"] > 0]
    return dict(accuracy=float(np.trace(conf) / max(conf.sum(), 1)), mean_iou=float(np.mean(ious)) if ious else 0.0, per_class=out, confusion=conf)
