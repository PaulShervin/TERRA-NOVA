"""2D point-to-line ICP (Gauss-Newton, Huber weights) with an optional Gaussian prior on the pose.

Port of src/slam/ScanMatcher.js.
  residual  r_i = n_i . (R(theta) p_i + t - q_i)
  solve     (H + P) d = -(g + P (x - x_pred))      P = diag(prior information)
The prior (odometry) keeps the solution bounded where the scan cannot observe the motion (long straight tunnels).
`hmin` (smallest eigenvalue of the 2x2 translation block of H) measures that observability.
"""
from dataclasses import dataclass

import numpy as np
from scipy.spatial import cKDTree


@dataclass
class Reference:
    xy: np.ndarray       # (k, 2) points that have a clean local line
    normals: np.ndarray  # (k, 2)
    tree: object         # cKDTree over xy, or None


def build_reference(xy):
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    n = len(xy)
    empty = Reference(np.zeros((0, 2)), np.zeros((0, 2)), None)
    if n < 6:
        return empty
    k = min(24, n)
    d, idx = cKDTree(xy).query(xy, k=k, distance_upper_bound=0.6)
    mask = np.isfinite(d)
    cnt = mask.sum(axis=1)
    pts = xy[np.where(mask, idx, 0)]                   # (n, k, 2)
    w = mask[..., None]
    mean = (pts * w).sum(axis=1) / np.maximum(cnt, 1)[:, None]
    dev = (pts - mean[:, None, :]) * w
    c = np.maximum(cnt, 1)
    sxx = (dev[..., 0] ** 2).sum(axis=1) / c
    sxy = (dev[..., 0] * dev[..., 1]).sum(axis=1) / c
    syy = (dev[..., 1] ** 2).sum(axis=1) / c
    half = (sxx + syy) / 2
    rad = np.sqrt(((sxx - syy) / 2) ** 2 + sxy ** 2)
    l1, l2 = half + rad, np.maximum(half - rad, 0.0)
    valid = (cnt >= 5) & (np.sqrt(l2) <= 0.07) & (l1 >= 1e-4)
    vx, vy = sxy.copy(), l2 - sxx                      # eigenvector of the smaller eigenvalue = line normal
    degenerate = (np.abs(vx) + np.abs(vy)) < 1e-9
    vx = np.where(degenerate, np.where(sxx < syy, 1.0, 0.0), vx)
    vy = np.where(degenerate, np.where(sxx < syy, 0.0, 1.0), vy)
    norm = np.maximum(np.hypot(vx, vy), 1e-12)
    normals = np.stack([vx / norm, vy / norm], axis=1)[valid]
    vxy = xy[valid]
    if len(vxy) < 3:
        return empty
    return Reference(vxy, normals, cKDTree(vxy))


@dataclass
class IcpResult:
    pose: np.ndarray
    fitness: float
    rms: float
    hmin: float
    inliers: int


def icp(ref, src, init, prior=None, iters=25, gate=0.5, sigma=0.05):
    """Align src (m,2, sensor frame) to ref starting from init=(x,y,th). prior = dict(pose=(x,y,th), wT, wR[, dir, wT_along])."""
    src = np.asarray(src, dtype=float).reshape(-1, 2)
    m = len(src)
    x, y, th = float(init[0]), float(init[1]), float(init[2])
    if ref.tree is None or m == 0:
        return IcpResult(np.array([x, y, th]), 0.0, 1.0, 0.0, 0)
    inv = 1.0 / (sigma * sigma)
    hmin, inliers, rssq = 0.0, 0, 0.0
    px, py = src[:, 0], src[:, 1]
    for it in range(iters):
        c, s = np.cos(th), np.sin(th)
        wx, wy = c * px - s * py + x, s * px + c * py + y
        g = gate if it < iters / 2 else gate * 0.5
        d, j = ref.tree.query(np.stack([wx, wy], axis=1), distance_upper_bound=g)
        ok = np.isfinite(d)
        if not ok.any():
            inliers, rssq = 0, 0.0
            break
        jj = j[ok]
        nx, ny = ref.normals[jj, 0], ref.normals[jj, 1]
        r = nx * (wx[ok] - ref.xy[jj, 0]) + ny * (wy[ok] - ref.xy[jj, 1])
        ar = np.abs(r)
        wt = np.where(ar < 0.1, 1.0, 0.1 / np.maximum(ar, 1e-12)) * inv
        j2 = nx * (-s * px[ok] - c * py[ok]) + ny * (c * px[ok] - s * py[ok])
        J = np.stack([nx, ny, j2], axis=1)
        H = (J * wt[:, None]).T @ J
        b = (J * (wt * r)[:, None]).sum(axis=0)
        inliers = int((ar < 0.12).sum())
        rssq = float((r * r).sum())
        a, bb, cc = H[0, 0], H[0, 1], H[1, 1]
        hmin = (a + cc) / 2 - np.sqrt(((a - cc) / 2) ** 2 + bb * bb)
        if prior is not None:
            P2 = np.eye(2) * prior["wT"]
            if prior.get("dir") is not None:                      # anisotropic: tight along the direction of motion, loose across
                u = np.asarray(prior["dir"], dtype=float)
                P2 = prior["wT"] * np.eye(2) + (prior["wT_along"] - prior["wT"]) * np.outer(u, u)
            H[:2, :2] += P2
            H[2, 2] += prior["wR"]
            b[:2] += P2 @ np.array([x - prior["pose"][0], y - prior["pose"][1]])
            b[2] += prior["wR"] * (th - prior["pose"][2])
        H = H + np.eye(3) * 1e-6
        try:
            dlt = -np.linalg.solve(H, b)
        except np.linalg.LinAlgError:
            break
        x, y, th = x + dlt[0], y + dlt[1], th + dlt[2]
        if np.abs(dlt).sum() < 1e-4:
            break
    th = (th + np.pi) % (2 * np.pi) - np.pi
    return IcpResult(np.array([x, y, th]), inliers / m, float(np.sqrt(rssq / max(inliers, 1))) if inliers else 1.0, float(hmin), inliers)
