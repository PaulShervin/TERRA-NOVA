// 2D point-to-line ICP (Gauss-Newton, Huber weights) with an optional Gaussian prior on the pose.
//   residual  r_i = n_i . (R(theta) p_i + t - q_i)         p_i: source point (sensor frame), q_i / n_i: nearest reference point + line normal
//   solve     (H + P) d = -(g + P (x - x_pred))            P = diag(prior information): odometry keeps the solution
//                                                          bounded in directions the scan cannot observe (long straight tunnels)
// Also returns the translational information (smallest eigenvalue of the 2x2 block of H): small = the scan is degenerate
// (corridor), which loop-closure uses to refuse ambiguous matches.
const CELL = 0.5;
const key = (ix, iy) => (ix + 4096) * 8192 + (iy + 4096);

/** Reference scan: points (interleaved x,y) in some frame, hashed, with a local line normal per point. */
export function buildReference(xy) {
  const n = xy.length / 2, cells = new Map();
  for (let i = 0; i < n; i++) {
    const k = key(Math.floor(xy[2 * i] / CELL), Math.floor(xy[2 * i + 1] / CELL));
    const l = cells.get(k); if (l) l.push(i); else cells.set(k, [i]);
  }
  const nx = new Float32Array(n), ny = new Float32Array(n), valid = new Uint8Array(n);
  const near = [];
  for (let i = 0; i < n; i++) {
    const px = xy[2 * i], py = xy[2 * i + 1], cx = Math.floor(px / CELL), cy = Math.floor(py / CELL);
    near.length = 0;
    for (let a = -1; a <= 1; a++) for (let b = -1; b <= 1; b++) {
      const l = cells.get(key(cx + a, cy + b)); if (!l) continue;
      for (const j of l) if ((xy[2 * j] - px) ** 2 + (xy[2 * j + 1] - py) ** 2 < 0.36) near.push(j);
    }
    if (near.length < 5) continue;
    let mx = 0, my = 0;
    for (const j of near) { mx += xy[2 * j]; my += xy[2 * j + 1]; }
    mx /= near.length; my /= near.length;
    let sxx = 0, sxy = 0, syy = 0;
    for (const j of near) { const dx = xy[2 * j] - mx, dy = xy[2 * j + 1] - my; sxx += dx * dx; sxy += dx * dy; syy += dy * dy; }
    sxx /= near.length; sxy /= near.length; syy /= near.length;
    const half = (sxx + syy) / 2, rad = Math.sqrt(((sxx - syy) / 2) ** 2 + sxy * sxy), l1 = half + rad, l2 = Math.max(half - rad, 0);
    if (Math.sqrt(l2) > 0.07 || l1 < 1e-4) continue; // not a clean line (corner / clutter)
    let vx = sxy, vy = l2 - sxx; // eigenvector of the smaller eigenvalue = line normal
    if (Math.abs(vx) + Math.abs(vy) < 1e-9) { vx = sxx < syy ? 1 : 0; vy = sxx < syy ? 0 : 1; }
    const len = Math.hypot(vx, vy);
    nx[i] = vx / len; ny[i] = vy / len; valid[i] = 1;
  }
  return { xy, nx, ny, valid, cells, n };
}

/**
 * Align `src` (interleaved sensor-frame x,y) to `ref`, starting from `init` = [x, y, theta].
 * opts: prior { pose:[x,y,th], wT, wR } (information weights), iters, gate (m), sigma (m, residual std).
 * Returns { pose, fitness (inlier fraction), rms, hmin (translation information), inliers }.
 */
export function icp(ref, src, init, { prior = null, iters = 25, gate = 0.5, sigma = 0.05 } = {}) {
  const m = src.length / 2, inv = 1 / (sigma * sigma);
  let x = init[0], y = init[1], th = init[2];
  let hmin = 0, inliers = 0, rssq = 0;
  for (let it = 0; it < iters; it++) {
    const c = Math.cos(th), s = Math.sin(th), g2 = (it < iters / 2 ? gate : gate * 0.5) ** 2;
    let h00 = 0, h01 = 0, h02 = 0, h11 = 0, h12 = 0, h22 = 0, b0 = 0, b1 = 0, b2 = 0;
    inliers = 0; rssq = 0;
    for (let i = 0; i < m; i++) {
      const px = src[2 * i], py = src[2 * i + 1];
      const wx = c * px - s * py + x, wy = s * px + c * py + y;
      const cx = Math.floor(wx / CELL), cy = Math.floor(wy / CELL);
      let best = -1, bd = g2;
      for (let a = -1; a <= 1; a++) for (let b = -1; b <= 1; b++) {
        const l = ref.cells.get(key(cx + a, cy + b)); if (!l) continue;
        for (const j of l) {
          if (!ref.valid[j]) continue;
          const d = (ref.xy[2 * j] - wx) ** 2 + (ref.xy[2 * j + 1] - wy) ** 2;
          if (d < bd) { bd = d; best = j; }
        }
      }
      if (best < 0) continue;
      const nx = ref.nx[best], ny = ref.ny[best];
      const r = nx * (wx - ref.xy[2 * best]) + ny * (wy - ref.xy[2 * best + 1]);
      const ar = Math.abs(r), w = (ar < 0.1 ? 1 : 0.1 / ar) * inv;
      const j2 = nx * (-s * px - c * py) + ny * (c * px - s * py);
      h00 += w * nx * nx; h01 += w * nx * ny; h02 += w * nx * j2; h11 += w * ny * ny; h12 += w * ny * j2; h22 += w * j2 * j2;
      b0 += w * nx * r; b1 += w * ny * r; b2 += w * j2 * r;
      if (ar < 0.12) inliers++;
      rssq += r * r;
    }
    hmin = (h00 + h11) / 2 - Math.sqrt(((h00 - h11) / 2) ** 2 + h01 * h01);
    if (prior) {
      h00 += prior.wT; h11 += prior.wT; h22 += prior.wR;
      b0 += prior.wT * (x - prior.pose[0]); b1 += prior.wT * (y - prior.pose[1]); b2 += prior.wR * (th - prior.pose[2]);
    }
    h00 += 1e-6; h11 += 1e-6; h22 += 1e-6;
    // solve 3x3 (symmetric) by Cramer's rule
    const det = h00 * (h11 * h22 - h12 * h12) - h01 * (h01 * h22 - h12 * h02) + h02 * (h01 * h12 - h11 * h02);
    if (Math.abs(det) < 1e-12) break;
    const d0 = -(b0 * (h11 * h22 - h12 * h12) - h01 * (b1 * h22 - h12 * b2) + h02 * (b1 * h12 - h11 * b2)) / det;
    const d1 = -(h00 * (b1 * h22 - h12 * b2) - b0 * (h01 * h22 - h12 * h02) + h02 * (h01 * b2 - b1 * h02)) / det;
    const d2 = -(h00 * (h11 * b2 - b1 * h12) - h01 * (h01 * b2 - b1 * h02) + b0 * (h01 * h12 - h11 * h02)) / det;
    x += d0; y += d1; th += d2;
    if (Math.abs(d0) + Math.abs(d1) + Math.abs(d2) < 1e-4) break;
  }
  while (th > Math.PI) th -= 2 * Math.PI; while (th < -Math.PI) th += 2 * Math.PI;
  return { pose: [x, y, th], fitness: m ? inliers / m : 0, rms: inliers ? Math.sqrt(rssq / Math.max(inliers, 1)) : 1, hmin, inliers };
}
