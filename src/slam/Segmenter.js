// Label-free semantic segmentation of a LiDAR map: classes come purely from local GEOMETRY (no training data, no labels):
//   per voxel -> PCA of its neighbourhood -> surface normal, planarity -> floor / ceiling / wall / object
//   (floor/ceiling = surface normal pointing up/down toward the sensor; object = convex blob bulging toward the sensor, wall = the rest)
// The simulator's category labels are used ONLY to score the result (see evaluateSegmentation).
export const CLASSES = ['floor', 'ceiling', 'wall', 'object'];

const VOXEL = 0.3;
const keyOf = (i, j, k) => ((i + 512) * 1024 + (j + 512)) * 1024 + (k + 512);

/** Eigen-decomposition of a symmetric 3x3 (Jacobi). Returns eigenvalues (descending) and the eigenvector of the smallest one. */
function eig3(a) {
  const A = [a[0], a[1], a[2], a[1], a[3], a[4], a[2], a[4], a[5]], V = [1, 0, 0, 0, 1, 0, 0, 0, 1];
  for (let sweep = 0; sweep < 12; sweep++) {
    let off = Math.abs(A[1]) + Math.abs(A[2]) + Math.abs(A[5]);
    if (off < 1e-12) break;
    for (const [p, q] of [[0, 1], [0, 2], [1, 2]]) {
      const apq = A[3 * p + q];
      if (Math.abs(apq) < 1e-14) continue;
      const th = (A[3 * q + q] - A[3 * p + p]) / (2 * apq), t = Math.sign(th || 1) / (Math.abs(th) + Math.sqrt(th * th + 1)), c = 1 / Math.sqrt(t * t + 1), s = t * c;
      for (let k = 0; k < 3; k++) { const akp = A[3 * k + p], akq = A[3 * k + q]; A[3 * k + p] = c * akp - s * akq; A[3 * k + q] = s * akp + c * akq; }
      for (let k = 0; k < 3; k++) { const apk = A[3 * p + k], aqk = A[3 * q + k]; A[3 * p + k] = c * apk - s * aqk; A[3 * q + k] = s * apk + c * aqk; }
      for (let k = 0; k < 3; k++) { const vkp = V[3 * k + p], vkq = V[3 * k + q]; V[3 * k + p] = c * vkp - s * vkq; V[3 * k + q] = s * vkp + c * vkq; }
    }
  }
  const ev = [A[0], A[4], A[8]], order = [0, 1, 2].sort((x, y) => ev[y] - ev[x]);
  const m = order[2];
  return { l: order.map((i) => Math.max(ev[i], 0)), n: [V[m], V[3 + m], V[6 + m]] };
}

const VP_CELL = 4;
function buildViewpointHash(vp) {
  const cells = new Map();
  for (let i = 0; i < vp.length; i += 3) { const k = Math.floor(vp[i] / VP_CELL) * 65536 + Math.floor(vp[i + 1] / VP_CELL); const l = cells.get(k); if (l) l.push(i); else cells.set(k, [i]); }
  return { vp, cells };
}
function nearestViewpoint(h, x, y, z) {
  const cx = Math.floor(x / VP_CELL), cy = Math.floor(y / VP_CELL); let best = null, bd = Infinity;
  for (let a = -1; a <= 1; a++) for (let b = -1; b <= 1; b++) {
    const l = h.cells.get((cx + a) * 65536 + (cy + b)); if (!l) continue;
    for (const i of l) { const d = (h.vp[i] - x) ** 2 + (h.vp[i + 1] - y) ** 2 + (h.vp[i + 2] - z) ** 2; if (d < bd) { bd = d; best = i; } }
  }
  return best === null ? [x, y, z + 1] : [h.vp[best], h.vp[best + 1], h.vp[best + 2]];
}

/**
 * pts: Float32Array/array interleaved [x,y,z,...]. Returns { labels: Uint8Array (class index per point), voxels }.
 * options: viewpoints (interleaved sensor positions: orient normals + convexity test), floorNz / ceilingNz (oriented normal thresholds),
 * convexity (mean offset of neighbours behind the tangent plane; below this = bulges toward the sensor = object), minObjectVoxels.
 */
export function segmentPoints(pts, { viewpoints = null, minNeighbours = 6, floorNz = 0.7, ceilingNz = -0.5, convexity = -0.02, minObjectVoxels = 6, horizontalNz = 0.72, floorBand = 1.4 } = {}) {
  const n = pts.length / 3, vox = new Map(), vp = viewpoints ? buildViewpointHash(viewpoints) : null;
  const vidx = new Int32Array(n);
  for (let p = 0; p < n; p++) {
    const i = Math.floor(pts[3 * p] / VOXEL), j = Math.floor(pts[3 * p + 1] / VOXEL), k = Math.floor(pts[3 * p + 2] / VOXEL), key = keyOf(i, j, k);
    let v = vox.get(key);
    if (!v) { v = { i, j, k, n: 0, sx: 0, sy: 0, sz: 0, cls: -1, id: vox.size }; vox.set(key, v); }
    v.n++; v.sx += pts[3 * p]; v.sy += pts[3 * p + 1]; v.sz += pts[3 * p + 2]; vidx[p] = v.id;
  }
  const list = [...vox.values()];
  for (const v of list) { v.cx = v.sx / v.n; v.cy = v.sy / v.n; v.cz = v.sz / v.n; }

  // 1) local geometry per voxel
  for (const v of list) {
    let w = 0, mx = 0, my = 0, mz = 0; const nb = [];
    for (let a = -1; a <= 1; a++) for (let b = -1; b <= 1; b++) for (let c = -1; c <= 1; c++) {
      const u = vox.get(keyOf(v.i + a, v.j + b, v.k + c)); if (!u) continue;
      nb.push(u); w += u.n; mx += u.cx * u.n; my += u.cy * u.n; mz += u.cz * u.n;
    }
    if (w < minNeighbours || nb.length < 4) { v.planar = 0; v.nz = 0; v.valid = false; continue; }
    mx /= w; my /= w; mz /= w;
    let xx = 0, xy = 0, xz = 0, yy = 0, yz = 0, zz = 0;
    for (const u of nb) { const dx = u.cx - mx, dy = u.cy - my, dz = u.cz - mz; xx += u.n * dx * dx; xy += u.n * dx * dy; xz += u.n * dx * dz; yy += u.n * dy * dy; yz += u.n * dy * dz; zz += u.n * dz * dz; }
    const { l, n: nrm } = eig3([xx / w, xy / w, xz / w, yy / w, yz / w, zz / w]);
    const s1 = Math.sqrt(l[0]), s2 = Math.sqrt(l[1]), s3 = Math.sqrt(l[2]);
    v.planar = s1 > 1e-6 ? (s2 - s3) / s1 : 0; v.thin = s3; v.valid = true;
    v.nz = Math.abs(nrm[2]);
    if (vp) { // orient the normal toward the nearest sensor position, then measure convexity (objects bulge toward the sensor, tunnel walls curve away)
      const o = nearestViewpoint(vp, v.cx, v.cy, v.cz);
      let dot = nrm[0] * (o[0] - v.cx) + nrm[1] * (o[1] - v.cy) + nrm[2] * (o[2] - v.cz);
      const sgn = dot >= 0 ? 1 : -1;
      v.oz = sgn * nrm[2];
      let cv = 0, cn = 0;
      for (const u of nb) { if (u === v) continue; cv += ((u.cx - v.cx) * nrm[0] + (u.cy - v.cy) * nrm[1] + (u.cz - v.cz) * nrm[2]) * sgn; cn++; }
      v.cvx = cn ? cv / cn : 0;
    }
  }

  // 2) classes from oriented normals (when sensor positions are known) or from the column heuristic
  const colMin = new Map(), colKey = (v) => Math.floor(v.cx / 1.5) * 65536 + Math.floor(v.cy / 1.5);
  if (!vp) for (const v of list) if (v.valid && v.nz >= horizontalNz) { const c = colKey(v), m = colMin.get(c); if (m === undefined || v.cz < m) colMin.set(c, v.cz); }
  for (const v of list) {
    if (!v.valid) { v.cls = 2; continue; }
    if (vp) v.cls = v.oz > floorNz ? 0 : v.oz < ceilingNz ? 1 : (v.cvx < convexity ? 3 : 2);
    else if (v.nz >= horizontalNz) v.cls = v.cz - colMin.get(colKey(v)) > floorBand ? 1 : 0;
    else v.cls = 2;
  }

  // 3) object voxels must form a blob of at least minObjectVoxels connected voxels (removes speckle); leftovers revert to wall
  const seen = new Uint8Array(list.length);
  for (const start of list) {
    if (start.cls !== 3 || seen[start.id]) continue;
    const comp = [start]; seen[start.id] = 1;
    for (let q = 0; q < comp.length; q++) {
      const u = comp[q];
      for (let a = -1; a <= 1; a++) for (let b = -1; b <= 1; b++) for (let c = -1; c <= 1; c++) {
        const w = vox.get(keyOf(u.i + a, u.j + b, u.k + c));
        if (w && w.cls === 3 && !seen[w.id]) { seen[w.id] = 1; comp.push(w); }
      }
    }
    if (comp.length < minObjectVoxels) for (const u of comp) u.cls = 2;
  }

  const labels = new Uint8Array(n);
  for (let p = 0; p < n; p++) labels[p] = list[vidx[p]].cls;
  return { labels, voxels: list.length };
}

/** Score predicted labels against simulator categories (0 floor, 1 ceiling, 2 wall, 3 object). */
export function evaluateSegmentation(pred, truth) {
  const conf = Array.from({ length: 4 }, () => new Array(4).fill(0));
  for (let i = 0; i < pred.length; i++) conf[truth[i]][pred[i]]++;
  let ok = 0, total = 0;
  const perClass = CLASSES.map((name, c) => {
    const tp = conf[c][c], fn = conf[c].reduce((a, b) => a + b, 0) - tp, fp = conf.reduce((a, row) => a + row[c], 0) - tp;
    ok += tp; total += tp + fn;
    return { name, support: tp + fn, recall: tp + fn ? tp / (tp + fn) : null, precision: tp + fp ? tp / (tp + fp) : null, iou: tp + fn + fp ? tp / (tp + fn + fp) : null };
  });
  return { accuracy: total ? ok / total : 0, meanIoU: perClass.filter((c) => c.iou !== null && c.support > 0).reduce((a, c, _, arr) => a + c.iou / arr.length, 0), perClass, confusion: conf };
}
