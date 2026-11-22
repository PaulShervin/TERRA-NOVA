// SE(2) pose-graph optimisation (Gauss-Newton; each linear system solved matrix-free with Jacobi-preconditioned CG).
// Nodes are keyframe poses [x, y, th]; an edge {i, j, z:[x,y,th], info:[wx,wy,wth]} says "j is z away from i".
// Odometry edges form a chain; a loop-closure edge ties two far-apart (in time) nodes and pulls the whole chain straight.
import { wrap } from './se2.js';

export function optimizePoseGraph(poses, edges, { iterations = 8, anchorWeight = 1e6 } = {}) {
  const n = poses.length, dim = 3 * n, ne = edges.length;
  const x = new Float64Array(dim);
  poses.forEach((p, i) => { x[3 * i] = p[0]; x[3 * i + 1] = p[1]; x[3 * i + 2] = p[2]; });
  const x0 = [x[0], x[1], x[2]];
  const Ji = new Float64Array(9 * ne), Jj = new Float64Array(9 * ne);
  const b = new Float64Array(dim), diag = new Float64Array(dim), delta = new Float64Array(dim);

  const t3 = new Float64Array(3);
  const apply = (v, out) => {
    out.fill(0);
    for (let e = 0; e < ne; e++) {
      const ed = edges[e], i = 3 * ed.i, j = 3 * ed.j, a = 9 * e;
      for (let r = 0; r < 3; r++) {
        t3[r] = (Ji[a + 3 * r] * v[i] + Ji[a + 3 * r + 1] * v[i + 1] + Ji[a + 3 * r + 2] * v[i + 2]
          + Jj[a + 3 * r] * v[j] + Jj[a + 3 * r + 1] * v[j + 1] + Jj[a + 3 * r + 2] * v[j + 2]) * ed.info[r];
      }
      for (let c = 0; c < 3; c++) {
        out[i + c] += Ji[a + c] * t3[0] + Ji[a + 3 + c] * t3[1] + Ji[a + 6 + c] * t3[2];
        out[j + c] += Jj[a + c] * t3[0] + Jj[a + 3 + c] * t3[1] + Jj[a + 6 + c] * t3[2];
      }
    }
    for (let c = 0; c < 3; c++) out[c] += anchorWeight * v[c];
  };

  const Ap = new Float64Array(dim), r = new Float64Array(dim), z = new Float64Array(dim), p = new Float64Array(dim);
  for (let it = 0; it < iterations; it++) {
    b.fill(0); diag.fill(0);
    for (let e = 0; e < ne; e++) {
      const ed = edges[e], i = 3 * ed.i, j = 3 * ed.j, a = 9 * e;
      const c = Math.cos(x[i + 2]), s = Math.sin(x[i + 2]), dx = x[j] - x[i], dy = x[j + 1] - x[i + 1];
      const rx = c * dx + s * dy, ry = -s * dx + c * dy;
      const err0 = rx - ed.z[0], err1 = ry - ed.z[1], err2 = wrap(x[j + 2] - x[i + 2] - ed.z[2]);
      Ji.set([-c, -s, ry, s, -c, -rx, 0, 0, -1], a);
      Jj.set([c, s, 0, -s, c, 0, 0, 0, 1], a);
      const err = [err0, err1, err2];
      for (let col = 0; col < 3; col++) {
        let bi = 0, bj = 0, di = 0, dj = 0;
        for (let row = 0; row < 3; row++) {
          bi += Ji[a + 3 * row + col] * ed.info[row] * err[row];
          bj += Jj[a + 3 * row + col] * ed.info[row] * err[row];
          di += Ji[a + 3 * row + col] ** 2 * ed.info[row];
          dj += Jj[a + 3 * row + col] ** 2 * ed.info[row];
        }
        b[i + col] += bi; b[j + col] += bj; diag[i + col] += di; diag[j + col] += dj;
      }
    }
    for (let c = 0; c < 3; c++) { diag[c] += anchorWeight; b[c] += anchorWeight * (x[c] - x0[c]); }
    // PCG for A d = -b
    delta.fill(0);
    let rz = 0;
    for (let k = 0; k < dim; k++) { r[k] = -b[k]; z[k] = r[k] / (diag[k] + 1e-9); p[k] = z[k]; rz += r[k] * z[k]; }
    const r0 = Math.sqrt(r.reduce((s2, v) => s2 + v * v, 0)) + 1e-12;
    for (let k = 0; k < Math.min(3 * dim, 3000); k++) {
      apply(p, Ap);
      let pAp = 0; for (let q = 0; q < dim; q++) pAp += p[q] * Ap[q];
      if (pAp <= 0) break;
      const alpha = rz / pAp;
      let rn = 0;
      for (let q = 0; q < dim; q++) { delta[q] += alpha * p[q]; r[q] -= alpha * Ap[q]; rn += r[q] * r[q]; }
      if (Math.sqrt(rn) < 1e-7 * r0) break;
      let rzNew = 0;
      for (let q = 0; q < dim; q++) { z[q] = r[q] / (diag[q] + 1e-9); rzNew += r[q] * z[q]; }
      const beta = rzNew / rz; rz = rzNew;
      for (let q = 0; q < dim; q++) p[q] = z[q] + beta * p[q];
    }
    let big = 0;
    for (let k = 0; k < dim; k++) { x[k] += delta[k]; big = Math.max(big, Math.abs(delta[k])); }
    if (big < 1e-5) break;
  }
  return poses.map((_, i) => [x[3 * i], x[3 * i + 1], wrap(x[3 * i + 2])]);
}

/** Sum of squared (information-weighted) edge errors, for diagnostics. */
export function graphError(poses, edges) {
  let sum = 0;
  for (const ed of edges) {
    const a = poses[ed.i], b2 = poses[ed.j], c = Math.cos(a[2]), s = Math.sin(a[2]), dx = b2[0] - a[0], dy = b2[1] - a[1];
    const e0 = c * dx + s * dy - ed.z[0], e1 = -s * dx + c * dy - ed.z[1], e2 = wrap(b2[2] - a[2] - ed.z[2]);
    sum += e0 * e0 * ed.info[0] + e1 * e1 * ed.info[1] + e2 * e2 * ed.info[2];
  }
  return sum;
}
