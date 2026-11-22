// Signed-distance field of the cave (air < 0, rock > 0) built by splatting tunnel / chamber cross-sections,
// carving solids, then adding height-aware fractal noise. Everything downstream (mesh, LiDAR, floor, truth)
// reads this ONE field, so what you see is exactly what the sensor measures. Pure JS, ROS axes.

// ---- 3D value noise (seeded, tileless) ----
function makeNoise3(seed) {
  const N = 256, perm = new Uint8Array(N * 2), vals = new Float32Array(N);
  let s = seed >>> 0 || 1;
  const rnd = () => ((s = (s * 1664525 + 1013904223) >>> 0) / 4294967296);
  for (let i = 0; i < N; i++) { perm[i] = i; vals[i] = rnd() * 2 - 1; }
  for (let i = N - 1; i > 0; i--) { const j = Math.floor(rnd() * (i + 1)); [perm[i], perm[j]] = [perm[j], perm[i]]; }
  for (let i = 0; i < N; i++) perm[i + N] = perm[i];
  const h = (x, y, z) => vals[perm[perm[perm[x & 255] + (y & 255)] + (z & 255)]];
  const fade = (t) => t * t * t * (t * (t * 6 - 15) + 10);
  return (x, y, z) => {
    const xi = Math.floor(x), yi = Math.floor(y), zi = Math.floor(z);
    const u = fade(x - xi), v = fade(y - yi), w = fade(z - zi);
    const l = (a, b, t) => a + (b - a) * t;
    return l(
      l(l(h(xi, yi, zi), h(xi + 1, yi, zi), u), l(h(xi, yi + 1, zi), h(xi + 1, yi + 1, zi), u), v),
      l(l(h(xi, yi, zi + 1), h(xi + 1, yi, zi + 1), u), l(h(xi, yi + 1, zi + 1), h(xi + 1, yi + 1, zi + 1), u), v), w);
  };
}

const smoothstep = (a, b, x) => { const t = Math.min(1, Math.max(0, (x - a) / (b - a))); return t * t * (3 - 2 * t); };

export const FIELD_VOXEL = 0.35;
const ROCK = 8;

export function buildField(network, { voxel = FIELD_VOXEL } = {}) {
  const B = network.cfg.meta.bounds;
  const nx = Math.ceil((B.xMax - B.xMin) / voxel) + 1, ny = Math.ceil((B.yMax - B.yMin) / voxel) + 1, nz = Math.ceil((B.zMax - B.zMin) / voxel) + 1;
  const data = new Float32Array(nx * ny * nz).fill(ROCK);
  const rel = new Float32Array(nx * ny * nz); // height above the local floor of the blob that defined the voxel
  const idx = (i, j, k) => (k * ny + j) * nx + i;
  const field = { nx, ny, nz, voxel, ox: B.xMin, oy: B.yMin, oz: B.zMin, data, bounds: B };

  /** Super-ellipse (p=4) cross-section blob: flat floor, steep-ish walls, arched ceiling. */
  function splat(cx, cy, floor, a, b, h, rotYaw = null) {
    const R = Math.max(a, b) + 2.2;
    const i0 = Math.max(1, Math.floor((cx - R - field.ox) / voxel)), i1 = Math.min(nx - 2, Math.ceil((cx + R - field.ox) / voxel));
    const j0 = Math.max(1, Math.floor((cy - R - field.oy) / voxel)), j1 = Math.min(ny - 2, Math.ceil((cy + R - field.oy) / voxel));
    const k0 = Math.max(1, Math.floor((floor - 1.5 - field.oz) / voxel)), k1 = Math.min(nz - 2, Math.ceil((floor + h + 1.5 - field.oz) / voxel));
    const vb = h / 2, zc = floor + vb;
    const scale = Math.min(Math.min(a, b === 0 ? a : a), vb) * 0.5;
    const co = rotYaw === null ? 1 : Math.cos(rotYaw), si = rotYaw === null ? 0 : Math.sin(rotYaw);
    for (let k = k0; k <= k1; k++) {
      const z = field.oz + k * voxel, v = (z - zc) / vb, v2 = v * v, v4 = v2 * v2;
      for (let j = j0; j <= j1; j++) {
        const y = field.oy + j * voxel;
        for (let i = i0; i <= i1; i++) {
          const x = field.ox + i * voxel;
          let u2;
          if (rotYaw === null) { const d2 = (x - cx) * (x - cx) + (y - cy) * (y - cy); u2 = d2 / (a * a); } else {
            const dx = x - cx, dy = y - cy, lx = (co * dx + si * dy) / a, ly = (-si * dx + co * dy) / b;
            u2 = lx * lx + ly * ly;
          }
          const q = Math.sqrt(Math.sqrt(u2 * u2 + v4));
          const val = (q - 1) * scale;
          const id = idx(i, j, k);
          if (val < data[id]) { data[id] = val; rel[id] = z - floor; }
        }
      }
    }
  }

  for (const t of network.tunnels) for (const s of t.samples) splat(s.x, s.y, s.z, s.w / 2, s.w / 2, s.h);
  for (const c of network.chambers) splat(c.x, c.y, c.z, c.rx, c.ry, c.h, c.yaw);

  // ---- carve solids: data = max(data, -solidSdf) ----
  function carve(sdfFn, x0, y0, z0, x1, y1, z1) {
    const i0 = Math.max(1, Math.floor((x0 - field.ox) / voxel)), i1 = Math.min(nx - 2, Math.ceil((x1 - field.ox) / voxel));
    const j0 = Math.max(1, Math.floor((y0 - field.oy) / voxel)), j1 = Math.min(ny - 2, Math.ceil((y1 - field.oy) / voxel));
    const k0 = Math.max(1, Math.floor((z0 - field.oz) / voxel)), k1 = Math.min(nz - 2, Math.ceil((z1 - field.oz) / voxel));
    for (let k = k0; k <= k1; k++) for (let j = j0; j <= j1; j++) for (let i = i0; i <= i1; i++) {
      const sd = sdfFn(field.ox + i * voxel, field.oy + j * voxel, field.oz + k * voxel);
      const id = idx(i, j, k);
      if (-sd > data[id]) data[id] = -sd;
    }
  }
  for (const s of network.solids) {
    if (s.kind === 'pillar') {
      const r = s.radius, ph = s.id.length * 1.7;
      carve((x, y, z) => Math.hypot(x - s.x, y - s.y) - r * (1 + 0.16 * Math.sin(z * 0.8 + ph) + (z - s.zFloor < 0.8 ? 0.25 * (1 - (z - s.zFloor) / 0.8) : 0)),
        s.x - r * 1.8, s.y - r * 1.8, s.zFloor - 1.5, s.x + r * 1.8, s.y + r * 1.8, s.zFloor + s.height + 1.5);
    } else if (s.kind === 'boulder') {
      const [rx, ry, rz] = s.radii, co = Math.cos(s.yaw), si = Math.sin(s.yaw), m = Math.min(rx, ry, rz);
      const R = Math.max(rx, ry, rz) * 1.4;
      carve((x, y, z) => {
        const dx = x - s.x, dy = y - s.y, lx = (co * dx + si * dy) / rx, ly = (-si * dx + co * dy) / ry, lz = (z - s.z) / rz;
        return (Math.sqrt(lx * lx + ly * ly + lz * lz) - 1) * m;
      }, s.x - R, s.y - R, s.z - R, s.x + R, s.y + R, s.z + R);
    } else {
      carve((x, y, z) => {
        const t = (s.zCeil - z) / s.length; // 0 at ceiling, 1 at tip
        if (t < -0.3 || t > 1) return ROCK;
        return Math.hypot(x - s.x, y - s.y) - s.radius * (1 - Math.max(t, 0) * 0.95);
      }, s.x - s.radius * 2, s.y - s.radius * 2, s.zCeil - s.length - 0.5, s.x + s.radius * 2, s.y + s.radius * 2, s.zCeil + 1.5);
    }
  }

  // ---- fractal noise, stronger away from the floor so floors stay drivable ----
  const n1 = makeNoise3(network.cfg.meta.seed), n2 = makeNoise3(network.cfg.meta.seed + 7), n3 = makeNoise3(network.cfg.meta.seed + 13);
  for (let k = 1; k < nz - 1; k++) {
    const z = field.oz + k * voxel;
    for (let j = 1; j < ny - 1; j++) {
      const y = field.oy + j * voxel;
      for (let i = 1; i < nx - 1; i++) {
        const id = idx(i, j, k);
        if (data[id] > 3.2) continue;
        const x = field.ox + i * voxel;
        const wall = 0.1 + 0.9 * smoothstep(0.6, 2.2, rel[id]);
        data[id] += wall * (0.62 * n1(x * 0.11, y * 0.11, z * 0.2) + 0.2 * n2(x * 0.33, y * 0.33, z * 0.45))
          + (0.05 + 0.1 * wall) * n3(x * 1.1, y * 1.1, z * 1.1);
      }
    }
  }
  // seal the grid border with rock
  for (let k = 0; k < nz; k++) for (let j = 0; j < ny; j++) for (let i = 0; i < nx; i++) {
    if (i === 0 || j === 0 || k === 0 || i === nx - 1 || j === ny - 1 || k === nz - 1) data[idx(i, j, k)] = ROCK;
  }

  // ---- trilinear sampling + gradient ----
  field.sample = (x, y, z) => {
    const fx = (x - field.ox) / voxel, fy = (y - field.oy) / voxel, fz = (z - field.oz) / voxel;
    if (fx < 0 || fy < 0 || fz < 0 || fx >= nx - 1 || fy >= ny - 1 || fz >= nz - 1) return ROCK;
    const i = fx | 0, j = fy | 0, k = fz | 0, u = fx - i, v = fy - j, w = fz - k;
    const a = (k * ny + j) * nx + i, sy = nx, sz = nx * ny;
    const c00 = data[a] * (1 - u) + data[a + 1] * u, c10 = data[a + sy] * (1 - u) + data[a + sy + 1] * u;
    const c01 = data[a + sz] * (1 - u) + data[a + sz + 1] * u, c11 = data[a + sz + sy] * (1 - u) + data[a + sz + sy + 1] * u;
    return (c00 * (1 - v) + c10 * v) * (1 - w) + (c01 * (1 - v) + c11 * v) * w;
  };
  field.gradient = (x, y, z, e = 0.2) => {
    const gx = field.sample(x + e, y, z) - field.sample(x - e, y, z), gy = field.sample(x, y + e, z) - field.sample(x, y - e, z), gz = field.sample(x, y, z + e) - field.sample(x, y, z - e);
    const l = Math.hypot(gx, gy, gz) || 1;
    return [gx / l, gy / l, gz / l]; // points INTO rock; the surface normal facing the air is the negative
  };

  /** Sphere-trace a ray through the air. Returns { t, point, normal } or null (no hit within maxT / started in rock). */
  field.cast = (ox_, oy_, oz_, dx, dy, dz, maxT) => {
    if (field.sample(ox_, oy_, oz_) > -0.02) return null;
    let t = 0.04, prevT = 0;
    for (let it = 0; it < 160 && t < maxT; it++) {
      const v = field.sample(ox_ + dx * t, oy_ + dy * t, oz_ + dz * t);
      if (v >= -0.01) {
        let lo = prevT, hi = t; // bisection to the zero crossing
        for (let b = 0; b < 7; b++) { const m = (lo + hi) / 2; if (field.sample(ox_ + dx * m, oy_ + dy * m, oz_ + dz * m) < 0) lo = m; else hi = m; }
        const tt = (lo + hi) / 2, px = ox_ + dx * tt, py = oy_ + dy * tt, pz = oz_ + dz * tt;
        const g = field.gradient(px, py, pz);
        return { t: tt, point: [px, py, pz], normal: [-g[0], -g[1], -g[2]] };
      }
      prevT = t;
      t += Math.max(-v * 0.62, 0.035);
    }
    return null;
  };

  return field;
}

/** Floor heightfield per field column + drivable mask (clearance >= 1.6 m, local slope limit). */
export function buildFloorMap(field, network, { minClear = 1.6, maxStep = 0.5 } = {}) {
  const { nx, ny, nz, voxel, data } = field;
  const hf = new Float32Array(nx * ny).fill(NaN), clear = new Float32Array(nx * ny);
  for (let j = 1; j < ny - 1; j++) {
    for (let i = 1; i < nx - 1; i++) {
      const x = field.ox + i * voxel, y = field.oy + j * voxel;
      const loc = network.locate(x, y);
      if (loc.q > 1.4) continue;
      let best = null;
      let k = 1;
      while (k < nz - 1) { // walk air intervals along this column
        while (k < nz - 1 && data[(k * ny + j) * nx + i] >= 0) k++;
        if (k >= nz - 1) break;
        const k0 = k;
        while (k < nz - 1 && data[(k * ny + j) * nx + i] < 0) k++;
        // exact (interpolated) floor / ceiling crossings, not voxel-quantised heights
        const colId = (kk) => (kk * ny + j) * nx + i;
        const dLo0 = data[colId(k0 - 1)], dLo1 = data[colId(k0)], dHi0 = data[colId(k - 1)], dHi1 = data[colId(k)];
        const z0 = field.oz + (k0 - 1 + dLo0 / (dLo0 - dLo1)) * voxel;
        const z1 = field.oz + (k - 1 + dHi0 / (dHi0 - dHi1)) * voxel;
        if (z1 - z0 >= minClear && (!best || Math.abs(z0 - loc.floor) < Math.abs(best.z0 - loc.floor))) best = { z0, z1 };
      }
      if (best && Math.abs(best.z0 - loc.floor) < 2.5) { hf[j * nx + i] = best.z0; clear[j * nx + i] = best.z1 - best.z0; }
    }
  }
  const drivable = new Uint8Array(nx * ny);
  for (let j = 1; j < ny - 1; j++) {
    for (let i = 1; i < nx - 1; i++) {
      const z = hf[j * nx + i];
      if (Number.isNaN(z)) continue;
      let ok = true;
      for (const [di, dj] of [[1, 0], [-1, 0], [0, 1], [0, -1]]) {
        const nzv = hf[(j + dj) * nx + i + di];
        if (!Number.isNaN(nzv) && Math.abs(nzv - z) > maxStepFor(maxStep, voxel)) ok = false;
      }
      if (ok) drivable[j * nx + i] = 1;
    }
  }
  const floorAt = (x, y) => {
    const fx = (x - field.ox) / voxel, fy = (y - field.oy) / voxel;
    const i = Math.round(fx), j = Math.round(fy);
    if (i < 1 || j < 1 || i >= nx - 1 || j >= ny - 1) return null;
    let z = hf[j * nx + i];
    if (Number.isNaN(z)) { // fall back to the nearest valid neighbour
      for (const [di, dj] of [[1, 0], [-1, 0], [0, 1], [0, -1], [1, 1], [-1, -1], [1, -1], [-1, 1]]) { const v = hf[(j + dj) * nx + i + di]; if (!Number.isNaN(v)) { z = v; break; } }
    }
    return Number.isNaN(z) ? null : z;
  };
  return { hf, clear, drivable, floorAt, nx, ny, voxel };
}
const maxStepFor = (slopeTan, voxel) => slopeTan * voxel * 1.1; // max height difference allowed between neighbouring columns
