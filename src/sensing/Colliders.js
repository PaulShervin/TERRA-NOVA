// Analytic ray-casting world for the simulated LiDAR (pure JS, ROS axes, no Three.js).
// Every layout object is approximated by oriented boxes / vertical cylinders (visual-only decor is skipped).
// This is a *simplified* sensor world: composite machines use box approximations, not their exact meshes.
const CEILING_Z = 6.6;

function boxesFor(e) {
  const [sx, sy, sz] = e.size;
  const [px, py, pz] = e.position;
  const c = Math.cos(e.yaw), s = Math.sin(e.yaw);
  const out = [];
  // local offset (dx,dy) rotated into world; z range absolute (relative to base z)
  const lb = (dx, dy, w, d, z0, z1) => out.push({
    t: 'b', cx: px + c * dx - s * dy, cy: py + s * dx + c * dy, hx: w / 2, hy: d / 2, c, s, z0: pz + z0, z1: pz + z1,
  });
  switch (e.model) {
    case 'platform': {
      lb(0, 0, sx, sy, sz - 0.15, sz);
      for (const dx of [-sx / 2 + 0.15, sx / 2 - 0.15]) for (const dy of [-sy / 2 + 0.15, sy / 2 - 0.15]) lb(dx, dy, 0.25, 0.25, 0, sz - 0.15);
      break;
    }
    case 'conveyor': {
      if (e.params?.incline) {
        const rise = sz - 0.9, n = 8;
        for (let i = 0; i < n; i++) {
          const f = (i + 0.5) / n;
          lb(-sx / 2 + f * sx, 0, sx / n, sy, 0.3 + f * rise, 0.6 + f * rise);
        }
      } else {
        lb(0, 0, sx, sy, sz - 0.4, sz - 0.15);
        const legs = Math.round(sx / 3) + 1;
        for (let i = 0; i < legs; i++) {
          const x = -sx / 2 + 0.2 + (i * (sx - 0.4)) / (legs - 1);
          for (const y of [sy / 2 - 0.1, -sy / 2 + 0.1]) lb(x, y, 0.15, 0.15, 0, sz - 0.4);
        }
      }
      break;
    }
    case 'mill': {
      const r = Math.min(sy, sz - 0.5) / 2;
      lb(-sx * 0.38, 0, 1.2, sy * 0.8, 0, 0.5);
      lb(sx * 0.38, 0, 1.2, sy * 0.8, 0, 0.5);
      lb(0, 0, sx * 0.72, 2 * r, 0.5, 0.5 + 2 * r);
      break;
    }
    case 'crusher': {
      lb(0, 0, sx, sy, 0, 1.0);
      lb(0, 0, sx * 0.8, sy * 0.85, 1.0, 1.0 + sz * 0.55);
      break;
    }
    case 'workbench': {
      lb(0, 0, sx, sy, sz - 0.08, sz);
      for (const dx of [-sx / 2 + 0.06, sx / 2 - 0.06]) for (const dy of [-sy / 2 + 0.06, sy / 2 - 0.06]) lb(dx, dy, 0.07, 0.07, 0, sz - 0.08);
      break;
    }
    case 'rock': lb(0, 0, sx * 0.8, sy * 0.8, 0, sz * 0.9); break;
    default: lb(0, 0, sx, sy, 0, sz);
  }
  return out;
}

const CYL_MODELS = new Set(['tank', 'drum', 'bollard', 'pipe_v']);

export function buildColliders(layout) {
  const items = [];
  for (const e of layout.objects) {
    if (e.params?.decor || e.category === 'pathway' || e.model === 'floor') continue;
    if (CYL_MODELS.has(e.model)) {
      items.push({ t: 'c', cx: e.position[0], cy: e.position[1], r: e.size[0] / 2, z0: e.position[2], z1: e.position[2] + e.size[2], cat: e.category, id: e.id });
    } else {
      for (const b of boxesFor(e)) items.push({ ...b, cat: e.category, id: e.id });
    }
  }
  // world AABB for broad phase
  for (const it of items) {
    if (it.t === 'b') {
      const ex = Math.abs(it.c) * it.hx + Math.abs(it.s) * it.hy, ey = Math.abs(it.s) * it.hx + Math.abs(it.c) * it.hy;
      it.x0 = it.cx - ex; it.x1 = it.cx + ex; it.y0 = it.cy - ey; it.y1 = it.cy + ey;
    } else {
      it.x0 = it.cx - it.r; it.x1 = it.cx + it.r; it.y0 = it.cy - it.r; it.y1 = it.cy + it.r;
    }
  }
  const b = layout.meta.bounds;
  return { items, bounds: b, ceilingZ: CEILING_Z };
}

/** Closest hit along the ray; returns { t, cat, id } or null. dir must be unit length. */
export function castRay(world, ox, oy, oz, dx, dy, dz, maxT) {
  let bestT = maxT, cat = null, id = null;
  const { bounds: B } = world;
  // Floor / ceiling planes (only inside the hall footprint)
  if (dz < -1e-6) {
    const t = -oz / dz;
    if (t > 0 && t < bestT) { const x = ox + dx * t, y = oy + dy * t; if (x >= B.xMin && x <= B.xMax && y >= B.yMin && y <= B.yMax) { bestT = t; cat = 'floor'; id = 'floor'; } }
  } else if (dz > 1e-6) {
    const t = (world.ceilingZ - oz) / dz;
    if (t > 0 && t < bestT) { const x = ox + dx * t, y = oy + dy * t; if (x >= B.xMin && x <= B.xMax && y >= B.yMin && y <= B.yMax) { bestT = t; cat = 'wall'; id = 'ceiling'; } }
  }
  const invx = dx !== 0 ? 1 / dx : 1e30, invy = dy !== 0 ? 1 / dy : 1e30, invz = dz !== 0 ? 1 / dz : 1e30;
  for (const it of world.items) {
    // broad phase: ray vs world AABB
    let t0 = 0, t1 = bestT;
    let a = (it.x0 - ox) * invx, b = (it.x1 - ox) * invx;
    if (a > b) { const tmp = a; a = b; b = tmp; }
    t0 = Math.max(t0, a); t1 = Math.min(t1, b);
    if (t0 > t1) continue;
    a = (it.y0 - oy) * invy; b = (it.y1 - oy) * invy;
    if (a > b) { const tmp = a; a = b; b = tmp; }
    t0 = Math.max(t0, a); t1 = Math.min(t1, b);
    if (t0 > t1) continue;
    a = (it.z0 - oz) * invz; b = (it.z1 - oz) * invz;
    if (a > b) { const tmp = a; a = b; b = tmp; }
    t0 = Math.max(t0, a); t1 = Math.min(t1, b);
    if (t0 > t1) continue;
    if (it.t === 'b') {
      // exact: slab test in the box's local frame (rotate by -yaw)
      const lx = it.c * (ox - it.cx) + it.s * (oy - it.cy), ly = -it.s * (ox - it.cx) + it.c * (oy - it.cy);
      const ldx = it.c * dx + it.s * dy, ldy = -it.s * dx + it.c * dy;
      let n0 = 0, n1 = bestT;
      if (Math.abs(ldx) < 1e-9) { if (Math.abs(lx) > it.hx) continue; } else {
        let p = (-it.hx - lx) / ldx, q = (it.hx - lx) / ldx;
        if (p > q) { const tmp = p; p = q; q = tmp; }
        n0 = Math.max(n0, p); n1 = Math.min(n1, q);
      }
      if (Math.abs(ldy) < 1e-9) { if (Math.abs(ly) > it.hy) continue; } else {
        let p = (-it.hy - ly) / ldy, q = (it.hy - ly) / ldy;
        if (p > q) { const tmp = p; p = q; q = tmp; }
        n0 = Math.max(n0, p); n1 = Math.min(n1, q);
      }
      if (Math.abs(dz) < 1e-9) { if (oz < it.z0 || oz > it.z1) continue; } else {
        let p = (it.z0 - oz) * invz, q = (it.z1 - oz) * invz;
        if (p > q) { const tmp = p; p = q; q = tmp; }
        n0 = Math.max(n0, p); n1 = Math.min(n1, q);
      }
      if (n0 <= n1 && n0 > 1e-4 && n0 < bestT) { bestT = n0; cat = it.cat; id = it.id; }
    } else {
      // vertical cylinder: side wall + top cap
      const fx = ox - it.cx, fy = oy - it.cy;
      const A = dx * dx + dy * dy;
      let hit = Infinity;
      if (A > 1e-9) {
        const Bq = fx * dx + fy * dy, Cq = fx * fx + fy * fy - it.r * it.r;
        const disc = Bq * Bq - A * Cq;
        if (disc >= 0) {
          const sq = Math.sqrt(disc);
          for (const t of [(-Bq - sq) / A, (-Bq + sq) / A]) {
            if (t > 1e-4) { const z = oz + dz * t; if (z >= it.z0 && z <= it.z1) { hit = t; break; } }
          }
        }
      }
      if (dz < -1e-9) {
        const t = (it.z1 - oz) / dz;
        if (t > 1e-4 && t < hit) { const x = fx + dx * t, y = fy + dy * t; if (x * x + y * y <= it.r * it.r) hit = t; }
      }
      if (hit < bestT) { bestT = hit; cat = it.cat; id = it.id; }
    }
  }
  return cat ? { t: bestT, cat, id } : null;
}
