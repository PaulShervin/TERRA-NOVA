// Headless sanity checks on the layout JSON: object overlaps + robot-inflated reachability (flood fill).
// Usage: node scripts/validate-layout.mjs
import { buildLayout } from '../src/layout/index.js';

const RES = 0.1;
const layout = buildLayout();
const { xMin, xMax, yMin, yMax } = layout.meta.bounds;
const W = Math.round((xMax - xMin) / RES);
const H = Math.round((yMax - yMin) / RES);
const idx = (i, j) => j * W + i;
const ROBOT_R = layout.robot.radius;
const ROBOT_H = 0.9;

const IGNORE = new Set(['floor', 'pathway']);
const zRange = (o) => [o.position[2], o.position[2] + o.size[2]];

/** Cells covered by an object's rotated footprint (cell centres inside the oriented rectangle). */
function footprint(o) {
  const [cx, cy] = o.position;
  const [sx, sy] = o.size;
  const c = Math.cos(o.yaw), s = Math.sin(o.yaw);
  const r = Math.hypot(sx, sy) / 2 + RES;
  const cells = [];
  const i0 = Math.max(0, Math.floor((cx - r - xMin) / RES));
  const i1 = Math.min(W - 1, Math.ceil((cx + r - xMin) / RES));
  const j0 = Math.max(0, Math.floor((cy - r - yMin) / RES));
  const j1 = Math.min(H - 1, Math.ceil((cy + r - yMin) / RES));
  for (let j = j0; j <= j1; j++) {
    for (let i = i0; i <= i1; i++) {
      const dx = xMin + (i + 0.5) * RES - cx;
      const dy = yMin + (j + 0.5) * RES - cy;
      const lx = c * dx + s * dy; // rotate world offset into object frame
      const ly = -s * dx + c * dy;
      if (Math.abs(lx) <= sx / 2 && Math.abs(ly) <= sy / 2) cells.push(idx(i, j));
    }
  }
  return cells;
}

// ---- 1. Overlap check (only objects whose vertical ranges intersect) ----
const solids = layout.objects.filter((o) => !IGNORE.has(o.category) && !o.params?.decor);
const fps = new Map(solids.map((o) => [o.id, footprint(o)]));
const overlaps = [];
for (let a = 0; a < solids.length; a++) {
  for (let b = a + 1; b < solids.length; b++) {
    const A = solids[a], B = solids[b];
    if (A.category === 'wall' && B.category === 'wall') continue;
    const [a0, a1] = zRange(A), [b0, b1] = zRange(B);
    if (a1 <= b0 + 1e-6 || b1 <= a0 + 1e-6) continue;
    const setB = new Set(fps.get(B.id));
    const n = fps.get(A.id).filter((k) => setB.has(k)).length;
    if (n > 6) overlaps.push(`${A.name} <-> ${B.name} (${(n * RES * RES).toFixed(2)} m2)`);
  }
}

// ---- 2. Reachability with robot-radius inflation ----
const blocked = new Uint8Array(W * H);
const nav = solids.filter((o) => zRange(o)[0] < ROBOT_H && o.category !== 'platform' && o.model !== 'roof');
nav.forEach((o) => fps.get(o.id).forEach((k) => { blocked[k] = 1; }));
// platform legs/stairs: stairs block, deck does not (robot passes beneath at 1.65 m clearance)
const inflated = new Uint8Array(W * H);
const rr = Math.ceil(ROBOT_R / RES);
for (let j = 0; j < H; j++) {
  for (let i = 0; i < W; i++) {
    if (!blocked[idx(i, j)]) continue;
    for (let dj = -rr; dj <= rr; dj++) {
      for (let di = -rr; di <= rr; di++) {
        if (di * di + dj * dj > rr * rr) continue;
        const ni = i + di, nj = j + dj;
        if (ni >= 0 && nj >= 0 && ni < W && nj < H) inflated[idx(ni, nj)] = 1;
      }
    }
  }
}
const toCell = ([x, y]) => [Math.floor((x - xMin) / RES), Math.floor((y - yMin) / RES)];
const [si, sj] = toCell(layout.robot.spawn);
const seen = new Uint8Array(W * H);
if (inflated[idx(si, sj)]) console.log('ERROR: robot spawn is in collision');
const queue = [[si, sj]];
seen[idx(si, sj)] = 1;
for (let q = 0; q < queue.length; q++) {
  const [i, j] = queue[q];
  for (const [di, dj] of [[1, 0], [-1, 0], [0, 1], [0, -1]]) {
    const ni = i + di, nj = j + dj;
    if (ni < 0 || nj < 0 || ni >= W || nj >= H) continue;
    const k = idx(ni, nj);
    if (seen[k] || inflated[k]) continue;
    seen[k] = 1;
    queue.push([ni, nj]);
  }
}
const freeCells = inflated.reduce((n, v) => n + (v ? 0 : 1), 0);

const PROBES = {
  'west hall': [-24, -6], 'north-west room': [-24, 12], 'south alcove': [-5, -15],
  'center junction': [4, 2], 'dead-end corridor tip': [0, 17], 'east hall': [14, 6],
  'SE bay A': [12, -11], 'SE bay B': [25, -10], 'north conveyor aisle': [14, 14],
};
console.log(`objects: ${layout.objects.length}`);
console.log(`overlaps (${overlaps.length}):`);
overlaps.forEach((o) => console.log('  ' + o));
console.log(`reachable free space: ${(100 * queue.length / freeCells).toFixed(1)}% of ${freeCells} free cells (robot r=${ROBOT_R} m)`);
let ok = true;
for (const [name, p] of Object.entries(PROBES)) {
  const [i, j] = toCell(p);
  const state = inflated[idx(i, j)] ? 'IN COLLISION' : seen[idx(i, j)] ? 'reachable' : 'UNREACHABLE';
  if (state !== 'reachable') ok = false;
  console.log(`  ${name.padEnd(24)} ${state}`);
}
// ---- 3. Patrol loop must be collision-free for the inflated robot ----
const loop = [...layout.navPath, layout.navPath[0]];
let pathHits = 0, samples = 0;
for (let i = 1; i < loop.length; i++) {
  const [x0, y0] = loop[i - 1], [x1, y1] = loop[i];
  const n = Math.ceil(Math.hypot(x1 - x0, y1 - y0) / 0.05);
  for (let k = 0; k <= n; k++) {
    const [ci, cj] = toCell([x0 + ((x1 - x0) * k) / n, y0 + ((y1 - y0) * k) / n]);
    samples++;
    if (inflated[idx(ci, cj)]) pathHits++;
  }
}
console.log(`  patrol loop: ${samples} samples, ${pathHits} in collision`);
process.exit(overlaps.length === 0 && ok && pathHits === 0 ? 0 : 1);
