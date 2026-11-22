// Headless check of the procedural cave: generation time, mesh size, drivable connectivity between all
// landmarks / chambers / dead ends from the entrance, and narrowest drivable passage.
import { buildNetwork } from '../src/cave/CaveTunnelNetwork.js';
import { buildField, buildFloorMap } from '../src/cave/CaveField.js';
import { extractSurface } from '../src/cave/CaveMesh.js';

let t0 = performance.now();
const lap = (label) => { const t = performance.now(); console.log(`${label.padEnd(26)} ${(t - t0).toFixed(0)} ms`); t0 = t; };
const net = buildNetwork(); lap('network');
const field = buildField(net); lap(`field ${field.nx}x${field.ny}x${field.nz}`);
const floor = buildFloorMap(field, net); lap('floor map');
const mesh = extractSurface(field); lap('surface nets');
console.log(`vertices ${mesh.positions.length / 3}, triangles ${mesh.indices.length / 3}`);

// flood fill over drivable columns (8-neighbour, no big height jumps) from the robot spawn
const { nx, ny, hf, drivable, voxel } = floor;
const cell = (x, y) => [Math.round((x - field.ox) / voxel), Math.round((y - field.oy) / voxel)];
const spawn = net.cfg.robot.spawn;
const [si, sj] = cell(spawn[0], spawn[1]);
const seen = new Uint8Array(nx * ny);
const q = [[si, sj]]; seen[sj * nx + si] = 1;
if (!drivable[sj * nx + si]) console.log('WARNING: spawn column is not drivable');
for (let h = 0; h < q.length; h++) {
  const [i, j] = q[h];
  for (let dj = -1; dj <= 1; dj++) for (let di = -1; di <= 1; di++) {
    const ni = i + di, nj = j + dj, k = nj * nx + ni;
    if (!di && !dj) continue;
    if (ni < 1 || nj < 1 || ni >= nx - 1 || nj >= ny - 1 || seen[k] || !drivable[k]) continue;
    if (Math.abs(hf[k] - hf[j * nx + i]) > 0.2) continue;
    seen[k] = 1; q.push([ni, nj]);
  }
}
let total = 0; for (let k = 0; k < drivable.length; k++) total += drivable[k];
console.log(`drivable columns ${total}, reachable from spawn ${q.length} (${((100 * q.length) / total).toFixed(1)}%)`);

// probes: every chamber centre-ish, junction, dead end and landmark must be reachable
const probes = [
  ...net.chambers.map((c) => [`chamber ${c.name}`, c.x + c.rx * 0.5, c.y]),
  ...net.cfg.nodes.filter((n) => n.type !== 'chamber').map((n) => [n.id, n.position[0], n.position[1]]),
];
let bad = 0;
for (const [name, x, y] of probes) {
  let ok = false;
  const [ci, cj] = cell(x, y);
  for (let dj = -4; dj <= 4 && !ok; dj++) for (let di = -4; di <= 4; di++) if (seen[(cj + dj) * nx + ci + di]) { ok = true; break; }
  if (!ok) bad++;
  console.log(`  ${name.padEnd(28)} ${ok ? 'reachable' : 'UNREACHABLE'}`);
}
// floor slope along reachable region
let maxSlope = 0;
for (const [i, j] of q) for (const [di, dj] of [[1, 0], [0, 1]]) { const k = (j + dj) * nx + i + di; if (seen[k]) maxSlope = Math.max(maxSlope, Math.abs(hf[k] - hf[j * nx + i]) / voxel); }
console.log(`max floor slope in reachable area: ${(Math.atan(maxSlope) * 57.2958).toFixed(1)} deg`);
process.exit(bad === 0 && q.length / total > 0.9 ? 0 : 1);
