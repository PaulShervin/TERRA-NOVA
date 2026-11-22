// Records the cave map (voxel-deduplicated hits + simulator category + sensor positions) and the JS segmentation score,
// so the Python port of the segmenter can be checked on identical input.   Usage: node scripts/dump-map-log.mjs [simSeconds] [outFile]
import { writeFileSync } from 'node:fs';
import { buildCaveWorld } from '../src/cave/CaveGeometry.js';
import { createCaveMapper } from '../src/cave/CaveMapper.js';
import { createRobotController } from '../src/RobotController.js';
import { segmentPoints, evaluateSegmentation } from '../src/slam/Segmenter.js';
const maxT = Number(process.argv[2] ?? 200), out = process.argv[3] ?? 'map_log.json';
let s = 7; Math.random = () => ((s = (s * 16807) % 2147483647) / 2147483647);
const world = buildCaveWorld(); const sp = world.config.robot;
const ctl = createRobotController({ setPose() {}, wheels: [], lidarHead: { rotation: {} } }, [], { spawn: [sp.spawn[0], sp.spawn[1], 0], yaw: sp.yaw });
const mapper = createCaveMapper({ world, controller: ctl });
for (let t = 0; t < maxT; t += 0.05) mapper.update(0.05);
const h = mapper.hitStore, n = h.x.length, pts = new Float32Array(n * 3);
for (let i = 0; i < n; i++) { pts[3 * i] = h.x[i]; pts[3 * i + 1] = h.y[i]; pts[3 * i + 2] = h.z[i]; }
const vps = new Float32Array(mapper.trajectory.length * 3);
mapper.trajectory.forEach((q, i) => { vps[3 * i] = q.x; vps[3 * i + 1] = q.y; vps[3 * i + 2] = (world.floor.floorAt(q.x, q.y) ?? 0) + 1; });
const ev = evaluateSegmentation(segmentPoints(pts, { viewpoints: vps }).labels, Uint8Array.from(h.cat));
const r = (v) => Math.round(v * 1000) / 1000;
writeFileSync(out, JSON.stringify({ pts: Array.from(pts, r), cat: h.cat, viewpoints: Array.from(vps, r), js: { accuracy: ev.accuracy, meanIoU: ev.meanIoU, perClass: ev.perClass } }));
console.log(`wrote ${out}: ${n} points, js accuracy ${(ev.accuracy * 100).toFixed(1)}% mIoU ${(ev.meanIoU * 100).toFixed(1)}%`);
