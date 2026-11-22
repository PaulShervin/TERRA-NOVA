// Evaluation harness for the cave: runs autonomous exploration for several seeds / frontier strategies and reports
//   exploration  : coverage, time to finish, distance driven
//   localisation : ATE / RPE of odometry alone vs LiDAR odometry + pose graph (shadow SLAM), loop closures accepted / rejected
//   semantics    : accuracy + IoU of the LABEL-FREE geometric segmentation vs the simulator categories
// Usage: node scripts/evaluate.mjs [--seeds 1,2,3] [--strategies nearest,balanced,large] [--time 1200] [--out data/eval/cave_eval]
import fs from 'node:fs';
import path from 'node:path';
import { buildCaveWorld } from '../src/cave/CaveGeometry.js';
import { createCaveMapper } from '../src/cave/CaveMapper.js';
import { createRobotController } from '../src/RobotController.js';
import { segmentPoints, evaluateSegmentation } from '../src/slam/Segmenter.js';

const arg = (name, def) => { const i = process.argv.indexOf('--' + name); return i > 0 ? process.argv[i + 1] : def; };
const seeds = arg('seeds', '1,2,3').split(',').map(Number), maxT = Number(arg('time', 1200)), out = arg('out', 'data/eval/cave_eval');
// frontier scoring: score = pathCost - sizeWeight * min(frontierSize, 40)  (lower is better)
const STRATEGIES = { nearest: 0, balanced: 0.12, large: 0.5 };
const names = arg('strategies', 'nearest,balanced,large').split(',');

const world = buildCaveWorld();
const spawn = world.config.robot;
const rows = [];
for (const strategy of names) for (const seed of seeds) {
  let s = seed * 7919;
  Math.random = () => ((s = (s * 16807) % 2147483647) / 2147483647);
  const ctl = createRobotController({ setPose() {}, wheels: [], lidarHead: { rotation: {} } }, [], { spawn: [spawn.spawn[0], spawn.spawn[1], 0], yaw: spawn.yaw });
  const mapper = createCaveMapper({ world, controller: ctl, explorerOptions: { sizeWeight: STRATEGIES[strategy] }, slamOptions: { seed } });
  const t0 = Date.now(); let t = 0;
  for (; t < maxT; t += 0.05) { mapper.update(0.05); if (mapper.stats().state === 'complete') break; }
  const st = mapper.stats(), m = mapper.slam.metrics();
  const h = mapper.hitStore, n = h.x.length, pts = new Float32Array(n * 3);
  for (let i = 0; i < n; i++) { pts[3 * i] = h.x[i]; pts[3 * i + 1] = h.y[i]; pts[3 * i + 2] = h.z[i]; }
  const vps = new Float32Array(mapper.trajectory.length * 3);
  mapper.trajectory.forEach((q, i) => { vps[3 * i] = q.x; vps[3 * i + 1] = q.y; vps[3 * i + 2] = (world.floor.floorAt(q.x, q.y) ?? 0) + 1; });
  const seg = evaluateSegmentation(segmentPoints(pts, { viewpoints: vps }).labels, Uint8Array.from(h.cat));
  const iou = Object.fromEntries(seg.perClass.map((c) => [c.name, c.iou ?? 0]));
  const row = {
    strategy, seed, finished: st.state === 'complete' ? 1 : 0, sim_time_s: +t.toFixed(0), coverage_pct: +(st.coverage * 100).toFixed(1), distance_m: +st.travelled.toFixed(0),
    ate_odom_m: +m.ateOdom.toFixed(2), ate_slam_m: +m.ate.toFixed(2), rpe10s_odom_m: +m.rpeOdom.toFixed(3), rpe10s_slam_m: +m.rpe.toFixed(3),
    final_err_odom_m: +m.finalErrorOdom.toFixed(2), final_err_slam_m: +m.finalError.toFixed(2), loop_closures: m.loopClosures, loops_rejected: m.rejectedLoops,
    seg_accuracy_pct: +(seg.accuracy * 100).toFixed(1), seg_miou_pct: +(seg.meanIoU * 100).toFixed(1),
    iou_floor: +iou.floor.toFixed(2), iou_ceiling: +iou.ceiling.toFixed(2), iou_wall: +iou.wall.toFixed(2), iou_object: +iou.object.toFixed(2), wall_clock_s: +((Date.now() - t0) / 1000).toFixed(0),
  };
  rows.push(row);
  console.log(JSON.stringify(row));
}
const cols = Object.keys(rows[0]);
fs.mkdirSync(path.dirname(out), { recursive: true });
fs.writeFileSync(out + '.csv', [cols.join(','), ...rows.map((r) => cols.map((c) => r[c]).join(','))].join('\n') + '\n');
const mean = (k, list) => list.reduce((a, r) => a + r[k], 0) / list.length;
const summary = names.map((strategy) => {
  const l = rows.filter((r) => r.strategy === strategy);
  return Object.fromEntries([['strategy', strategy], ...['sim_time_s', 'coverage_pct', 'distance_m', 'ate_odom_m', 'ate_slam_m', 'rpe10s_odom_m', 'rpe10s_slam_m', 'final_err_odom_m', 'final_err_slam_m', 'loop_closures', 'seg_accuracy_pct', 'seg_miou_pct'].map((k) => [k, +mean(k, l).toFixed(2)])]);
});
fs.writeFileSync(out + '_summary.json', JSON.stringify({ seeds, maxSimSeconds: maxT, strategies: STRATEGIES, summary }, null, 2));
console.log('\nSUMMARY (mean over seeds)\n' + summary.map((r) => JSON.stringify(r)).join('\n') + `\nwritten ${out}.csv and ${out}_summary.json`);
