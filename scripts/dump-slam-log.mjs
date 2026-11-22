// Records a cave exploration run (true pose, simulated odometry, the 2D scan points used for matching, and the JS SLAM result)
// so the Python/ROS 2 port of the estimator can be replayed on identical input and compared.
// Usage: node scripts/dump-slam-log.mjs [simSeconds] [seed] [outFile]
import { writeFileSync } from 'node:fs';
import { buildCaveWorld } from '../src/cave/CaveGeometry.js';
import { createCaveMapper } from '../src/cave/CaveMapper.js';
import { createRobotController } from '../src/RobotController.js';

const maxT = Number(process.argv[2] ?? 240), seed = Number(process.argv[3] ?? 7), out = process.argv[4] ?? 'slam_log.json';
let s = seed;
Math.random = () => ((s = (s * 16807) % 2147483647) / 2147483647);
const world = buildCaveWorld();
const sp = world.config.robot;
const ctl = createRobotController({ setPose() {}, wheels: [], lidarHead: { rotation: {} } }, [], { spawn: [sp.spawn[0], sp.spawn[1], 0], yaw: sp.yaw });
const mapper = createCaveMapper({ world, controller: ctl, slamOptions: { seed, keepScans: true } });
for (let t = 0; t < maxT; t += 0.05) mapper.update(0.05);
const slam = mapper.slam, m = slam.metrics();
const r3 = (v) => Math.round(v * 1000) / 1000;
const log = {
  start: [sp.spawn[0], sp.spawn[1], sp.yaw],
  samples: slam.samples.map((q, i) => ({
    t: r3(q.t), truth: q.truth.map(r3), odom: q.odom.map((v) => Math.round(v * 1e5) / 1e5), pts: Array.from(q.pts, r3), est: slam.samplePose(i).map(r3),
  })),
  js: { ate: m.ate, ateOdom: m.ateOdom, rpe: m.rpe, rpeOdom: m.rpeOdom, finalError: m.finalError, finalErrorOdom: m.finalErrorOdom, loopClosures: m.loopClosures, keyframes: m.keyframes },
};
writeFileSync(out, JSON.stringify(log));
console.log(`wrote ${out}: ${log.samples.length} sweeps`, JSON.stringify(log.js));
