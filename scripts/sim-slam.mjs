// Headless run: autonomous exploration of the cave with the shadow SLAM estimator; prints trajectory error over time.
// Usage: node scripts/sim-slam.mjs [maxSimSeconds] [seed]
import { buildCaveWorld } from '../src/cave/CaveGeometry.js';
import { createCaveMapper } from '../src/cave/CaveMapper.js';
import { createRobotController } from '../src/RobotController.js';

const maxT = Number(process.argv[2] ?? 900), seed = Number(process.argv[3] ?? 7);
let s = seed;
Math.random = () => ((s = (s * 16807) % 2147483647) / 2147483647); // deterministic lidar noise
const world = buildCaveWorld();
const sp = world.config.robot;
const ctl = createRobotController({ setPose() {}, wheels: [], lidarHead: { rotation: {} } }, [], { spawn: [sp.spawn[0], sp.spawn[1], 0], yaw: sp.yaw });
const mapper = createCaveMapper({ world, controller: ctl, slamOptions: { seed } });
const t0 = Date.now(); let t = 0;
for (; t < maxT; t += 0.05) {
  mapper.update(0.05);
  if (mapper.stats().state === 'complete') break;
  if (Math.round(t / 0.05) % 1200 === 0) {
    const m = mapper.slam.metrics();
    console.log(`t=${t.toFixed(0)}s cov=${(mapper.stats().coverage * 100).toFixed(0)}% kf=${m.keyframes} loops=${m.loopClosures}(rej ${m.rejectedLoops}) ATE slam=${m.ate.toFixed(2)} odom=${m.ateOdom.toFixed(2)} | final err slam=${m.finalError.toFixed(2)} odom=${m.finalErrorOdom.toFixed(2)}`);
  }
}
const m = mapper.slam.metrics();
console.log('END', t.toFixed(0) + 's', JSON.stringify(m, (k, v) => (typeof v === 'number' ? +v.toFixed(3) : v)), 'wall', ((Date.now() - t0) / 1000).toFixed(1) + 's');
console.log('loops', mapper.slam.loopClosures.map((l) => `${l.from}->${l.to} fit=${l.fitness.toFixed(2)} corr=${l.correction.toFixed(1)} info=${l.info.toFixed(0)}`).join(' | '));
