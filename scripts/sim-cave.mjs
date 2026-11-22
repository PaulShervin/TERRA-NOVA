// Headless closed-loop exploration of the CAVE: LiDAR (sphere tracing) -> occupancy -> frontiers -> planner -> controller.
// Usage: node scripts/sim-cave.mjs [maxSimSeconds] [seed]
import { buildCaveWorld } from '../src/cave/CaveGeometry.js';
import { createCaveMapper } from '../src/cave/CaveMapper.js';
import { createRobotController } from '../src/RobotController.js';

const maxT = Number(process.argv[2] ?? 1500);
let seed = Number(process.argv[3] ?? 7);
const origRandom = Math.random;
Math.random = () => ((seed = (seed * 16807) % 2147483647) / 2147483647); // deterministic lidar noise

const t0 = Date.now();
const world = buildCaveWorld();
console.log(`world built in ${Date.now() - t0} ms`);
const mock = { setPose() {}, wheels: [], lidarHead: { rotation: {} } };
const spawn = world.config.robot;
const ctl = createRobotController(mock, [], { spawn: [spawn.spawn[0], spawn.spawn[1], 0], yaw: spawn.yaw });
const mapper = createCaveMapper({ world, controller: ctl });

const dt = 0.05; let t = 0, contact = 0, inRock = 0, worstSlope = 0;
const t1 = Date.now();
for (; t < maxT; t += dt) {
  mapper.update(dt);
  const p = ctl.pose;
  // physical checks against the real field: robot centre must be in air with >= 0.45 m of lateral clearance at 0.5 m height
  const fz = world.floor.floorAt(p.x, p.y);
  if (fz === null) { inRock++; } else {
    const zc = fz + 0.5;
    if (world.field.sample(p.x, p.y, zc) > -0.02) inRock++;
    else for (const a of [0, 1.57, 3.14, 4.71]) if (world.field.sample(p.x + Math.cos(a) * 0.4, p.y + Math.sin(a) * 0.4, zc) > 0) { contact++; break; }
  }
  const st = mapper.stats();
  if (st.state === 'complete') break;
  if (Math.round(t / dt) % 2400 === 0) console.log(`t=${t.toFixed(0)}s ${st.state} cov=${(st.coverage * 100).toFixed(1)}% frontiers=${st.frontiers} dist=${st.travelled.toFixed(0)}m pos=(${p.x.toFixed(1)},${p.y.toFixed(1)})`);
}
const st = mapper.stats();
console.log(`END t=${t.toFixed(0)}s state=${st.state} coverage=${(st.coverage * 100).toFixed(1)}% travelled=${st.travelled.toFixed(0)}m points=${st.points} centre-in-rock=${inRock} wall-contact(0.4m)=${contact} wall-clock=${((Date.now() - t1) / 1000).toFixed(1)}s`);
console.log(`final pose (${ctl.pose.x.toFixed(1)}, ${ctl.pose.y.toFixed(1)}) home (${spawn.spawn[0]}, ${spawn.spawn[1]})`);
Math.random = origRandom;
