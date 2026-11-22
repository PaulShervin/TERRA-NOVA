// Headless closed-loop exploration test: simulated LiDAR -> occupancy grid -> frontier planner -> controller.
// Usage: node scripts/sim-explore.mjs [maxSimSeconds]
import { buildLayout } from '../src/layout/index.js';
import { buildColliders } from '../src/sensing/Colliders.js';
import { scan } from '../src/sensing/Lidar.js';
import { createOccupancyGrid, buildTruthFree } from '../src/mapping/OccupancyGrid.js';
import { createExplorer } from '../src/mapping/Explorer.js';
import { createRobotController } from '../src/RobotController.js';
import { ROBOT_SCALE } from '../src/Robot.js';

const maxT = Number(process.argv[2] ?? 1500);
const L = buildLayout();
const world = buildColliders(L);
const grid = createOccupancyGrid(L.meta.bounds, 0.2);
const truth = buildTruthFree(grid, world);
const mock = { setPose() {}, wheels: [], lidarHead: { rotation: {} } };
const ctl = createRobotController(mock, [], L.robot);
ctl.setRoute([]);
const ex = createExplorer({ grid, controller: ctl, truth, home: L.robot.spawn.slice(0, 2) });

// deterministic RNG
let seed = Number(process.argv[3] ?? 7); const rng = () => ((seed = (seed * 16807) % 2147483647) / 2147483647);
const dt = 0.05; let sinceScan = 1, collisions = 0, contacts = 0, minClear = Infinity;
const t0 = Date.now();
let t = 0;
for (; t < maxT; t += dt) {
  sinceScan += dt;
  if (sinceScan >= 0.2) {
    const { rays } = scan(world, { x: ctl.pose.x, y: ctl.pose.y, z: 0.72 * ROBOT_SCALE, yaw: ctl.pose.yaw }, rng);
    grid.integrate({ x: ctl.pose.x, y: ctl.pose.y, z: 0.72 * ROBOT_SCALE }, rays);
    sinceScan = 0;
  }
  ex.update(dt);
  ctl.update(dt);
  // ground-truth clearance of the robot centre to any obstacle (approx via truth grid)
  const [ci, cj] = grid.toCell(ctl.pose.x, ctl.pose.y);
  { // body-clearance probe: any blocked truth cell within 0.3 m of the centre counts as contact
    let contact = false;
    for (let dj = -2; dj <= 2 && !contact; dj++) for (let di = -2; di <= 2; di++) if (di * di + dj * dj <= 2.25 && !truth.truth[grid.idx(ci + di, cj + dj)]) { contact = true; break; }
    if (contact) contacts++;
  }
  if (!truth.truth[grid.idx(ci, cj)]) { collisions++; if (collisions <= 12) console.log(`  collision t=${t.toFixed(1)} at (${ctl.pose.x.toFixed(2)},${ctl.pose.y.toFixed(2)})`); }
  if (ex.status.state === 'complete') break;
  if (Math.round(t / dt) % 1200 === 0) console.log(`t=${t.toFixed(0)}s state=${ex.status.state} cov=${(ex.status.coverage * 100).toFixed(1)}% frontiers=${ex.status.frontiers} dist=${ex.status.travelled.toFixed(0)}m pos=(${ctl.pose.x.toFixed(1)},${ctl.pose.y.toFixed(1)})`);
}
console.log(`pose=(${ctl.pose.x.toFixed(1)},${ctl.pose.y.toFixed(1)}) home=${L.robot.spawn.slice(0,2)}`);
console.log(`END t=${t.toFixed(0)}s state=${ex.status.state} coverage=${(ex.status.coverage * 100).toFixed(1)}% travelled=${ex.status.travelled.toFixed(0)}m centre-in-obstacle samples=${collisions} body-contact samples=${contacts} wall-clock=${((Date.now() - t0) / 1000).toFixed(1)}s`);
