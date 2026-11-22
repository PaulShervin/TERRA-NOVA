// Ties the sensing + mapping + exploration pieces together and advances them in simulation time.
import { buildColliders } from '../sensing/Colliders.js';
import { scan } from '../sensing/Lidar.js';
import { createOccupancyGrid, buildTruthFree } from './OccupancyGrid.js';
import { createExplorer } from './Explorer.js';
import { createMapCloud } from './MapCloud.js';

import { ROBOT_SCALE } from '../Robot.js';

const SENSOR_HEIGHT = 0.72 * ROBOT_SCALE; // LiDAR height above the floor (world metres)
const SCAN_PERIOD = 0.2; // s of sim time (5 Hz sweeps)

export function createMapper({ layout, controller, robot, onScan }) {
  const world = buildColliders(layout);
  const cloud = createMapCloud();
  let grid, truth, explorer;
  let mode = 'explore';
  let lastScan = null;
  let simSpeed = 1, sinceScan = SCAN_PERIOD, scans = 0, simTime = 0;
  const home = layout.robot.spawn.slice(0, 2);

  function build() {
    grid = createOccupancyGrid(layout.meta.bounds, 0.2);
    truth = buildTruthFree(grid, world);
    explorer = createExplorer({ grid, controller, truth, home });
    cloud.clear();
    sinceScan = SCAN_PERIOD; scans = 0;
  }
  build();
  controller.setRoute([]);

  function update(realDt) {
    const dt = Math.min(realDt, 0.1) * simSpeed;
    sinceScan += dt;
    simTime += dt;
    if (sinceScan >= SCAN_PERIOD) {
      const p = controller.pose;
      const { hits, rays } = scan(world, { x: p.x, y: p.y, z: SENSOR_HEIGHT, yaw: p.yaw });
      grid.integrate({ x: p.x, y: p.y, z: SENSOR_HEIGHT }, rays);
      cloud.add(hits);
      lastScan = { rays, pose: { x: p.x, y: p.y, z: SENSOR_HEIGHT, yaw: p.yaw }, time: simTime, hits: hits.length };
      onScan?.({ rays, pose: { x: p.x, y: p.y, z: SENSOR_HEIGHT, yaw: p.yaw }, time: simTime });
      sinceScan = 0; scans++;
    }
    if (mode === 'explore') explorer.update(dt);
    controller.update(dt);
  }

  return {
    update, cloud,
    get simTime() { return simTime; },
    get lastScan() { return lastScan; },
    get grid() { return grid; },
    get explorer() { return explorer; },
    stats: () => ({ points: cloud.count(), scans, coverage: explorer.status.coverage, state: mode === 'explore' ? explorer.status.state : mode, travelled: explorer.status.travelled, frontiers: explorer.status.frontiers }),
    setSimSpeed: (v) => { simSpeed = v; },
    setMode(m) {
      mode = m;
      if (m === 'patrol') controller.setRoute(layout.navPath, { loop: true });
      else controller.setRoute([]);
    },
    getMode: () => mode,
    /** Step the simulation forward by `seconds` of sim time without rendering (used by tests / fast-forward). */
    advance(seconds) { for (let t = 0; t < seconds; t += 0.05) update(0.05 / simSpeed); },
    /** Wipe the map and put the robot back at its spawn pose. */
    reset() {
      robot.setPose(layout.robot.spawn, layout.robot.yaw);
      Object.assign(controller.pose, { x: layout.robot.spawn[0], y: layout.robot.spawn[1], yaw: layout.robot.yaw });
      controller.teleport(layout.robot.spawn[0], layout.robot.spawn[1], layout.robot.yaw);
      build();
      controller.setRoute([]);
    },
    dispose: () => cloud.dispose(),
  };
}
