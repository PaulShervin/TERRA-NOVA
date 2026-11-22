// Cave version of the sensing -> mapping -> exploration loop. Reuses the generic occupancy-grid storage, the frontier
// Explorer, the planner and the map point cloud; only the sensor world and the height logic (sloping floors) differ.
import { scanCave } from './CaveLidar.js';
import { createOccupancyGrid } from '../mapping/OccupancyGrid.js';
import { createExplorer } from '../mapping/Explorer.js';
import { createMapCloud } from '../mapping/MapCloud.js';
import { ROBOT_SCALE } from '../Robot.js';
import { createSlamEstimator } from '../slam/SlamEstimator.js';

const SENSOR_HEIGHT = 0.72 * ROBOT_SCALE; // above the local floor
const SCAN_PERIOD = 0.2;
const OBSTACLE_MIN = 0.15, OBSTACLE_MAX = 1.7; // m above the local floor
const L_OCC = 1.0, L_FREE = -0.25, L_MIN = -2, L_MAX = 5;

/** Occupancy update with a sloping floor: heights are measured relative to the terrain under each ray point. */
function createCaveIntegrator(grid, floorAt) {
  const occStamp = new Uint32Array(grid.W * grid.H), freeStamp = new Uint32Array(grid.W * grid.H);
  let scanId = 0;
  const bump = (k, d) => { grid.seen[k] = 1; const v = grid.logOdds[k] + d; grid.logOdds[k] = v > L_MAX ? L_MAX : v < L_MIN ? L_MIN : v; };
  return function integrate(pose, rays) {
    scanId++;
    for (const r of rays) { // pass 1: obstacle returns
      if (!r.hit || r.floorLike || r.relZ < OBSTACLE_MIN || r.relZ > OBSTACLE_MAX) continue;
      const [i, j] = grid.toCell(pose.x + r.dx * r.range, pose.y + r.dy * r.range);
      if (!grid.inside(i, j)) continue;
      const k = grid.idx(i, j);
      if (occStamp[k] !== scanId) { occStamp[k] = scanId; bump(k, L_OCC); }
    }
    for (const r of rays) { // pass 2: carve free space while the ray is still within robot height of the floor
      const stop = r.hit && !r.floorLike && r.relZ >= OBSTACLE_MIN && r.relZ <= OBSTACLE_MAX ? r.range - grid.res * 1.5 : r.range;
      for (let t = 0; t <= stop; t += grid.res * 0.5) {
        const x = pose.x + r.dx * t, y = pose.y + r.dy * t, z = pose.z + r.dz * t;
        const fz = floorAt(x, y);
        if (fz === null || z - fz > OBSTACLE_MAX) break;
        const [i, j] = grid.toCell(x, y);
        if (!grid.inside(i, j)) break;
        const k = grid.idx(i, j);
        if (freeStamp[k] === scanId || occStamp[k] === scanId) continue;
        freeStamp[k] = scanId;
        bump(k, L_FREE);
      }
    }
  };
}

export function createCaveMapper({ world, controller, onScan, explorerOptions = {}, slamOptions = {}, storeHits = true }) {
  const cloud = createMapCloud();
  const home = world.config.robot.spawn.slice(0, 2);
  const trajectory = []; // driven path samples { x, y, t } recorded inside the sim loop (for the debug trail / loop heuristic)
  let grid, truth, explorer, integrate, slam;
  // voxel-deduplicated hit store (incl. ceiling) with the simulator category: input for the label-free segmentation + its evaluation
  const hitStore = { x: [], y: [], z: [], cat: [], seen: new Set() };
  const CAT_CODE = { floor: 0, ceiling: 1, wall: 2, pillar: 3, boulder: 3, stalactite: 3 };
  let mode = 'explore', simSpeed = 1, sinceScan = SCAN_PERIOD, scans = 0, simTime = 0, lastScan = null;

  function build() {
    grid = createOccupancyGrid(world.bounds, 0.25);
    // ground truth drivable cells (coverage metric only): nearest floor column that is drivable
    const f = world.floor, t = new Uint8Array(grid.W * grid.H);
    let count = 0;
    for (let j = 0; j < grid.H; j++) {
      for (let i = 0; i < grid.W; i++) {
        const [x, y] = grid.toWorld(i, j);
        const ci = Math.round((x - world.field.ox) / f.voxel), cj = Math.round((y - world.field.oy) / f.voxel);
        if (ci > 0 && cj > 0 && ci < f.nx - 1 && cj < f.ny - 1 && f.drivable[cj * f.nx + ci]) { t[grid.idx(i, j)] = 1; count++; }
      }
    }
    truth = { truth: t, count };
    integrate = createCaveIntegrator(grid, f.floorAt);
    explorer = createExplorer({ grid, controller, truth, home, inflations: [0.5, 0.45, 0.4], ...explorerOptions });
    slam = createSlamEstimator(slamOptions); slam.init({ x: controller.pose.x, y: controller.pose.y, yaw: controller.pose.yaw });
    hitStore.x.length = hitStore.y.length = hitStore.z.length = hitStore.cat.length = 0; hitStore.seen.clear();
    cloud.clear(); sinceScan = SCAN_PERIOD; scans = 0;
  }
  build();
  controller.setRoute([]);

  function update(realDt) {
    const dt = Math.min(realDt, 0.1) * simSpeed;
    sinceScan += dt; simTime += dt;
    if (sinceScan >= SCAN_PERIOD) {
      const p = controller.pose;
      const floorZ = world.floor.floorAt(p.x, p.y) ?? 0;
      const pose = { x: p.x, y: p.y, z: floorZ + SENSOR_HEIGHT, yaw: p.yaw };
      const { rays, hits } = scanCave(world, pose);
      integrate(pose, rays);
      slam.update(p, rays, simTime);
      if (storeHits) for (const h of hits) {
        const key = Math.round(h[0] * 4) * 1e8 + Math.round(h[1] * 4) * 1e4 + Math.round(h[2] * 4);
        if (hitStore.seen.has(key)) continue;
        hitStore.seen.add(key); hitStore.x.push(h[0]); hitStore.y.push(h[1]); hitStore.z.push(h[2]); hitStore.cat.push(CAT_CODE[h[3]] ?? 2);
      }
      cloud.add(hits.filter((h) => h[3] !== 'ceiling')); // ceiling returns stay in the occupancy logic but are not drawn (keeps the 3D map readable)
      lastScan = { rays, pose, time: simTime, hits: hits.length, floorZ };
      onScan?.({ ...lastScan });
      sinceScan = 0; scans++;
    }
    if (mode === 'explore') explorer.update(dt);
    controller.update(dt);
    const q = controller.pose, last = trajectory[trajectory.length - 1];
    if (!last || Math.hypot(q.x - last.x, q.y - last.y) > 0.2) trajectory.push({ x: q.x, y: q.y, t: simTime });
  }

  return {
    update, cloud, trajectory,
    get grid() { return grid; }, get slam() { return slam; }, hitStore, get explorer() { return explorer; }, get truth() { return truth; },
    get simTime() { return simTime; }, get lastScan() { return lastScan; },
    stats: () => ({ points: cloud.count(), scans, coverage: explorer.status.coverage, state: mode === 'explore' ? explorer.status.state : mode, travelled: explorer.status.travelled, frontiers: explorer.status.frontiers }),
    setSimSpeed: (v) => { simSpeed = v; },
    setMode(m) { mode = m; controller.setRoute([]); },
    getMode: () => mode,
    advance(seconds) { for (let t = 0; t < seconds; t += 0.05) update(0.05 / simSpeed); },
    reset() {
      const s = world.config.robot;
      controller.teleport(s.spawn[0], s.spawn[1], s.yaw);
      trajectory.length = 0;
      build(); controller.setRoute([]);
    },
    dispose: () => cloud.dispose(),
  };
}
