// 2D log-odds occupancy grid built from the 3D LiDAR rays (ROS axes, metres).
// Only obstacles between ~0.1 m and 1.5 m high count as occupied: overhead pipes / the platform deck are traversable.
export const OBSTACLE_Z_MIN = 0.1;
export const OBSTACLE_Z_MAX = 1.5;
const L_OCC = 1.0, L_FREE = -0.25, L_MIN = -2, L_MAX = 5;
export const UNKNOWN = 0, FREE = 1, OCC = 2;

export function createOccupancyGrid(bounds, res = 0.2) {
  const W = Math.round((bounds.xMax - bounds.xMin) / res);
  const H = Math.round((bounds.yMax - bounds.yMin) / res);
  const logOdds = new Float32Array(W * H);
  const seen = new Uint8Array(W * H);
  const occStamp = new Uint32Array(W * H);
  const freeStamp = new Uint32Array(W * H);
  let scanId = 0;

  const idx = (i, j) => j * W + i;
  const inside = (i, j) => i >= 0 && j >= 0 && i < W && j < H;
  const toCell = (x, y) => [Math.floor((x - bounds.xMin) / res), Math.floor((y - bounds.yMin) / res)];
  const toWorld = (i, j) => [bounds.xMin + (i + 0.5) * res, bounds.yMin + (j + 0.5) * res];

  const stateAt = (k) => (!seen[k] ? UNKNOWN : logOdds[k] > 0.85 ? OCC : logOdds[k] < -0.35 ? FREE : UNKNOWN);

  function bump(k, delta) {
    seen[k] = 1;
    const v = logOdds[k] + delta;
    logOdds[k] = v > L_MAX ? L_MAX : v < L_MIN ? L_MIN : v;
  }

  /** Integrate a scan: pose = {x,y,z}, rays = from Lidar.scan(). */
  function integrate(pose, rays) {
    scanId += 1;
    const [oi, oj] = toCell(pose.x, pose.y);
    // Pass 1: occupied end cells
    const jobs = [];
    for (const r of rays) {
      const ce = Math.hypot(r.dx, r.dy);
      if (ce < 1e-6) continue;
      const hx = r.dx / ce, hy = r.dy / ce;
      let reach = r.range * ce; // horizontal distance we can vouch for
      let occupiedEnd = false;
      if (r.hit) {
        const zHit = pose.z + r.dz * r.range;
        if (r.cat !== 'floor' && zHit >= OBSTACLE_Z_MIN && zHit <= OBSTACLE_Z_MAX) occupiedEnd = true;
      }
      if (r.dz > 1e-6) { // rising ray: only free below the obstacle band
        const tLim = (OBSTACLE_Z_MAX - pose.z) / r.dz;
        const hLim = tLim * ce;
        if (hLim < reach) { reach = hLim; occupiedEnd = false; }
      }
      jobs.push([hx, hy, reach, occupiedEnd]);
      if (occupiedEnd) {
        const [i, j] = toCell(pose.x + hx * reach, pose.y + hy * reach);
        if (inside(i, j)) {
          const k = idx(i, j);
          if (occStamp[k] !== scanId) { occStamp[k] = scanId; bump(k, L_OCC); }
        }
      }
    }
    // Pass 2: free space carving (each cell at most once per scan; never over an occupied cell of this scan)
    for (const [hx, hy, reach, occupiedEnd] of jobs) {
      const stop = occupiedEnd ? reach - res * 1.5 : reach;
      for (let d = 0; d <= stop; d += res * 0.5) {
        const [i, j] = toCell(pose.x + hx * d, pose.y + hy * d);
        if (!inside(i, j)) break;
        const k = idx(i, j);
        if (freeStamp[k] === scanId || occStamp[k] === scanId) continue;
        freeStamp[k] = scanId;
        bump(k, L_FREE);
      }
    }
    return [oi, oj];
  }

  return { W, H, res, bounds, logOdds, seen, idx, inside, toCell, toWorld, stateAt, integrate };
}

/** Ground-truth traversable cells from the collider world (for the explored-% metric only). */
export function buildTruthFree(grid, world) {
  const truth = new Uint8Array(grid.W * grid.H);
  let count = 0;
  for (let j = 0; j < grid.H; j++) {
    for (let i = 0; i < grid.W; i++) {
      const [x, y] = grid.toWorld(i, j);
      let blocked = false;
      for (const it of world.items) {
        if (it.z1 < OBSTACLE_Z_MIN || it.z0 > OBSTACLE_Z_MAX) continue;
        if (x < it.x0 || x > it.x1 || y < it.y0 || y > it.y1) continue;
        if (it.t === 'b') {
          const lx = it.c * (x - it.cx) + it.s * (y - it.cy), ly = -it.s * (x - it.cx) + it.c * (y - it.cy);
          if (Math.abs(lx) <= it.hx && Math.abs(ly) <= it.hy) { blocked = true; break; }
        } else if ((x - it.cx) ** 2 + (y - it.cy) ** 2 <= it.r * it.r) { blocked = true; break; }
      }
      if (!blocked) { truth[grid.idx(i, j)] = 1; count++; }
    }
  }
  return { truth, count };
}
