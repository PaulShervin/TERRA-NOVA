// Simulated 16-beam 3D LiDAR for the cave: sphere-traces the SAME signed-distance field the mesh is built from,
// then labels every return (floor / wall / ceiling / pillar / boulder / stalactite) and its height above the local floor.
import { LIDAR } from '../sensing/Lidar.js';

const deg = Math.PI / 180;

function gauss(rng) {
  const u = Math.max(rng(), 1e-9), v = rng();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
}

/** Semantic label of a hit: nearest solid formation if close, otherwise by surface orientation. */
export function classifyHit(world, p, n) {
  const [x, y, z] = p;
  for (const s of world.solids) {
    if (s.kind === 'pillar') {
      if (Math.hypot(x - s.x, y - s.y) < s.radius * 1.4 + 0.35 && z > s.zFloor - 0.5 && z < s.zFloor + s.height + 0.5) return 'pillar';
    } else if (s.kind === 'boulder') {
      const co = Math.cos(s.yaw), si = Math.sin(s.yaw), dx = x - s.x, dy = y - s.y;
      const lx = (co * dx + si * dy) / s.radii[0], ly = (-si * dx + co * dy) / s.radii[1], lz = (z - s.z) / s.radii[2];
      if (Math.sqrt(lx * lx + ly * ly + lz * lz) < 1.45) return 'boulder';
    } else if (Math.hypot(x - s.x, y - s.y) < s.radius * 2.2 + 0.25 && z > s.zCeil - s.length - 0.4 && n[2] < 0.35) return 'stalactite';
  }
  if (n[2] > 0.7) return 'floor';
  if (n[2] < -0.5) return 'ceiling';
  return 'wall';
}

/**
 * One 360-degree sweep. pose = { x, y, z (sensor height, world), yaw }.
 * Each ray: { dx, dy, dz, range, hit, cat, relZ, floorLike } (relZ = height of the return above the local floor).
 */
export function scanCave(world, pose, rng = Math.random, cfg = LIDAR) {
  const rays = [], hits = [];
  const azCount = Math.round(360 / cfg.azStepDeg);
  for (let b = 0; b < cfg.beams; b++) {
    const el = (cfg.elevMinDeg + (b * (cfg.elevMaxDeg - cfg.elevMinDeg)) / (cfg.beams - 1)) * deg;
    const ce = Math.cos(el), se = Math.sin(el);
    for (let a = 0; a < azCount; a++) {
      const az = pose.yaw + a * cfg.azStepDeg * deg;
      const dx = Math.cos(az) * ce, dy = Math.sin(az) * ce, dz = se;
      const h = world.field.cast(pose.x, pose.y, pose.z, dx, dy, dz, cfg.maxRange);
      if (!h) { rays.push({ dx, dy, dz, range: cfg.maxRange, hit: false, cat: null, relZ: 99, floorLike: false }); continue; }
      const range = h.t + gauss(rng) * cfg.noiseSigma;
      const hit = range >= cfg.minRange;
      const fz = world.floor.floorAt(h.point[0], h.point[1]);
      const relZ = fz === null ? 99 : h.point[2] - fz;
      const cat = classifyHit(world, h.point, h.normal);
      const floorLike = h.normal[2] > 0.7 && relZ < 0.28;
      rays.push({ dx, dy, dz, range, hit, cat, relZ, floorLike });
      if (hit) hits.push([pose.x + dx * range, pose.y + dy * range, pose.z + dz * range, cat]);
    }
  }
  return { rays, hits };
}
