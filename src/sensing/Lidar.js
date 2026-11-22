// Simulated 16-beam spinning 3D LiDAR (VLP-16-like geometry) driven by analytic ray casting.
import { castRay } from './Colliders.js';

export const LIDAR = {
  beams: 16, elevMinDeg: -15, elevMaxDeg: 15, azStepDeg: 1.5, maxRange: 25, minRange: 0.3, noiseSigma: 0.02,
};

const deg = Math.PI / 180;

// Deterministic-ish Gaussian noise (Box-Muller) with an injectable RNG so tests can be repeatable.
function gauss(rng) {
  const u = Math.max(rng(), 1e-9), v = rng();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
}

/**
 * One full 360-degree sweep from `pose` = { x, y, z (sensor height), yaw }.
 * Returns hits (world frame, ROS axes) and per-ray info needed for free-space carving.
 */
export function scan(world, pose, rng = Math.random, cfg = LIDAR) {
  const hits = [];
  const rays = [];
  const azCount = Math.round(360 / cfg.azStepDeg);
  for (let b = 0; b < cfg.beams; b++) {
    const el = (cfg.elevMinDeg + (b * (cfg.elevMaxDeg - cfg.elevMinDeg)) / (cfg.beams - 1)) * deg;
    const ce = Math.cos(el), se = Math.sin(el);
    for (let a = 0; a < azCount; a++) {
      const az = pose.yaw + a * cfg.azStepDeg * deg;
      const dx = Math.cos(az) * ce, dy = Math.sin(az) * ce, dz = se;
      const h = castRay(world, pose.x, pose.y, pose.z, dx, dy, dz, cfg.maxRange);
      const range = h ? h.t + gauss(rng) * cfg.noiseSigma : cfg.maxRange;
      const hit = !!h && range >= cfg.minRange;
      rays.push({ dx, dy, dz, range, hit, cat: h?.cat ?? null });
      if (hit) hits.push([pose.x + dx * range, pose.y + dy * range, pose.z + dz * range, h.cat]);
    }
  }
  return { hits, rays };
}
