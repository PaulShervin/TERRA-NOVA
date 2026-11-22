// Tiny SE(2) helpers. A pose is [x, y, theta] (metres, radians, ROS REP-103: x forward, y left, CCW positive).
export const wrap = (a) => { while (a > Math.PI) a -= 2 * Math.PI; while (a < -Math.PI) a += 2 * Math.PI; return a; };

/** a (+) b : apply the relative transform b in the frame of a. */
export function compose(a, b) {
  const c = Math.cos(a[2]), s = Math.sin(a[2]);
  return [a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2])];
}

/** a (-) b : pose b expressed in the frame of a (the relative transform from a to b). */
export function between(a, b) {
  const c = Math.cos(a[2]), s = Math.sin(a[2]), dx = b[0] - a[0], dy = b[1] - a[1];
  return [c * dx + s * dy, -s * dx + c * dy, wrap(b[2] - a[2])];
}

/** Deterministic PRNG (mulberry32) + Gaussian, so estimator noise never depends on Math.random. */
export function createRng(seed = 1) {
  let a = seed >>> 0;
  const uniform = () => { a = (a + 0x6d2b79f5) >>> 0; let t = a; t = Math.imul(t ^ (t >>> 15), t | 1); t ^= t + Math.imul(t ^ (t >>> 7), t | 61); return ((t ^ (t >>> 14)) >>> 0) / 4294967296; };
  const gauss = () => Math.sqrt(-2 * Math.log(Math.max(uniform(), 1e-12))) * Math.cos(2 * Math.PI * uniform());
  return { uniform, gauss };
}
