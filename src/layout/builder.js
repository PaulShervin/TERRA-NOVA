// Tiny helpers that produce plain, JSON-serialisable layout entries (no Three.js here).
let counter = 0;
export const resetIds = () => { counter = 0; };

/** Create one layout object. position = base-centre [x,y,z]; size = [sx,sy,sz] bounding box in the object's local frame. */
export function obj(category, model, { name, position, size, yaw = 0, material, params }) {
  counter += 1;
  return {
    id: `${category}_${String(counter).padStart(3, '0')}`,
    name: name ?? `${model}_${counter}`,
    category, model,
    material: material ?? null,
    position: [position[0], position[1], position[2] ?? 0],
    size,
    yaw,
    ...(params ? { params } : {}),
  };
}

/** Axis-aligned wall from a start/end point on one axis. */
export const wallAlongY = (x, y0, y1, h, t, name) => obj('wall', 'box', {
  name, material: 'wall', position: [x, (y0 + y1) / 2, 0], size: [t, y1 - y0, h],
});
export const wallAlongX = (y, x0, x1, h, t, name) => obj('wall', 'box', {
  name, material: 'wall', position: [(x0 + x1) / 2, y, 0], size: [x1 - x0, t, h],
});

/** Cartesian grid helper: returns [x,y] pairs. */
export const grid = (xs, ys) => xs.flatMap((x) => ys.map((y) => [x, y]));
