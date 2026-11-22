// Composite machinery models. Each builder receives (entry, materials) and returns a THREE.Group.
import { box, cylinder, cylinderX, group } from './parts.js';

export function crusher(e, m) {
  const [sx, sy, sz] = e.size;
  return group(e, [
    box(m.darkSteel, [sx, sy, 1.0], [0, 0, 0.5]), // base frame
    box(m[e.material], [sx * 0.8, sy * 0.85, sz * 0.55], [0, 0, 1.0 + (sz * 0.55) / 2]), // crushing chamber
    box(m.steel, [sx * 0.9, sy * 0.95, 0.15], [0, 0, sz * 0.55 + 1.0 + 0.5], [0, -0.18, 0]), // inclined hopper lip
    box(m.machineYellow, [0.5, sy * 0.6, 1.4], [-sx / 2 + 0.3, 0, 1.2]), // flywheel guard
    cylinder(m.steel, 0.55, 0.35, [-sx / 2 - 0.05, 0, 1.6]),
    box(m.darkSteel, [1.2, 1.0, 0.9], [sx / 2 - 0.7, -sy / 2 + 0.6, 1.45]), // drive motor
  ]);
}

export function generator(e, m) {
  const [sx, sy, sz] = e.size;
  return group(e, [
    box(m.darkSteel, [sx, sy, 0.25], [0, 0, 0.125]), // skid
    box(m[e.material], [sx * 0.95, sy * 0.95, sz - 0.5], [0, 0, 0.25 + (sz - 0.5) / 2]), // enclosure
    cylinder(m.darkSteel, 0.12, 0.6, [sx * 0.3, 0, sz - 0.05]), // exhaust stack
    box(m.steel, [0.6, sy * 0.9, 0.6], [-sx * 0.4, 0, sz - 0.3]), // radiator
  ]);
}

export function mill(e, m) {
  const [sx, sy, sz] = e.size;
  const r = Math.min(sy, sz - 0.5) / 2;
  return group(e, [
    box(m.concrete, [1.2, sy * 0.8, 0.5], [-sx * 0.38, 0, 0.25]), // trunnion piers
    box(m.concrete, [1.2, sy * 0.8, 0.5], [sx * 0.38, 0, 0.25]),
    cylinderX(m[e.material], r, sx * 0.72, [0, 0, 0.5 + r]), // drum
    cylinderX(m.steel, r * 0.5, sx, [0, 0, 0.5 + r], 16), // axle
    cylinderX(m.darkSteel, r * 1.05, 0.3, [-sx * 0.12, 0, 0.5 + r]), // drive ring gear
    box(m.machineYellow, [1.0, 0.9, 0.9], [-sx * 0.45, sy / 2 + 0.2, 0.45]), // motor
  ]);
}

export function conveyor(e, m) {
  const [sx, sy, sz] = e.size;
  const kids = [];
  if (e.params?.incline) {
    const rise = sz - 0.9;
    const ang = Math.atan2(rise, sx);
    const len = Math.hypot(sx, rise);
    kids.push(box(m.darkSteel, [len, sy, 0.3], [0, 0, 0.45 + rise / 2], [0, -ang, 0]));
    kids.push(box(m.rubber, [len, sy * 0.8, 0.08], [0, 0, 0.62 + rise / 2], [0, -ang, 0]));
    [-0.4, -0.15, 0.1, 0.35, 0.5].forEach((t) => {
      const h = 0.3 + (t + 0.5) * rise;
      kids.push(box(m.steel, [0.12, 0.12, h], [t * sx, 0.4, h / 2]));
      kids.push(box(m.steel, [0.12, 0.12, h], [t * sx, -0.4, h / 2]));
    });
  } else {
    kids.push(box(m.darkSteel, [sx, sy, 0.25], [0, 0, sz - 0.4]));
    kids.push(box(m.rubber, [sx, sy * 0.8, 0.08], [0, 0, sz - 0.24]));
    kids.push(box(m.steel, [sx, 0.06, 0.25], [0, sy / 2, sz - 0.2]), box(m.steel, [sx, 0.06, 0.25], [0, -sy / 2, sz - 0.2]));
    const legs = Math.round(sx / 3) + 1;
    for (let i = 0; i < legs; i++) {
      const x = -sx / 2 + 0.2 + (i * (sx - 0.4)) / (legs - 1);
      kids.push(box(m.steel, [0.15, 0.15, sz - 0.5], [x, sy / 2 - 0.1, (sz - 0.5) / 2]));
      kids.push(box(m.steel, [0.15, 0.15, sz - 0.5], [x, -sy / 2 + 0.1, (sz - 0.5) / 2]));
    }
  }
  return group(e, kids);
}

export function pump(e, m) {
  const [sx, sy, sz] = e.size;
  return group(e, [
    box(m.darkSteel, [sx, sy, 0.15], [0, 0, 0.075]),
    cylinder(m[e.material], sy * 0.38, sz * 0.7, [-sx * 0.2, 0, 0.15 + sz * 0.35]),
    cylinderX(m.steel, sy * 0.3, sx * 0.5, [sx * 0.25, 0, 0.45]),
    cylinder(m.pipeRed, 0.08, sz * 0.5, [sx * 0.4, 0, 0.15 + sz * 0.25]),
  ]);
}
