// Composite structural models: fences, racks, platform + stairs.
import { box, group } from './parts.js';

export function fence(e, m) {
  const [len, , h] = e.size;
  const kids = [
    box(m.fence, [len, 0.03, 0.05], [0, 0, h]), // top rail
    box(m.fence, [len, 0.03, 0.05], [0, 0, h * 0.5]), // mid rail
    box(m.fence, [len, 0.015, h * 0.9], [0, 0, h * 0.5 + 0.05]), // wire mesh panel (thin)
  ];
  const posts = Math.max(2, Math.round(len / 2) + 1);
  for (let i = 0; i < posts; i++) {
    kids.push(box(m.darkSteel, [0.08, 0.08, h], [-len / 2 + (i * len) / (posts - 1), 0, h / 2]));
  }
  return group(e, kids);
}

export function rack(e, m) {
  const [sx, sy, sz] = e.size;
  const kids = [];
  for (const dx of [-sx / 2 + 0.05, sx / 2 - 0.05]) {
    for (const dy of [-sy / 2 + 0.05, sy / 2 - 0.05]) kids.push(box(m[e.material], [0.08, 0.08, sz], [dx, dy, sz / 2]));
  }
  [0.15, 1.1, 2.05].forEach((z, i) => {
    kids.push(box(m.darkSteel, [sx, sy, 0.05], [0, 0, z]));
    const n = 3 - (i % 2);
    for (let k = 0; k < n; k++) {
      const w = sx / 3.4;
      kids.push(box(m.crate, [w, sy * 0.8, 0.6 - 0.1 * (k % 2)], [-sx / 2 + w * (k + 0.7), 0, z + 0.3]));
    }
  });
  return group(e, kids);
}

export function platform(e, m) {
  const [sx, sy, sz] = e.size;
  const deck = 0.15;
  const kids = [box(m.steel, [sx, sy, deck], [0, 0, sz - deck / 2])];
  for (const dx of [-sx / 2 + 0.15, sx / 2 - 0.15]) {
    for (const dy of [-sy / 2 + 0.15, sy / 2 - 0.15]) kids.push(box(m.darkSteel, [0.25, 0.25, sz - deck], [dx, dy, (sz - deck) / 2]));
  }
  // Railing on three sides (open on +x where stairs attach)
  const rh = 1.0;
  const rail = (len, c, alongX) => [
    box(m.machineYellow, alongX ? [len, 0.05, 0.05] : [0.05, len, 0.05], [c[0], c[1], sz + rh]),
    box(m.machineYellow, alongX ? [len, 0.05, 0.05] : [0.05, len, 0.05], [c[0], c[1], sz + rh * 0.5]),
  ];
  kids.push(...rail(sx, [0, sy / 2 - 0.05], true), ...rail(sx, [0, -sy / 2 + 0.05], true), ...rail(sy, [-sx / 2 + 0.05, 0], false));
  for (const [px, py] of [[-sx / 2 + 0.05, sy / 2 - 0.05], [-sx / 2 + 0.05, -sy / 2 + 0.05], [0, sy / 2 - 0.05], [0, -sy / 2 + 0.05], [sx / 2 - 0.05, sy / 2 - 0.05], [sx / 2 - 0.05, -sy / 2 + 0.05]]) {
    kids.push(box(m.machineYellow, [0.05, 0.05, rh], [px, py, sz + rh / 2]));
  }
  return group(e, kids);
}

export function stairs(e, m) {
  const [sx, sy, sz] = e.size;
  const n = 9;
  const kids = [];
  for (let i = 0; i < n; i++) {
    const h = ((i + 1) * sz) / n;
    // Rising toward -x (attaches to the platform's +x edge at the highest step)
    kids.push(box(m.steel, [sx / n, sy, 0.06], [sx / 2 - (i + 0.5) * (sx / n), 0, h]));
  }
  kids.push(box(m.darkSteel, [Math.hypot(sx, sz), 0.06, 0.15], [0, sy / 2, sz / 2], [0, Math.atan2(sz, sx), 0]));
  kids.push(box(m.darkSteel, [Math.hypot(sx, sz), 0.06, 0.15], [0, -sy / 2, sz / 2], [0, Math.atan2(sz, sx), 0]));
  return group(e, kids);
}

export function workbench(e, m) {
  const [sx, sy, sz] = e.size;
  const kids = [
    box(m.wood, [sx, sy, 0.08], [0, 0, sz - 0.04]), // top
    box(m.darkSteel, [sx * 0.96, sy * 0.9, 0.04], [0, 0, 0.25]), // lower shelf
    box(m.toolRed, [0.35, 0.3, 0.2], [-sx * 0.3, 0, sz + 0.1]), // toolbox on bench
    box(m.darkSteel, [0.18, 0.14, 0.14], [sx * 0.38, sy * 0.25, sz + 0.07]), // vise
    box(m.steel, [0.5, 0.06, 0.02], [sx * 0.05, -sy * 0.15, sz + 0.01]), // flat tool
    box(m.crate, [0.4, 0.3, 0.2], [sx * 0.2, 0, 0.37]), // parts bin on shelf
  ];
  for (const dx of [-sx / 2 + 0.06, sx / 2 - 0.06]) {
    for (const dy of [-sy / 2 + 0.06, sy / 2 - 0.06]) kids.push(box(m.darkSteel, [0.07, 0.07, sz - 0.08], [dx, dy, (sz - 0.08) / 2]));
  }
  return group(e, kids);
}
