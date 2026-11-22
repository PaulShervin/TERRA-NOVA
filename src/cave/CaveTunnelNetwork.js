// Turns CaveConfiguration.json into a queryable network: densely sampled tunnel centre-lines, chambers,
// solids (pillars / boulders / stalactites), graph analysis (intersections, dead ends, loops) and landmarks.
// Pure JS (no Three.js). Units: metres, ROS axes.
import config from './CaveConfiguration.json' with { type: 'json' };

export const caveConfig = config;

/** Uniform Catmull-Rom through 5-component control points [x, y, floorZ, width, height]. */
function catmull(p0, p1, p2, p3, t) {
  const t2 = t * t, t3 = t2 * t;
  return p1.map((_, i) => 0.5 * ((2 * p1[i]) + (-p0[i] + p2[i]) * t + (2 * p0[i] - 5 * p1[i] + 4 * p2[i] - p3[i]) * t2 + (-p0[i] + 3 * p1[i] - 3 * p2[i] + p3[i]) * t3));
}

function sampleTunnel(t, spacing = 0.6) {
  const pts = t.points;
  const ext = [pts[0], ...pts, pts[pts.length - 1]];
  const samples = [];
  let length = 0;
  for (let s = 0; s < pts.length - 1; s++) {
    const [a, b] = [pts[s], pts[s + 1]];
    const n = Math.max(2, Math.ceil(Math.hypot(b[0] - a[0], b[1] - a[1]) / spacing));
    for (let i = 0; i < n; i++) {
      const [x, y, z, w, h] = catmull(ext[s], ext[s + 1], ext[s + 2], ext[s + 3], i / n);
      const prev = samples[samples.length - 1];
      if (prev) length += Math.hypot(x - prev.x, y - prev.y);
      samples.push({ x, y, z, w, h });
    }
  }
  const last = pts[pts.length - 1];
  samples.push({ x: last[0], y: last[1], z: last[2], w: last[3], h: last[4] });
  return { id: t.id, from: t.from, to: t.to, tags: t.tags ?? [], samples, length };
}

export function buildNetwork(cfg = config) {
  const tunnels = cfg.tunnels.map((t) => sampleTunnel(t));
  const nodes = new Map(cfg.nodes.map((n) => [n.id, n]));
  const chambers = cfg.nodes.filter((n) => n.type === 'chamber').map((n) => ({
    id: n.id, name: n.name, x: n.position[0], y: n.position[1], z: n.position[2], rx: n.radii[0], ry: n.radii[1], h: n.height, yaw: n.yaw ?? 0,
  }));

  // graph analysis
  const degree = new Map();
  for (const t of cfg.tunnels) for (const id of [t.from, t.to]) degree.set(id, (degree.get(id) ?? 0) + 1);
  // a tunnel that passes straight THROUGH a junction node (without ending there) adds two branches to it
  for (const t of tunnels) {
    for (const n of cfg.nodes) {
      if (n.type === 'entrance' || n.type === 'deadend' || t.from === n.id || t.to === n.id) continue;
      if (t.samples.some((smp) => Math.hypot(smp.x - n.position[0], smp.y - n.position[1]) < 1.5)) degree.set(n.id, (degree.get(n.id) ?? 0) + 2);
    }
  }
  const intersections = cfg.nodes.filter((n) => (n.type === 'junction' || n.type === 'chamber') && (degree.get(n.id) ?? 0) >= 3)
    .map((n) => ({ id: n.id, position: n.position, degree: degree.get(n.id) }));
  const deadEnds = cfg.nodes.filter((n) => n.type === 'deadend' || (degree.get(n.id) === 1 && n.type !== 'entrance')).map((n) => ({ id: n.id, position: n.position }));

  /** Closest tunnel sample / chamber for (x, y): returns { floor, height, q (0 centre .. 1 wall), source }. */
  function locate(x, y) {
    let best = { q: Infinity, floor: 0, height: 3, source: null };
    for (const t of tunnels) {
      for (const s of t.samples) {
        const d = Math.hypot(x - s.x, y - s.y);
        const q = d / (s.w / 2);
        if (q < best.q) best = { q, floor: s.z, height: s.h, source: t.id };
      }
    }
    for (const c of chambers) {
      const dx = x - c.x, dy = y - c.y, co = Math.cos(c.yaw), si = Math.sin(c.yaw);
      const u = (co * dx + si * dy) / c.rx, v = (-si * dx + co * dy) / c.ry;
      const q = Math.hypot(u, v);
      if (q < best.q) best = { q, floor: c.z, height: c.h, source: c.id };
    }
    return best;
  }

  const solids = [];
  for (const p of cfg.pillars) {
    const loc = locate(p.position[0], p.position[1]);
    solids.push({ id: p.id, kind: 'pillar', category: 'pillar', x: p.position[0], y: p.position[1], zFloor: loc.floor, height: loc.height, radius: p.radius, landmark: !!p.landmark });
  }
  for (const b of cfg.boulders) {
    const loc = locate(b.position[0], b.position[1]);
    solids.push({ id: b.id, kind: 'boulder', category: 'boulder', x: b.position[0], y: b.position[1], z: loc.floor + b.radii[2] * 0.55, radii: b.radii, yaw: b.yaw ?? 0, landmark: !!b.landmark });
  }
  for (const s of cfg.stalactites) {
    const loc = locate(s.position[0], s.position[1]);
    solids.push({ id: s.id, kind: 'stalactite', category: 'stalactite', x: s.position[0], y: s.position[1], zCeil: s.position[2] + loc.height * 0.92, radius: s.radius, length: s.length, landmark: !!s.landmark });
  }

  const byId = new Map(solids.map((s) => [s.id, s]));
  const landmarks = cfg.landmarks.map((l) => {
    const s = byId.get(l.ref);
    const z = s ? (s.zFloor ?? (s.z ?? s.zCeil)) : 0;
    return { ...l, position: [s.x, s.y, z] };
  });

  return { cfg, tunnels, nodes, chambers, solids, intersections, deadEnds, landmarks, loops: cfg.loops, locate };
}

/** Axis-aligned bounding box [min, max] (ROS metres) of a solid, for the debug layer / export. */
export function solidBounds(s) {
  if (s.kind === 'pillar') return [[s.x - s.radius, s.y - s.radius, s.zFloor], [s.x + s.radius, s.y + s.radius, s.zFloor + s.height]];
  if (s.kind === 'boulder') { const r = Math.max(s.radii[0], s.radii[1]); return [[s.x - r, s.y - r, s.z - s.radii[2]], [s.x + r, s.y + r, s.z + s.radii[2]]]; }
  return [[s.x - s.radius, s.y - s.radius, s.zCeil - s.length], [s.x + s.radius, s.y + s.radius, s.zCeil]];
}
