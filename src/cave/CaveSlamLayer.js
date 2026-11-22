// Optional SLAM-demo debugging layer for the cave. EVERYTHING here is a visualisation helper:
//  - "explored" comes from the simulator's occupancy grid (built with ground-truth pose), not from a SLAM estimate
//  - "loop-closure candidates" are a plain distance heuristic on the driven trajectory, not a SLAM loop closure
import * as THREE from 'three';
import { rosToThree } from '../core/coords.js';
import { rosAxes, disposeObject } from '../core/axes.js';
import { solidBounds } from './CaveTunnelNetwork.js';
import { CATEGORY_COLORS } from '../core/materials.js';
import { SENSOR_FRAMES } from '../Robot.js';
import { FREE } from '../mapping/OccupancyGrid.js';

const T = (x, y, z) => rosToThree([x, y, z]);
const COL = { explored: new THREE.Color('#3ddc84'), unexplored: new THREE.Color('#c0504d') };

function labelSprite(text, color = '#ffffff') {
  const c = document.createElement('canvas');
  c.width = 320; c.height = 64;
  const g = c.getContext('2d');
  g.fillStyle = 'rgba(8,10,14,0.82)'; g.fillRect(0, 0, 320, 64);
  g.strokeStyle = color; g.lineWidth = 3; g.strokeRect(1.5, 1.5, 317, 61);
  g.fillStyle = '#fff'; g.font = '600 24px Segoe UI, sans-serif'; g.textAlign = 'center'; g.textBaseline = 'middle';
  g.fillText(text.length > 26 ? `${text.slice(0, 25)}...` : text, 160, 32);
  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  const s = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, depthTest: false, transparent: true }));
  s.scale.set(6.4, 1.28, 1);
  s.renderOrder = 30;
  return s;
}
const overlayMat = (color, extra = {}) => new THREE.MeshBasicMaterial({ color, depthTest: false, transparent: true, opacity: 0.95, ...extra });

export function createCaveSlamLayer({ world, getGrid, getTrajectory, robot, bodyGeo, baseColors, vertsRos }) {
  const group = new THREE.Group();
  group.name = 'cave_slam_layer';
  const parts = {};
  const net = world.network;
  const spawn = world.config.robot.spawn;

  // ---- tunnel network graph (colour = explored / unexplored) ----
  const segPos = [], segMeta = [];
  for (const t of net.tunnels) {
    for (let i = 0; i < t.samples.length - 1; i++) {
      const a = t.samples[i], b = t.samples[i + 1];
      segPos.push(...T(a.x, a.y, a.z + 0.2).toArray(), ...T(b.x, b.y, b.z + 0.2).toArray());
      segMeta.push([a.x, a.y]);
    }
  }
  const netGeo = new THREE.BufferGeometry();
  netGeo.setAttribute('position', new THREE.Float32BufferAttribute(segPos, 3));
  netGeo.setAttribute('color', new THREE.Float32BufferAttribute(new Float32Array(segPos.length).fill(0.7), 3));
  const netLines = new THREE.LineSegments(netGeo, new THREE.LineBasicMaterial({ vertexColors: true, depthTest: false, transparent: true, opacity: 0.95 }));
  netLines.renderOrder = 12; netLines.frustumCulled = false;
  const nodeGroup = new THREE.Group();
  const sphere = new THREE.SphereGeometry(0.55, 16, 12), ring = new THREE.TorusGeometry(1.1, 0.12, 8, 28).rotateX(Math.PI / 2), oct = new THREE.OctahedronGeometry(0.7);
  for (const n of net.cfg.nodes) {
    const [x, y, z] = n.position;
    const loc = net.locate(x, y);
    const p = T(x, y, loc.floor + 0.6);
    let mesh, label = null;
    if (n.type === 'chamber') { mesh = new THREE.Mesh(sphere, overlayMat('#4aa3ff')); mesh.scale.setScalar(1.6); label = n.name; }
    else if (n.type === 'deadend') { mesh = new THREE.Mesh(oct, overlayMat('#ff4d4d')); label = 'dead end'; }
    else if (n.type === 'entrance') { mesh = new THREE.Mesh(sphere, overlayMat('#39ff88')); label = 'entrance'; }
    else { mesh = new THREE.Mesh(ring, overlayMat('#ffd60a')); label = 'intersection'; }
    mesh.position.copy(p); mesh.renderOrder = 13;
    nodeGroup.add(mesh);
    if (label) { const s = labelSprite(label, '#cfd8e8'); s.position.copy(p).add(new THREE.Vector3(0, 2.2, 0)); nodeGroup.add(s); }
  }
  parts.network = new THREE.Group(); parts.network.add(netLines, nodeGroup);

  // ---- landmarks ----
  parts.landmarks = new THREE.Group();
  const lmGeo = new THREE.OctahedronGeometry(0.6);
  for (const lm of net.landmarks) {
    const loc = net.locate(lm.position[0], lm.position[1]);
    const p = T(lm.position[0], lm.position[1], loc.floor + 3.4);
    const m = new THREE.Mesh(lmGeo, overlayMat('#2be3ff')); m.position.copy(p); m.renderOrder = 14;
    const s = labelSprite(lm.name, '#2be3ff'); s.position.copy(p).add(new THREE.Vector3(0, 1.6, 0));
    parts.landmarks.add(m, s);
  }

  // ---- start marker + map-origin axes ----
  parts.start = new THREE.Group();
  const startRing = new THREE.Mesh(new THREE.TorusGeometry(1.3, 0.12, 8, 32).rotateX(Math.PI / 2), overlayMat('#39ff88'));
  startRing.position.copy(T(spawn[0], spawn[1], (world.floor.floorAt(spawn[0], spawn[1]) ?? 0) + 0.1));
  const startLabel = labelSprite('START (robot spawn)', '#39ff88'); startLabel.position.copy(startRing.position).add(new THREE.Vector3(0, 3, 0));
  const axes = rosAxes(4); axes.position.copy(startRing.position).add(new THREE.Vector3(0, 0.3, 0));
  parts.start.add(startRing, startLabel);
  parts.axes = new THREE.Group(); parts.axes.add(axes);

  // ---- trajectory placeholder (older = dim blue, newer = bright yellow) ----
  const TRAIL_MAX = 9000;
  const trailPos = new Float32Array(TRAIL_MAX * 3), trailCol = new Float32Array(TRAIL_MAX * 3);
  const trailGeo = new THREE.BufferGeometry();
  trailGeo.setAttribute('position', new THREE.BufferAttribute(trailPos, 3));
  trailGeo.setAttribute('color', new THREE.BufferAttribute(trailCol, 3));
  trailGeo.setDrawRange(0, 0); trailGeo.boundingSphere = new THREE.Sphere(new THREE.Vector3(), 500);
  const trailLine = new THREE.Line(trailGeo, new THREE.LineBasicMaterial({ vertexColors: true, depthTest: false, transparent: true, linewidth: 2 }));
  trailLine.renderOrder = 15; trailLine.frustumCulled = false;
  parts.trail = new THREE.Group(); parts.trail.add(trailLine);
  const samples = []; // { x, y, s } driven path samples for the loop heuristic
  let pathLen = 0, consumed = 0, lastCheckT = -1, lastNetT = -1, lastTintT = -1;

  // ---- loop-closure candidates (distance heuristic) ----
  parts.loops = new THREE.Group();
  const loopRingGeo = new THREE.TorusGeometry(1.0, 0.14, 8, 28).rotateX(Math.PI / 2);
  const candidates = [];

  // ---- bounding boxes of formations ----
  const edges = new THREE.EdgesGeometry(new THREE.BoxGeometry(1, 1, 1));
  const bp = [], bc = [], c = new THREE.Color();
  for (const s of net.solids) {
    const [lo, hi] = solidBounds(s);
    c.set(CATEGORY_COLORS[s.category] ?? '#ffffff');
    const ctr = T((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2), sz = new THREE.Vector3(hi[0] - lo[0], hi[2] - lo[2], hi[1] - lo[1]);
    const e = edges.attributes.position;
    for (let i = 0; i < e.count; i++) { bp.push(ctr.x + e.getX(i) * sz.x, ctr.y + e.getY(i) * sz.y, ctr.z + e.getZ(i) * sz.z); bc.push(c.r, c.g, c.b); }
  }
  const bGeo = new THREE.BufferGeometry();
  bGeo.setAttribute('position', new THREE.Float32BufferAttribute(bp, 3)); bGeo.setAttribute('color', new THREE.Float32BufferAttribute(bc, 3));
  parts.boxes = new THREE.LineSegments(bGeo, new THREE.LineBasicMaterial({ vertexColors: true, depthTest: false, transparent: true, opacity: 0.8 }));
  parts.boxes.renderOrder = 11;

  // ---- sensor markers (follow the robot) ----
  parts.sensors = new THREE.Group();
  const sMat = { lidar_link: '#4cc9f0', camera_link: '#ff6bd6', imu_link: '#ff8c1a' };
  const sGeo = new THREE.SphereGeometry(0.075, 12, 8);
  const sensorOffsets = { ...SENSOR_FRAMES, ...world.config.robot.sensors };
  for (const k of Object.keys(sMat)) {
    const m = new THREE.Mesh(sGeo, overlayMat(sMat[k])); m.position.copy(rosToThree(sensorOffsets[k])); m.renderOrder = 16; parts.sensors.add(m);
  }
  const robotRing = new THREE.Mesh(new THREE.RingGeometry(0.7, 0.82, 40).rotateX(-Math.PI / 2), overlayMat('#ffd60a', { side: THREE.DoubleSide }));
  robotRing.position.y = 0.06; robotRing.renderOrder = 16;
  parts.sensors.add(robotRing);

  Object.values(parts).forEach((p) => { p.visible = false; group.add(p); });

  // ---- explored / unexplored rock tint ----
  let tintOn = false;
  const colorAttr = bodyGeo.getAttribute('color');
  function applyTint(on) {
    if (!on) { colorAttr.array.set(baseColors); colorAttr.needsUpdate = true; return; }
    const grid = getGrid();
    const arr = colorAttr.array;
    for (let v = 0; v < vertsRos.length / 3; v++) {
      const [i, j] = grid.toCell(vertsRos[v * 3], vertsRos[v * 3 + 1]);
      const known = grid.inside(i, j) && grid.seen[grid.idx(i, j)];
      const f = known ? 1.0 : 0.34;
      arr[v * 3] = baseColors[v * 3] * f; arr[v * 3 + 1] = baseColors[v * 3 + 1] * f + (known ? 0.03 : 0); arr[v * 3 + 2] = baseColors[v * 3 + 2] * f + (known ? 0.06 : 0);
    }
    colorAttr.needsUpdate = true;
  }

  function update(simTime, pose, floorZ) {
    const grid = getGrid();
    // robot-follow marker group
    parts.sensors.position.copy(robot.root.position);
    parts.sensors.quaternion.copy(robot.root.quaternion);
    parts.sensors.scale.copy(robot.root.scale);

    const traj = getTrajectory();
    if (traj.length < consumed) { consumed = 0; samples.length = 0; pathLen = 0; } // map was reset
    if (traj.length > consumed) { // append new driven-path samples (recorded in the sim loop, so none are skipped at high speed)
      for (; consumed < traj.length && samples.length < TRAIL_MAX; consumed++) {
        const q = traj[consumed], prev = samples[samples.length - 1];
        if (prev) pathLen += Math.hypot(q.x - prev.x, q.y - prev.y);
        const n = samples.length;
        samples.push({ x: q.x, y: q.y, s: pathLen });
        const p = T(q.x, q.y, (world.floor.floorAt(q.x, q.y) ?? floorZ) + 0.3);
        trailPos[n * 3] = p.x; trailPos[n * 3 + 1] = p.y; trailPos[n * 3 + 2] = p.z;
      }
      const n = samples.length - 1;
      for (let i = 0; i <= n; i++) { // age ramp: dim blue -> bright yellow
        const k = n === 0 ? 1 : i / n, o = i * 3;
        trailCol[o] = 0.15 + 0.85 * k; trailCol[o + 1] = 0.3 + 0.6 * k; trailCol[o + 2] = 0.9 - 0.8 * k;
      }
      trailGeo.setDrawRange(0, n + 1);
      trailGeo.attributes.position.needsUpdate = true; trailGeo.attributes.color.needsUpdate = true;
    }
    if (simTime - lastCheckT >= 1.0 && samples.length > 20) { // loop-closure candidate heuristic
      lastCheckT = simTime;
      const cur = samples[samples.length - 1];
      for (let i = 0; i < samples.length - 1; i++) {
        const o = samples[i];
        if (cur.s - o.s < 30) break; // only places driven at least 30 m of path ago
        if (Math.hypot(cur.x - o.x, cur.y - o.y) < 2.0 && !candidates.some((c2) => Math.hypot(c2.x - cur.x, c2.y - cur.y) < 6)) {
          const m = new THREE.Mesh(loopRingGeo, overlayMat('#ff3df2')); m.position.copy(T(cur.x, cur.y, floorZ + 0.5)); m.renderOrder = 17;
          const s = labelSprite('loop candidate (heuristic)', '#ff3df2'); s.position.copy(m.position).add(new THREE.Vector3(0, 2, 0));
          parts.loops.add(m, s); candidates.push({ x: cur.x, y: cur.y });
          break;
        }
      }
    }
    if (parts.network.visible && simTime - lastNetT >= 1.0) { // colour each tunnel segment by explored state
      lastNetT = simTime;
      const col = netGeo.attributes.color.array;
      for (let s = 0; s < segMeta.length; s++) {
        const [x, y] = segMeta[s];
        const [i, j] = grid.toCell(x, y);
        const c2 = grid.inside(i, j) && grid.stateAt(grid.idx(i, j)) === FREE ? COL.explored : COL.unexplored;
        col[s * 6] = col[s * 6 + 3] = c2.r; col[s * 6 + 1] = col[s * 6 + 4] = c2.g; col[s * 6 + 2] = col[s * 6 + 5] = c2.b;
      }
      netGeo.attributes.color.needsUpdate = true;
    }
    if (tintOn && simTime - lastTintT >= 1.5) { lastTintT = simTime; applyTint(true); }
  }

  return {
    group, parts, update,
    setPart(name, on) { if (parts[name]) parts[name].visible = on; },
    setTint(on) { tintOn = on; lastTintT = -9; applyTint(on); },
    clearTrail() { samples.length = 0; pathLen = 0; consumed = 0; trailGeo.setDrawRange(0, 0); for (const o of [...parts.loops.children]) { parts.loops.remove(o); } candidates.length = 0; },
    dispose() { disposeObject(group); sphere.dispose(); ring.dispose(); oct.dispose(); lmGeo.dispose(); loopRingGeo.dispose(); edges.dispose(); sGeo.dispose(); },
  };
}
