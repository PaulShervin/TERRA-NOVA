// Optional robotics-debug overlay, kept in its own Group so it never touches the main environment.
import * as THREE from 'three';
import { poseMatrix, rosToThree } from './core/coords.js';
import { rosAxes, disposeObject } from './core/axes.js';
import { CATEGORY_COLORS } from './core/materials.js';
import { SENSOR_FRAMES } from './Robot.js';

const SKIP_BOX = new Set(['floor', 'pathway']);
const LABEL_SKIP = new Set(['floor', 'pathway', 'wall']);

function gridLines(bounds, step = 1, majorEvery = 5) {
  const pos = [], col = [];
  const minor = new THREE.Color('#3a4650'), major = new THREE.Color('#7f93a3');
  const push = (a, b, c) => { pos.push(...a, ...b); col.push(c.r, c.g, c.b, c.r, c.g, c.b); };
  for (let x = bounds.xMin; x <= bounds.xMax; x += step) {
    push(rosToThree([x, bounds.yMin, 0.03]).toArray(), rosToThree([x, bounds.yMax, 0.03]).toArray(), x % majorEvery === 0 ? major : minor);
  }
  for (let y = bounds.yMin; y <= bounds.yMax; y += step) {
    push(rosToThree([bounds.xMin, y, 0.03]).toArray(), rosToThree([bounds.xMax, y, 0.03]).toArray(), y % majorEvery === 0 ? major : minor);
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  g.setAttribute('color', new THREE.Float32BufferAttribute(col, 3));
  return new THREE.LineSegments(g, new THREE.LineBasicMaterial({ vertexColors: true, transparent: true, opacity: 0.55 }));
}

/** All object oriented bounding boxes in one LineSegments (one draw call), coloured by category. */
function boundingBoxes(entries) {
  const edges = new THREE.EdgesGeometry(new THREE.BoxGeometry(1, 1, 1).translate(0, 0.5, 0)).attributes.position;
  const pos = [], col = [];
  const v = new THREE.Vector3(), c = new THREE.Color();
  for (const e of entries) {
    if (SKIP_BOX.has(e.category)) continue;
    const m = poseMatrix(e);
    c.set(CATEGORY_COLORS[e.category] ?? '#ffffff');
    for (let i = 0; i < edges.count; i++) {
      v.fromBufferAttribute(edges, i).applyMatrix4(m);
      pos.push(v.x, v.y, v.z);
      col.push(c.r, c.g, c.b);
    }
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  g.setAttribute('color', new THREE.Float32BufferAttribute(col, 3));
  return new THREE.LineSegments(g, new THREE.LineBasicMaterial({ vertexColors: true }));
}

function labelSprite(text, color) {
  const c = document.createElement('canvas');
  c.width = 256; c.height = 64;
  const g = c.getContext('2d');
  g.fillStyle = 'rgba(12,14,16,0.78)'; g.fillRect(0, 0, 256, 64);
  g.strokeStyle = color; g.lineWidth = 3; g.strokeRect(1.5, 1.5, 253, 61);
  g.fillStyle = '#ffffff'; g.font = '600 24px system-ui, sans-serif';
  g.textAlign = 'center'; g.textBaseline = 'middle';
  g.fillText(text.length > 20 ? `${text.slice(0, 19)}...` : text, 128, 32);
  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  const s = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, depthTest: false, transparent: true }));
  s.scale.set(3.2, 0.8, 1);
  s.renderOrder = 20;
  return s;
}

function labels(entries) {
  const g = new THREE.Group();
  for (const e of entries) {
    if (LABEL_SKIP.has(e.category) || e.name.startsWith('roof_beam')) continue;
    const s = labelSprite(`${e.category}: ${e.name}`, CATEGORY_COLORS[e.category] ?? '#fff');
    s.position.copy(rosToThree([e.position[0], e.position[1], e.position[2] + e.size[2] + 0.7]));
    g.add(s);
  }
  g.name = 'labels';
  return g;
}

function pathLine(waypoints) {
  const pts = waypoints.map(([x, y]) => rosToThree([x, y, 0.08]));
  const geo = new THREE.BufferGeometry().setFromPoints(pts);
  const line = new THREE.Line(geo, new THREE.LineDashedMaterial({ color: '#39ff88', dashSize: 0.5, gapSize: 0.3 }));
  line.computeLineDistances();
  const dots = new THREE.Group();
  const dotGeo = new THREE.SphereGeometry(0.15, 12, 8);
  const dotMat = new THREE.MeshBasicMaterial({ color: '#39ff88' });
  pts.forEach((p) => { const d = new THREE.Mesh(dotGeo, dotMat); d.position.copy(p); dots.add(d); });
  const g = new THREE.Group();
  g.add(line, dots);
  g.name = 'nav_path_placeholder';
  return g;
}

function robotMarkers() {
  const g = new THREE.Group();
  g.name = 'robot_markers';
  const ring = new THREE.Mesh(new THREE.RingGeometry(0.55, 0.65, 40).rotateX(-Math.PI / 2), new THREE.MeshBasicMaterial({ color: '#ffd60a', side: THREE.DoubleSide, depthTest: false }));
  ring.position.y = 0.05;
  ring.renderOrder = 15;
  g.add(ring);
  const sphere = new THREE.SphereGeometry(0.07, 12, 8);
  for (const [key, color] of [['lidar_link', '#4cc9f0'], ['camera_link', '#ff6bd6']]) {
    const m = new THREE.Mesh(sphere, new THREE.MeshBasicMaterial({ color, depthTest: false }));
    m.position.copy(rosToThree(SENSOR_FRAMES[key]));
    m.renderOrder = 15;
    g.add(m);
  }
  return g;
}

export function createDebugLayer(layout) {
  const root = new THREE.Group();
  root.name = 'debug_layer';
  const parts = {
    axes: rosAxes(4, 0.12),
    grid: gridLines(layout.meta.bounds),
    boxes: boundingBoxes(layout.objects),
    labels: labels(layout.objects),
    robot: robotMarkers(),
    path: pathLine([[0, 0], [0, 0]]),
  };
  Object.values(parts).forEach((p) => { p.visible = false; root.add(p); });
  parts.path.visible = true; // planned path is shown by default
  root.visible = true; // sub-parts individually toggled

  return {
    root,
    parts,
    setPart: (name, on) => { if (parts[name]) parts[name].visible = on; },
    /** Keep robot/sensor markers glued to the robot pose. */
    /** Replace the navigation path (ROS x,y waypoints), e.g. with the explorer's current plan. */
    setPath: (waypoints) => {
      const old = parts.path;
      const next = pathLine(waypoints.length > 1 ? waypoints : [[0, 0], [0, 0]]);
      next.visible = old.visible;
      root.remove(old); disposeObject(old);
      root.add(next); parts.path = next;
    },
    followRobot: (robotRoot) => {
      parts.robot.position.copy(robotRoot.position);
      parts.robot.rotation.copy(robotRoot.rotation);
      parts.robot.scale.copy(robotRoot.scale);
    },
    dispose: () => {
      disposeObject(root);
    },
  };
}
