// Shared unit geometries for instancing. All have their base at y=0 and unit extents so that
// instance scale = (sx, sz, sy) (ROS sizes mapped to three axes) reproduces the layout size exactly.
import * as THREE from 'three';
import { mergeGeometries } from 'three/examples/jsm/utils/BufferGeometryUtils.js';

function unitBox() {
  return new THREE.BoxGeometry(1, 1, 1).translate(0, 0.5, 0);
}

function unitCylinder() {
  return new THREE.CylinderGeometry(0.5, 0.5, 1, 28).translate(0, 0.5, 0);
}

/** Horizontal pipe: axis along +x, length 1, diameter 1 (scaled y/z by pipe diameter). */
function unitPipeX() {
  return new THREE.CylinderGeometry(0.5, 0.5, 1, 20).rotateZ(Math.PI / 2).translate(0, 0.5, 0);
}

/**
 * Shipping container: body + corrugation ribs on both long sides + door locking bars on the +x end.
 * Built in unit space for a 6.0 x 2.4 x 2.6 m container, so rib sizes below are fractions of those extents.
 */
function unitContainer() {
  const parts = [new THREE.BoxGeometry(1, 1, 1).translate(0, 0.5, 0)];
  const ribs = 30;
  for (let i = 0; i < ribs; i++) {
    const x = -0.5 + (i + 0.5) / ribs;
    for (const side of [-1, 1]) {
      parts.push(new THREE.BoxGeometry(0.012, 0.9, 0.018).translate(x, 0.5, side * 0.5));
    }
  }
  for (const z of [-0.18, -0.06, 0.06, 0.18]) parts.push(new THREE.BoxGeometry(0.01, 0.92, 0.012).translate(0.5, 0.5, z));
  // top and bottom rails
  for (const y of [0.015, 0.985]) for (const side of [-1, 1]) parts.push(new THREE.BoxGeometry(1.0, 0.03, 0.022).translate(0, y, side * 0.5));
  const merged = mergeGeometries(parts);
  parts.forEach((g) => g.dispose());
  return merged;
}

/** Jersey barrier: tapered cross-section extruded along x. */
function unitJersey() {
  const u = [[-0.5, 0], [0.5, 0], [0.5, 0.2], [0.3, 0.55], [0.22, 1], [-0.22, 1], [-0.3, 0.55], [-0.5, 0.2]];
  const shape = new THREE.Shape(u.map(([a, b]) => new THREE.Vector2(a, b)));
  const g = new THREE.ExtrudeGeometry(shape, { depth: 1, bevelEnabled: false });
  return g.translate(0, 0, -0.5).rotateY(Math.PI / 2);
}

/** Irregular rock: icosahedron with deterministic per-position vertex jitter (shared verts stay welded). */
function unitRock() {
  const g = new THREE.IcosahedronGeometry(0.5, 1);
  const pos = g.attributes.position;
  const hash = (x, y, z) => {
    const s = Math.sin(x * 127.1 + y * 311.7 + z * 74.7) * 43758.5453;
    return s - Math.floor(s);
  };
  for (let i = 0; i < pos.count; i++) {
    const x = pos.getX(i), y = pos.getY(i), z = pos.getZ(i);
    const k = 0.82 + 0.36 * hash(x, y, z);
    pos.setXYZ(i, x * k, y * k, z * k);
  }
  g.translate(0, 0.45, 0);
  g.computeVertexNormals();
  return g;
}

export const GEOMETRY_FACTORY = {
  box: unitBox, floor: unitBox, crate: unitBox, cabinet: unitBox,
  container: unitContainer,
  jersey: unitJersey,
  bollard: unitCylinder, drum: unitCylinder, tank: unitCylinder, pipe_v: unitCylinder,
  pipe_h: unitPipeX,
  rock: unitRock,
};
