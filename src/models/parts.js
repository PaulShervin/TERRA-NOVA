// Helpers for building composite models in an object-local ROS frame (x fwd, y left, z up, origin = base centre).
// Three.js axes: x->x, z(up)->y, y(left)->-z. Only this file performs that swizzle for composites.
import * as THREE from 'three';

/** Box with ROS-ordered size [sx,sy,sz], centred at ROS-local [cx,cy,cz]. */
export function box(mat, size, center, rot = [0, 0, 0]) {
  const mesh = new THREE.Mesh(new THREE.BoxGeometry(size[0], size[2], size[1]), mat);
  mesh.position.set(center[0], center[2], -center[1]);
  // rot = [rollAboutX, pitchAboutY(ROS), yawAboutZ] applied in three axes (x, -z, y)
  mesh.rotation.set(rot[0], rot[2], -rot[1], 'YXZ');
  return mesh;
}

/** Vertical cylinder (axis = ROS z), radius r, height h, centred at ROS-local [cx,cy,cz]. */
export function cylinder(mat, r, h, center, segments = 24) {
  const mesh = new THREE.Mesh(new THREE.CylinderGeometry(r, r, h, segments), mat);
  mesh.position.set(center[0], center[2], -center[1]);
  return mesh;
}

/** Cylinder lying along ROS x (axis horizontal), length l, radius r. */
export function cylinderX(mat, r, l, center, segments = 24) {
  const mesh = new THREE.Mesh(new THREE.CylinderGeometry(r, r, l, segments), mat);
  mesh.rotation.z = Math.PI / 2;
  mesh.position.set(center[0], center[2], -center[1]);
  return mesh;
}

/** Group that collects meshes; returned object carries the layout entry in userData. */
export function group(entry, children) {
  const g = new THREE.Group();
  g.name = entry.name;
  children.forEach((c) => g.add(c));
  g.userData = { id: entry.id, category: entry.category, model: entry.model };
  return g;
}
