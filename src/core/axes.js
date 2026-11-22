import * as THREE from 'three';

/** REP-103 axis triad: X red (forward), Y green (left), Z blue (up). Drawn in Three.js axes (x, -z, y). */
export function rosAxes(length = 1, headRatio = 0.2) {
  const g = new THREE.Group();
  const defs = [[new THREE.Vector3(1, 0, 0), 0xff3b30], [new THREE.Vector3(0, 0, -1), 0x34c759], [new THREE.Vector3(0, 1, 0), 0x0a84ff]];
  for (const [dir, color] of defs) {
    const arrow = new THREE.ArrowHelper(dir, new THREE.Vector3(), length, color, length * headRatio, length * headRatio * 0.6);
    arrow.line.material.depthTest = false;
    arrow.cone.material.depthTest = false;
    arrow.renderOrder = 10;
    g.add(arrow);
  }
  g.name = 'ros_axes';
  return g;
}

export function disposeObject(root) {
  root.traverse((o) => {
    o.geometry?.dispose?.();
    const mats = Array.isArray(o.material) ? o.material : o.material ? [o.material] : [];
    mats.forEach((m) => m.dispose());
  });
}
