import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { rosToThree } from './core/coords.js';

// Named viewpoints expressed in ROS axes (x, y, z) so they stay consistent with the layout JSON.
const VIEWS = {
  isometric: { pos: [-10, -70, 46], target: [0, 0, 0], fov: 38 },
  top: { pos: [0, -0.5, 95], target: [0, 0, 0], fov: 45 },
  robot: { pos: [-22, -8.5, 3.0], target: [-14, -6, 0.6], fov: 60 },
};

export function createCameraControls(domElement, aspect) {
  const camera = new THREE.PerspectiveCamera(45, aspect, 0.1, 400);
  camera.up.set(0, 1, 0);
  const controls = new OrbitControls(camera, domElement);
  controls.enableDamping = true; // smooth zoom/orbit
  controls.dampingFactor = 0.08;
  controls.screenSpacePanning = true;
  controls.minDistance = 2;
  controls.maxDistance = 160;
  controls.maxPolarAngle = Math.PI * 0.499 + 0.05;

  const goTo = (name) => {
    const v = VIEWS[name] ?? VIEWS.isometric;
    camera.fov = v.fov;
    camera.updateProjectionMatrix();
    camera.position.copy(rosToThree(v.pos));
    controls.target.copy(rosToThree(v.target));
    controls.update();
  };
  goTo('isometric');

  return {
    camera, controls,
    reset: () => goTo('isometric'),
    setView: goTo,
    resize: (w, h) => {
      camera.aspect = w / h;
      // Shift the scene left so the hall stays clear of the right-hand inset column on wide screens.
      const shift = w > 1100 ? 120 : 0;
      camera.setViewOffset(w, h, shift, 0, w, h);
      camera.updateProjectionMatrix();
    },
    update: () => controls.update(),
    dispose: () => controls.dispose(),
  };
}
