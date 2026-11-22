import * as THREE from 'three';
import { rosToThree } from './core/coords.js';

/**
 * Industrial lighting: dim sky/ground hemisphere, one shadow-casting "skylight" directional light
 * (covers the open-roof hall), and warm ceiling lamps (instanced fixtures + cheap shadowless point lights).
 */
export function createLighting(lampPositions, materials, bounds) {
  const group = new THREE.Group();
  group.name = 'lighting';

  group.add(new THREE.HemisphereLight('#aab6c2', '#2b2724', 0.9));

  const sun = new THREE.DirectionalLight('#dfe8f2', 2.4);
  sun.position.set(-18, 42, 26);
  sun.castShadow = true;
  sun.shadow.mapSize.set(4096, 4096);
  const halfX = (bounds.xMax - bounds.xMin) / 2 + 8;
  const halfZ = (bounds.yMax - bounds.yMin) / 2 + 8;
  const cam = sun.shadow.camera;
  Object.assign(cam, { left: -halfX, right: halfX, top: halfZ, bottom: -halfZ, near: 1, far: 120 });
  sun.shadow.bias = -0.0004;
  sun.shadow.normalBias = 0.04;
  group.add(sun);
  group.add(sun.target);

  const fixtureGeo = new THREE.BoxGeometry(1.4, 0.15, 0.5);
  const fixtures = new THREE.InstancedMesh(fixtureGeo, materials.lamp, lampPositions.length);
  fixtures.name = 'lamp_fixtures';
  fixtures.frustumCulled = false;
  const m = new THREE.Matrix4();
  lampPositions.forEach((p, i) => {
    const t = rosToThree(p);
    m.makeTranslation(t.x, t.y, t.z);
    fixtures.setMatrixAt(i, m);
    const light = new THREE.PointLight('#ffd9a0', 90, 22, 2);
    light.position.set(t.x, t.y - 0.4, t.z);
    group.add(light);
  });
  group.add(fixtures);

  const dispose = () => {
    fixtureGeo.dispose();
    fixtures.dispose();
    sun.shadow.map?.dispose();
  };
  return { group, sun, dispose };
}
