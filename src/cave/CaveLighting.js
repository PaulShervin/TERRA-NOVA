// Cave lighting: dim warm hemisphere ambient, sparse industrial cage lamps (point lights + fixtures), and one shadow-casting
// "inspection" light from above that only reaches the interior when the roof is hidden. Robot lights live in CaveRobot.
import * as THREE from 'three';
import { rosToThree } from '../core/coords.js';

export function createCaveLighting(world) {
  const group = new THREE.Group();
  group.name = 'cave_lighting';
  const B = world.bounds;

  group.add(new THREE.HemisphereLight('#8a7f72', '#2c2520', 0.85));

  const sun = new THREE.DirectionalLight('#d9dee6', 1.7);
  const cx = (B.xMin + B.xMax) / 2, cy = (B.yMin + B.yMax) / 2;
  sun.position.copy(rosToThree([cx - 30, cy - 40, 80]));
  sun.target.position.copy(rosToThree([cx, cy, 0]));
  sun.castShadow = true;
  sun.shadow.mapSize.set(4096, 4096);
  const hx = (B.xMax - B.xMin) / 2 + 6, hz = (B.yMax - B.yMin) / 2 + 6;
  Object.assign(sun.shadow.camera, { left: -hx, right: hx, top: hz, bottom: -hz, near: 5, far: 200 });
  sun.shadow.bias = -0.0006;
  sun.shadow.normalBias = 0.06;
  group.add(sun, sun.target);

  // Industrial cage lamps hung from the ceiling at the configured positions
  const fixtureGeo = new THREE.BoxGeometry(0.5, 0.18, 0.3);
  const cageGeo = new THREE.BoxGeometry(0.62, 0.3, 0.42);
  const lampMat = new THREE.MeshStandardMaterial({ color: '#fff0cf', emissive: '#ffcf8a', emissiveIntensity: 3.0 });
  const cageMat = new THREE.MeshBasicMaterial({ color: '#2a2a2a', wireframe: true });
  const lampPositions = [];
  for (const [x, y] of world.config.lamps) {
    const loc = world.network.locate(x, y);
    const z = loc.floor + Math.min(loc.height - 0.7, 3.4);
    lampPositions.push([x, y, z]);
    const t = rosToThree([x, y, z]);
    const fixture = new THREE.Mesh(fixtureGeo, lampMat);
    fixture.position.copy(t);
    const cage = new THREE.Mesh(cageGeo, cageMat);
    cage.position.copy(t);
    const light = new THREE.PointLight('#ffbf73', 38, 16, 1.7);
    light.position.copy(t).add(new THREE.Vector3(0, -0.35, 0));
    group.add(fixture, cage, light);
  }

  const setInspectionLight = (on) => { sun.visible = on; };
  return {
    group, sun, lampPositions, setInspectionLight,
    dispose() { fixtureGeo.dispose(); cageGeo.dispose(); lampMat.dispose(); cageMat.dispose(); sun.shadow.map?.dispose(); },
  };
}
