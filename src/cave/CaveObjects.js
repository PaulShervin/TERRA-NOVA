// Visual-only cave dressing: scattered rubble (instanced, < 18 cm so it never matters to the robot or the LiDAR world).
// Large formations (pillars, boulders, stalactites) are NOT here: they are part of the signed-distance field itself.
import * as THREE from 'three';
import { rosToThree } from '../core/coords.js';

function rockGeometry() {
  const g = new THREE.IcosahedronGeometry(0.5, 1);
  const pos = g.attributes.position;
  for (let i = 0; i < pos.count; i++) {
    const x = pos.getX(i), y = pos.getY(i), z = pos.getZ(i);
    const s = Math.sin(x * 91.7 + y * 53.3 + z * 17.9) * 43758.5;
    const k = 0.78 + 0.4 * (s - Math.floor(s));
    pos.setXYZ(i, x * k, y * k * 0.7, z * k);
  }
  g.computeVertexNormals();
  return g;
}

export function createCaveObjects(world, count = 900) {
  const geo = rockGeometry();
  const mat = new THREE.MeshStandardMaterial({ color: '#6b6258', roughness: 1, flatShading: true });
  const mesh = new THREE.InstancedMesh(geo, mat, count);
  mesh.name = 'rubble';
  let seed = 4242;
  const rnd = () => ((seed = (seed * 16807) % 2147483647) / 2147483647);
  const f = world.floor;
  const m = new THREE.Matrix4(), q = new THREE.Quaternion(), s = new THREE.Vector3(), p = new THREE.Vector3();
  let placed = 0;
  for (let tries = 0; tries < count * 20 && placed < count; tries++) {
    const ci = 1 + Math.floor(rnd() * (f.nx - 2)), cj = 1 + Math.floor(rnd() * (f.ny - 2));
    const k = cj * f.nx + ci;
    if (Number.isNaN(f.hf[k])) continue;
    const near = f.hf[k + 1] !== undefined && (Number.isNaN(f.hf[k + 2]) || Number.isNaN(f.hf[k - 2]) || Number.isNaN(f.hf[k + 2 * f.nx]) || Number.isNaN(f.hf[k - 2 * f.nx]));
    if (!near && rnd() > 0.15) continue; // mostly along the walls
    const x = world.field.ox + ci * f.voxel + (rnd() - 0.5) * f.voxel, y = world.field.oy + cj * f.voxel + (rnd() - 0.5) * f.voxel;
    const size = 0.05 + rnd() * rnd() * 0.16;
    p.copy(rosToThree([x, y, f.hf[k] + size * 0.2]));
    q.setFromEuler(new THREE.Euler(rnd() * 3, rnd() * 6.28, rnd() * 3));
    s.set(size * (1 + rnd()), size, size * (1 + rnd()));
    m.compose(p, q, s);
    mesh.setMatrixAt(placed++, m);
  }
  mesh.count = placed;
  mesh.instanceMatrix.needsUpdate = true;
  mesh.castShadow = false; mesh.receiveShadow = true;
  return { mesh, count: placed, dispose() { geo.dispose(); mat.dispose(); mesh.dispose(); } };
}
