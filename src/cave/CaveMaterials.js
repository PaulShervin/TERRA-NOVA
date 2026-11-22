// Rock material + baked per-vertex colour (strata banding + cheap ambient occlusion from the signed-distance field).
import * as THREE from 'three';
import { concreteTexture } from '../core/textures.js';
import { applyTriplanar } from '../core/triplanar.js';

export function createRockMaterial() {
  const tex = concreteTexture({ size: 512, base: [118, 106, 96], seed: 11, contrast: 1.55, stains: 1.0, cracks: 26 });
  const mat = new THREE.MeshStandardMaterial({ map: tex, vertexColors: true, roughness: 0.96, metalness: 0.0 });
  applyTriplanar(mat, { tile: 2.6, bump: 0.55, dirt: false, roughnessVar: 0.3 });
  mat.userData.dispose = () => tex.dispose();
  return mat;
}

const clamp01 = (v) => Math.min(1, Math.max(0, v));

/** Returns Float32Array(n*3) of vertex colours for the ROS-space mesh. */
export function computeVertexColors(world) {
  const { positions, normals } = world.mesh;
  const n = positions.length / 3;
  const out = new Float32Array(n * 3);
  const grey = new THREE.Color(0.5, 0.5, 0.5), brown = new THREE.Color(0.66, 0.5, 0.36), c = new THREE.Color();
  for (let i = 0; i < n; i++) {
    const x = positions[i * 3], y = positions[i * 3 + 1], z = positions[i * 3 + 2];
    const nx = normals[i * 3], ny = normals[i * 3 + 1], nz = normals[i * 3 + 2];
    const strata = 0.5 + 0.5 * Math.sin(z * 1.9 + 0.9 * Math.sin(x * 0.27 + y * 0.19) + 0.4 * Math.sin(y * 0.41));
    c.copy(grey).lerp(brown, 0.15 + 0.7 * strata);
    // open space in front of the surface => bright, crevices => dark
    const v = world.field.sample(x + nx * 0.9, y + ny * 0.9, z + nz * 0.9);
    const ao = 0.5 + 0.5 * clamp01(-v / 0.42);
    const floorBoost = nz > 0.6 ? 1.08 : 1.0;
    const k = ao * floorBoost;
    out[i * 3] = c.r * k; out[i * 3 + 1] = c.g * k; out[i * 3 + 2] = c.b * k;
  }
  return out;
}
