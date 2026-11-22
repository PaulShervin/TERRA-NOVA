// Turns layout entries into Three.js objects. Single-geometry models are batched into InstancedMeshes
// (one draw call per geometry/material/category); composite models become small Groups.
import * as THREE from 'three';
import { poseMatrix, rosToThree } from '../core/coords.js';
import { GEOMETRY_FACTORY } from './geometries.js';
import { crusher, generator, mill, conveyor, pump } from './machinery.js';
import { fence, rack, platform, stairs, workbench } from './structures.js';

const COMPOSITES = { crusher, generator, mill, conveyor, pump, fence, rack, platform, stairs, workbench };

export function buildIndustrialObjects(entries, materials) {
  const root = new THREE.Group();
  root.name = 'industrial_objects';
  const geometries = new Map();
  const batches = new Map();

  const geometryFor = (model) => {
    if (!geometries.has(model)) geometries.set(model, GEOMETRY_FACTORY[model]());
    return geometries.get(model);
  };

  for (const e of entries) {
    if (COMPOSITES[e.model]) {
      const g = COMPOSITES[e.model](e, materials);
      g.position.copy(rosToThree(e.position));
      g.rotation.y = e.yaw;
      g.traverse((o) => { if (o.isMesh) o.userData.category = e.category; });
      root.add(g);
    } else if (GEOMETRY_FACTORY[e.model]) {
      const key = `${e.model}|${e.material}|${e.category}`;
      if (!batches.has(key)) batches.set(key, { model: e.model, material: e.material, category: e.category, items: [] });
      batches.get(key).items.push(e);
    } else {
      throw new Error(`Unknown model "${e.model}" for object ${e.id}`);
    }
  }

  for (const [key, b] of batches) {
    const mat = materials[b.material];
    if (!mat) throw new Error(`Unknown material "${b.material}" (${key})`);
    const mesh = new THREE.InstancedMesh(geometryFor(b.model), mat, b.items.length);
    mesh.name = key;
    mesh.userData = { category: b.category, ids: b.items.map((i) => i.id) };
    b.items.forEach((item, i) => mesh.setMatrixAt(i, poseMatrix(item)));
    mesh.instanceMatrix.needsUpdate = true;
    mesh.computeBoundingSphere();
    mesh.frustumCulled = false; // instances span the whole hall
    root.add(mesh);
  }

  const dispose = () => {
    geometries.forEach((g) => g.dispose());
    root.traverse((o) => { if (o.isMesh && !o.isInstancedMesh) o.geometry.dispose(); if (o.isInstancedMesh) o.dispose(); });
  };
  return { root, dispose, instancedBatches: batches.size };
}
