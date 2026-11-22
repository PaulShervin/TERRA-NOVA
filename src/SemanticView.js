import * as THREE from 'three';
import { CATEGORY_COLORS } from './core/materials.js';

/** Swaps every categorised mesh to a flat category colour (segmentation ground-truth preview). */
export function createSemanticView(root) {
  const flat = new Map();
  const flatFor = (category) => {
    if (!flat.has(category)) {
      flat.set(category, new THREE.MeshBasicMaterial({ color: CATEGORY_COLORS[category] ?? '#202020' }));
    }
    return flat.get(category);
  };
  let enabled = false;

  const set = (on) => {
    if (on === enabled) return;
    enabled = on;
    root.traverse((o) => {
      if (!o.isMesh) return;
      if (on) {
        o.userData.origMaterial = o.material;
        o.material = flatFor(o.userData.category);
      } else if (o.userData.origMaterial) {
        o.material = o.userData.origMaterial;
        delete o.userData.origMaterial;
      }
    });
  };
  return { set, isEnabled: () => enabled, dispose: () => flat.forEach((m) => m.dispose()) };
}
