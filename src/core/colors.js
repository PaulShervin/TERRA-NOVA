import * as THREE from 'three';
import { CATEGORY_COLORS } from './materials.js';

// Blue -> cyan -> green -> yellow -> red height ramp (metres 0..HEIGHT_MAX).
const RAMP = ['#2433ff', '#18b8ff', '#22e07a', '#f2e41a', '#ff3b1f'].map((c) => new THREE.Color(c));
export const HEIGHT_MAX = 6.5;

export function heightColor(z, out = new THREE.Color()) {
  const x = Math.min(Math.max(z / HEIGHT_MAX, 0), 1) * (RAMP.length - 1);
  const i = Math.min(Math.floor(x), RAMP.length - 2);
  return out.copy(RAMP[i]).lerp(RAMP[i + 1], x - i);
}

const catCache = new Map();
export function categoryColor(cat, out = new THREE.Color()) {
  if (!catCache.has(cat)) catCache.set(cat, new THREE.Color(CATEGORY_COLORS[cat] ?? '#9a9a9a'));
  return out.copy(catCache.get(cat));
}
