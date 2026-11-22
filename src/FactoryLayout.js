// Builds the static factory (floor, walls, pillars, machinery, storage...) from the layout JSON.
import { buildIndustrialObjects } from './models/IndustrialObjects.js';

export function buildFactory(layout, materials) {
  const { root, dispose, instancedBatches } = buildIndustrialObjects(layout.objects, materials);
  root.name = 'factory';
  return { root, dispose, instancedBatches };
}
