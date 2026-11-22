// Writes the environment description to data/environment.json (input for future SDF/Gazebo generation).
import { mkdirSync, writeFileSync } from 'node:fs';
import { buildLayout } from '../src/layout/index.js';

mkdirSync(new URL('../data/', import.meta.url), { recursive: true });
const layout = buildLayout();
writeFileSync(new URL('../data/environment.json', import.meta.url), `${JSON.stringify(layout, null, 2)}\n`);
console.log(`wrote data/environment.json (${layout.objects.length} objects)`);
