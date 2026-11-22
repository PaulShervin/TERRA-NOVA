// Writes data/cave.json: structured cave description for the future Gazebo / ROS 2 world.
import { mkdirSync, writeFileSync } from 'node:fs';
import { buildCaveWorld, exportCaveJSON } from '../src/cave/CaveGeometry.js';

mkdirSync(new URL('../data/', import.meta.url), { recursive: true });
const out = exportCaveJSON(buildCaveWorld());
writeFileSync(new URL('../data/cave.json', import.meta.url), `${JSON.stringify(out, null, 2)}\n`);
console.log(`wrote data/cave.json (${out.tunnels.length} tunnels, ${out.formations.length} formations, ${out.landmarks.length} landmarks)`);
