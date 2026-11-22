import { obj } from './builder.js';

// Central junction: raised inspection platform, jersey barrier chicane, and the dead-end pump corridor.
export function buildCenterZone() {
  const pump = (name, x, y) => obj('equipment', 'pump', {
    name, material: 'machineBlue', position: [x, y, 0], size: [1.0, 0.8, 0.9],
  });
  return [
    obj('platform', 'platform', { name: 'inspection_platform', material: 'steel', position: [-6, 0, 0], size: [6, 4, 2.0] }),
    obj('platform', 'stairs', { name: 'platform_stairs', material: 'steel', position: [-1.75, 0, 0], size: [2.4, 1.0, 1.85] }),
    // Chicane across route B (gap 1.5 m at x in [3.5, 5.0])
    obj('barrier', 'jersey', { name: 'barrier_c1', material: 'barrier', position: [2.0, -2, 0], size: [3.0, 0.6, 0.8] }),
    obj('barrier', 'jersey', { name: 'barrier_c2', material: 'barrier', position: [6.3, -2, 0], size: [3.0, 0.6, 0.8] }),
    obj('barrier', 'bollard', { name: 'bollard_c1', material: 'machineYellow', position: [3.5, 1.5, 0], size: [0.25, 0.25, 1.0] }),
    obj('storage', 'crate', { name: 'crate_c1', material: 'crate', position: [-8, -14, 0], size: [1.2, 1.2, 1.2], yaw: 0.3 }),
    obj('storage', 'crate', { name: 'crate_c2', material: 'crate', position: [-3.5, -17, 0], size: [1.5, 1.5, 1.0] }),
    // Dead-end pump corridor (x in [-2,2], y in [8,20]); zig-zag leaves >= 1.6 m passage
    pump('pump_1', -1.2, 11), pump('pump_2', 1.2, 14.2), pump('pump_3', -1.2, 17),
    obj('equipment', 'cabinet', { name: 'control_cabinet_end', material: 'darkSteel', position: [0, 19.4, 0], size: [1.6, 0.6, 2.2] }),
    obj('pipe', 'pipe_v', { name: 'riser_dead_1', material: 'pipeRed', position: [-1.75, 19.5, 0], size: [0.25, 0.25, 5.0] }),
    obj('pipe', 'pipe_v', { name: 'riser_dead_2', material: 'pipe', position: [1.75, 19.5, 0], size: [0.25, 0.25, 5.0] }),
    obj('pipe', 'pipe_h', { name: 'manifold_dead', material: 'pipe', position: [0, 19.5, 4.8], size: [3.5, 0.25, 0.25] }),
    // Overhead pipe bridge across the dead-end entrance
    obj('pipe', 'pipe_h', { name: 'bridge_pipe_x', material: 'pipeRed', position: [0, 8.2, 3.6], size: [3.9, 0.3, 0.3] }),
  ];
}
