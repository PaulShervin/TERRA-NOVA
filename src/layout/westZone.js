import { obj } from './builder.js';

const CONT = [6.0, 2.4, 2.6];
const container = (name, x, y, z, yaw, material) =>
  obj('container', 'container', { name, material, position: [x, y, z], size: CONT, yaw });
const crate = (name, x, y, z, yaw = 0, s = [1.2, 1.2, 1.0]) =>
  obj('storage', 'crate', { name, material: 'crate', position: [x, y, z], size: s, yaw });
const drum = (name, x, y) =>
  obj('storage', 'drum', { name, material: 'drum', position: [x, y, 0], size: [0.6, 0.6, 0.9] });
const rock = (name, x, y, s, yaw) =>
  obj('obstacle', 'rock', { name, material: 'rubble', position: [x, y, 0], size: s, yaw });

// West hall (open, pillared) + north-west storage room (racks + rubble).
export function buildWestZone() {
  return [
    // Two container rows forming a 1.2 m narrow passage (robot width ~0.7 m).
    container('container_a1', -26, -3, 0, 0, 'contGreen'),
    container('container_a2', -26, 0.6, 0, 0, 'contRust'),
    container('container_a3', -26, -3, 2.6, 0, 'contBlue'),
    container('container_b1', -13, -14, 0, Math.PI / 2, 'contRust'),
    container('container_c1', -22, -16.5, 0, 0, 'contGreen'),
    container('container_c2', -22, -16.5, 2.6, 0, 'contRust'),
    // Material stacks
    crate('crate_1', -25, 5.2, 0), crate('crate_2', -23.7, 5.2, 0), crate('crate_3', -25, 5.2, 1.0, 0.2),
    crate('crate_4', -12, 4, 0, 0.4, [1.0, 1.0, 0.8]), crate('crate_5', -12, 2.9, 0, 0, [1.0, 1.0, 1.6]),
    drum('drum_1', -12.3, -1.8), drum('drum_2', -12.9, -1.4), drum('drum_3', -12.1, -0.9), drum('drum_4', -12.8, -0.6),
    // NW room: shelving racks and a rubble pile partially blocking the north route
    obj('rack', 'rack', { name: 'rack_1', position: [-28.3, 14, 0], size: [3.2, 1.0, 3.0], yaw: Math.PI / 2, material: 'rackOrange' }),
    obj('rack', 'rack', { name: 'rack_2', position: [-28.3, 17.5, 0], size: [3.2, 1.0, 3.0], yaw: Math.PI / 2, material: 'rackOrange' }),
    obj('rack', 'rack', { name: 'rack_3', position: [-14, 19.2, 0], size: [4.0, 1.0, 3.0], yaw: 0, material: 'rackOrange' }),
    rock('rubble_1', -19, 13.5, [2.2, 1.8, 1.2], 0.3), rock('rubble_2', -17.4, 14.7, [1.4, 1.3, 0.9], 1.1),
    rock('rubble_3', -20.4, 14.6, [1.2, 1.0, 0.7], 2.0), rock('rubble_4', -18.6, 15.3, [0.9, 0.9, 0.6], 0.7),
    rock('rubble_5', -24, -9, [0.9, 0.8, 0.55], 0.2), rock('rubble_6', -23.2, -9.6, [0.7, 0.7, 0.45], 1.4),
    // Z4 storage extras: red container + blue racks + pallet stacks
    container('container_z4', -24, 18.6, 0, 0, 'contRed'),
    obj('rack', 'rack', { name: 'rack_4', position: [-14, 16.4, 0], size: [4.0, 1.0, 3.0], material: 'rackBlue' }),
    crate('pallet_1', -22, 12.4, 0), crate('pallet_2', -22, 12.4, 1.0, 0.3, [1.0, 1.0, 0.9]), crate('pallet_3', -13, 12.8, 0, 0.2),
    // Z1 maintenance: workbench, tool cabinets, shelving
    obj('equipment', 'workbench', { name: 'workbench_1', material: 'wood', position: [-28.6, -10.5, 0], size: [3.0, 0.9, 1.0], yaw: Math.PI / 2 }),
    obj('equipment', 'cabinet', { name: 'toolbox_1', material: 'toolRed', position: [-28.8, -14, 0], size: [0.6, 0.8, 1.2], yaw: Math.PI / 2 }),
    obj('equipment', 'cabinet', { name: 'toolbox_2', material: 'toolRed', position: [-28.8, -15, 0], size: [0.6, 0.8, 1.2], yaw: Math.PI / 2 }),
    obj('rack', 'rack', { name: 'rack_m1', position: [-28.3, -18, 0], size: [3.2, 1.0, 3.0], yaw: Math.PI / 2, material: 'rackBlue' }),
    // Tank + barriers around west hall exit
    obj('equipment', 'tank', { name: 'water_tank', material: 'machineBlue', position: [-16, 4.5, 0], size: [2.2, 2.2, 3.4] }),
    obj('barrier', 'bollard', { name: 'bollard_1', material: 'machineYellow', position: [-11.2, -9.2, 0], size: [0.25, 0.25, 1.0] }),
    obj('barrier', 'bollard', { name: 'bollard_2', material: 'machineYellow', position: [-11.2, -2.8, 0], size: [0.25, 0.25, 1.0] }),
    obj('barrier', 'jersey', { name: 'barrier_w1', material: 'barrier', position: [-17, -2, 0], size: [3.0, 0.6, 0.8], yaw: 0 }),
    obj('barrier', 'jersey', { name: 'barrier_w2', material: 'barrierGrey', position: [-17, -9.5, 0], size: [3.0, 0.6, 0.8], yaw: 0.15 }),
  ];
}
