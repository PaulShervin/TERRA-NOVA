import { resetIds } from './builder.js';
import { buildStructure } from './structure.js';
import { buildWestZone } from './westZone.js';
import { buildCenterZone } from './centerZone.js';
import { buildEastZone } from './eastZone.js';
import { HALL, ROBOT, NAV_PATH } from './dimensions.js';

// Named functional zones (label anchors are the point the callout leader line touches).
const ZONES = [
  { id: 'Z1', name: 'Maintenance Zone', bounds: [-30, -20, -10, 8], anchor: [-28.6, -10.5, 1.0] },
  { id: 'Z2', name: 'Processing Zone', bounds: [8, -8, 30, 20], anchor: [20, 0, 2.6] },
  { id: 'Z3', name: 'Utility Zone', bounds: [8, -20, 30, -8], anchor: [27.5, -17.5, 5.0] },
  { id: 'Z4', name: 'Storage Zone', bounds: [-30, 8, -10, 20], anchor: [-24, 18.6, 2.6] },
];

// Ceiling lamp positions [x, y, z]; used for both visible fixtures and point lights.
const LAMPS = [
  [-20, -10], [-20, 0], [-20, 12], [-5, -14], [-5, 6], [0, 14],
  [14, -14], [26, -14], [14, 6], [26, 6], [20, 16], [8, -2],
].map(([x, y]) => [x, y, 5.4]);

/** Full environment description as plain JSON (metres / radians, ROS axes). */
export function buildLayout() {
  resetIds();
  const objects = [
    ...buildStructure(), ...buildWestZone(), ...buildCenterZone(), ...buildEastZone(),
  ];
  return {
    meta: {
      name: 'industrial_mining_facility',
      units: 'metres',
      frame: 'ROS REP-103 (x fwd, y left, z up, yaw CCW)',
      note: 'position = base-centre of object; size = [x,y,z] extents in object local frame. Visual prototype only.',
      bounds: HALL,
    },
    robot: { spawn: ROBOT.spawn, yaw: ROBOT.yaw, radius: ROBOT.radius },
    zones: ZONES,
    navPath: NAV_PATH,
    lights: LAMPS,
    objects,
  };
}
