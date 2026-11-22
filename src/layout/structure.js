import { obj, wallAlongX, wallAlongY, grid } from './builder.js';
import { HALL, OUTER_WALL_H, INNER_WALL_H, OUTER_WALL_T, INNER_WALL_T } from './dimensions.js';

const { xMin, xMax, yMin, yMax } = HALL;

function outerWalls() {
  const t = OUTER_WALL_T, h = OUTER_WALL_H;
  return [
    wallAlongX(yMin - t / 2, xMin - t, xMax + t, h, t, 'outer_south'),
    wallAlongX(yMax + t / 2, xMin - t, xMax + t, h, t, 'outer_north'),
    wallAlongY(xMin - t / 2, yMin, yMax, h, t, 'outer_west'),
    wallAlongY(xMax + t / 2, yMin, yMax, h, t, 'outer_east'),
  ];
}

// Segments between doorways. Gaps are >= 4 m wide so the robot has ample clearance.
export function innerWalls() {
  const t = INNER_WALL_T, h = INNER_WALL_H;
  const ys = (x, pairs, tag) => pairs.map(([a, b], i) => wallAlongY(x, a, b, h, t, `${tag}_${i}`));
  const xs = (y, pairs, tag) => pairs.map(([a, b], i) => wallAlongX(y, a, b, h, t, `${tag}_${i}`));
  return [
    ...ys(-10, [[-20, -8], [-4, 8], [12, 20]], 'west_divider'),
    ...ys(8, [[-20, -12], [-8, 4], [8, 20]], 'east_divider'),
    ...xs(-8, [[8, 18], [22, 30]], 'se_divider'),
    ...xs(8, [[-30, -22], [-18, -10]], 'nw_divider'),
    ...ys(-2, [[8, 20]], 'deadend_west'),
    ...ys(2, [[8, 20]], 'deadend_east'),
    ...xs(-10, [[-10, 0]], 'south_alcove'),
    ...ys(19, [[-20, -14]], 'se_bay_split'),
  ];
}

// Painted floor lanes (category "pathway") from polylines: thin strips for semantic segmentation.
const LANES = [
  [[-28, -6], [4, -6], [4, -10], [26, -10]],
  [[4, -6], [4, 6], [26, 6]],
  [[-20, -6], [-20, 10], [-6, 10], [-6, 6], [4, 6]],
  [[0, 6], [0, 18]],
];
function lanes() {
  const w = 0.2;
  return LANES.flatMap((pts, li) => pts.slice(1).map(([x1, y1], i) => {
    const [x0, y0] = pts[i];
    const horizontal = y0 === y1;
    const len = Math.abs(horizontal ? x1 - x0 : y1 - y0) + w;
    return obj('pathway', 'box', {
      name: `lane_${li}_${i}`, material: 'pathway',
      position: [(x0 + x1) / 2, (y0 + y1) / 2, 0],
      size: horizontal ? [len, w, 0.02] : [w, len, 0.02],
    });
  }));
}

function pillars() {
  const pts = [
    ...grid([-26, -16], [-16, -11, 3]),
    ...grid([-6], [-15, 14]),
    ...grid([14, 24], [0, 12]),
  ];
  return pts.flatMap((xy, i) => [
    obj('pillar', 'box', { name: `column_${i}`, material: 'pillar', position: [xy[0], xy[1], 0], size: [0.6, 0.6, 6.0] }),
    // Yellow/black hazard collar (decor: visual only, shares the column's footprint)
    obj('pillar', 'box', { name: `column_${i}_band`, material: 'hazard', position: [xy[0], xy[1], 0], size: [0.64, 0.64, 1.3], params: { decor: true } }),
  ]);
}

// Steel doors set into inner walls (decor, flush on both faces) for a more lived-in look.
function doors(walls) {
  return walls.filter((w) => Math.max(w.size[0], w.size[1]) >= 9).flatMap((w, i) => {
    const alongY = w.size[1] > w.size[0];
    const len = alongY ? w.size[1] : w.size[0];
    return [-1, 1].map((side) => {
      const off = (side * len) / 4;
      const face = (INNER_WALL_T / 2 + 0.03) * (i % 2 ? 1 : -1);
      return obj('wall', 'box', {
        name: `door_${w.name}_${side > 0 ? 'b' : 'a'}`, material: 'door', params: { decor: true },
        position: alongY ? [w.position[0] + face, w.position[1] + off, 0] : [w.position[0] + off, w.position[1] + face, 0],
        size: alongY ? [0.08, 1.2, 2.2] : [1.2, 0.08, 2.2],
      });
    });
  });
}

// Overhead roof beams: vertical structure variation + ceiling returns for 3D LiDAR.
function beams() {
  return [-24, -16, -8, 0, 8, 16, 24].map((x, i) => obj('pillar', 'box', {
    name: `roof_beam_${i}`, material: 'steel', position: [x, 0, 6.0], size: [0.25, 40, 0.35],
  }));
}

export function buildStructure() {
  const inner = innerWalls();
  return [
    obj('floor', 'floor', { name: 'factory_floor', material: 'floor', position: [0, 0, -0.1], size: [60, 40, 0.1] }),
    ...outerWalls(), ...inner, ...doors(inner), ...lanes(), ...pillars(), ...beams(),
  ];
}
