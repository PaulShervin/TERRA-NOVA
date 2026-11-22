import { obj } from './builder.js';

// Four fence runs around a hazard; the `gapSide` run is omitted as an entrance.
const fenceRect = (tag, cx, cy, w, d, gapSide) => {
  const h = 1.8;
  const runs = [
    ['s', cx, cy - d / 2, w, 0],
    ['n', cx, cy + d / 2, w, 0],
    ['w', cx - w / 2, cy, d, Math.PI / 2],
    ['e', cx + w / 2, cy, d, Math.PI / 2],
  ];
  return runs
    .filter(([n]) => n !== gapSide)
    .map(([n, x, y, len, yaw]) => obj('fence', 'fence', {
      name: `${tag}_${n}`, material: 'fence', position: [x, y, 0], size: [len, 0.06, h], yaw,
    }));
};

// East side: crushing bay (SE-A), power bay (SE-B) and ball-mill / conveyor hall.
export function buildEastZone() {
  return [
    // SE bay A (x 8..19, y -20..-8): primary crusher + motor control centre
    obj('machinery', 'crusher', { name: 'jaw_crusher', material: 'machineBlue', position: [13.5, -16, 0], size: [5.0, 3.5, 4.0] }),
    ...[0, 1, 2, 3].map((i) => obj('equipment', 'cabinet', {
      name: `mcc_${i}`, material: 'darkSteel', position: [9.2 + i * 1.0, -19.5, 0], size: [0.9, 0.6, 2.2],
    })),
    obj('barrier', 'jersey', { name: 'barrier_se1', material: 'barrier', position: [16.5, -11.2, 0], size: [3.0, 0.6, 0.8], yaw: 0.2 }),
    // SE bay B (x 19..30, y -20..-8): storage tanks + generators (fenced)
    obj('equipment', 'tank', { name: 'tank_1', material: 'machineBlue', position: [27.5, -17.5, 0], size: [3.0, 3.0, 5.0] }),
    obj('equipment', 'tank', { name: 'tank_2', material: 'steel', position: [27.5, -12.0, 0], size: [3.0, 3.0, 4.0] }),
    obj('machinery', 'generator', { name: 'generator_1', material: 'machineYellow', position: [22.5, -17.5, 0], size: [3.0, 1.5, 2.0] }),
    obj('machinery', 'generator', { name: 'generator_2', material: 'machineYellow', position: [22.5, -14.5, 0], size: [3.0, 1.5, 2.0] }),
    ...fenceRect('fence_gen', 22.5, -16, 4.6, 5.6, 'n'),
    // East hall: ball mill with fence, conveyors, switchgear
    obj('machinery', 'mill', { name: 'ball_mill', material: 'machineBlue', position: [20, 0, 0], size: [6.0, 3.0, 3.2] }),
    ...fenceRect('fence_mill', 20, 0, 9, 6.5, 'e'),
    obj('machinery', 'conveyor', { name: 'conveyor_flat', material: 'darkSteel', position: [19, 17.2, 0], size: [18.0, 1.0, 1.3] }),
    obj('machinery', 'conveyor', { name: 'conveyor_incline', material: 'darkSteel', position: [27.5, 8.5, 0], size: [8.0, 1.0, 3.5], yaw: Math.PI / 2, params: { incline: true } }),
    obj('equipment', 'cabinet', { name: 'switchgear_1', material: 'darkSteel', position: [29.4, 1.5, 0], size: [0.6, 1.6, 2.2] }),
    obj('barrier', 'jersey', { name: 'barrier_e1', material: 'barrierGrey', position: [12, 10, 0], size: [3.0, 0.6, 0.8], yaw: Math.PI / 2 }),
    obj('barrier', 'jersey', { name: 'barrier_e2', material: 'barrier', position: [12, 3, 0], size: [3.0, 0.6, 0.8], yaw: Math.PI / 2 }),
    ...[0, 1, 2].map((i) => obj('storage', 'crate', {
      name: `ore_bin_${i}`, material: 'darkSteel', position: [10.5, 12.2 + i * 1.7, 0], size: [1.4, 1.4, 1.0], yaw: 0.1 * i,
    })),
    // Wall-mounted pipe runs (overhead, different heights)
    obj('pipe', 'pipe_h', { name: 'pipe_run_n1', material: 'pipe', position: [19.5, 19.55, 4.1], size: [20, 0.4, 0.4] }),
    obj('pipe', 'pipe_h', { name: 'pipe_run_n2', material: 'pipeRed', position: [19.5, 19.55, 4.8], size: [20, 0.3, 0.3] }),
    obj('pipe', 'pipe_v', { name: 'riser_e1', material: 'pipe', position: [9.2, 19.55, 0], size: [0.4, 0.4, 4.3] }),
    obj('pipe', 'pipe_v', { name: 'riser_e2', material: 'pipeRed', position: [29.5, 19.55, 0], size: [0.3, 0.3, 5.0] }),
    // West-wall pipe run: model is X-aligned, so rotate 90 deg to run along Y.
    obj('pipe', 'pipe_h', { name: 'pipe_run_w', material: 'pipe', position: [-29.55, -6, 3.8], size: [26, 0.4, 0.4], yaw: Math.PI / 2 }),
  ];
}
