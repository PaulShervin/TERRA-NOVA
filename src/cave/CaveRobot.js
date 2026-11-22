// Cave version of the placeholder robot: the existing industrial robot model, plus terrain following (pitch/roll from the
// sloping floor), an IMU indicator, a headlamp and a fill light. The industrial Robot.js is imported, not modified.
import * as THREE from 'three';
import { createRobot, SENSOR_FRAMES } from '../Robot.js';
import { rosAxes } from '../core/axes.js';
import { rosToThree } from '../core/coords.js';

export function createCaveRobot(world, spawn, yaw = 0) {
  const base = createRobot([spawn[0], spawn[1], 0], yaw);
  const { root } = base;
  root.rotation.order = 'YZX'; // yaw outermost, then pitch (about local z), then roll (about local x)

  // IMU indicator: small orange block + its own coordinate frame at imu_link
  const imuMat = new THREE.MeshStandardMaterial({ color: '#ff8c1a', emissive: '#7a3d00', roughness: 0.4 });
  const imu = new THREE.Mesh(new THREE.BoxGeometry(0.1, 0.05, 0.08), imuMat);
  imu.position.copy(rosToThree(world.config.robot.sensors.imu_link ?? [0, 0, 0.3]));
  imu.userData.category = 'robot';
  const imuAxes = rosAxes(0.16);
  imuAxes.position.copy(imu.position);
  root.add(imu, imuAxes);

  // Headlamp (spot) next to the RGB camera + a small fill so the robot itself is never a silhouette
  const lamp = new THREE.SpotLight('#fff1d6', 260, 26, 0.62, 0.55, 1.6);
  lamp.position.copy(rosToThree([SENSOR_FRAMES.camera_link[0] + 0.05, 0, SENSOR_FRAMES.camera_link[2] + 0.08]));
  const target = new THREE.Object3D();
  target.position.copy(rosToThree([8, 0, 0.1]));
  root.add(lamp, target);
  lamp.target = target;
  const fill = new THREE.PointLight('#ffe2b8', 6, 7, 2);
  fill.position.copy(rosToThree([-0.1, 0, 1.0]));
  root.add(fill);

  const state = { pitch: 0, roll: 0 };
  /** Called by the motion controller: [x, y, ignored], heading. Height + tilt come from the cave floor. */
  function setPose([x, y], heading = 0) {
    const f = world.floor;
    const z = f.floorAt(x, y) ?? 0;
    const cs = Math.cos(heading), sn = Math.sin(heading);
    const at = (dx, dy) => f.floorAt(x + dx * cs - dy * sn, y + dx * sn + dy * cs) ?? z;
    const pitch = Math.atan2(at(0.5, 0) - at(-0.5, 0), 1.0); // nose-up positive
    const roll = Math.atan2(at(0, 0.4) - at(0, -0.4), 0.8); // left-up positive
    state.pitch += (pitch - state.pitch) * 0.35;
    state.roll += (roll - state.roll) * 0.35;
    root.position.copy(rosToThree([x, y, z + 0.02]));
    root.rotation.set(state.roll, heading, state.pitch);
  }
  setPose(spawn, yaw);

  return {
    ...base, root, setPose, imu, lamp,
    dispose() { base.dispose(); imu.geometry.dispose(); imuMat.dispose(); },
  };
}
