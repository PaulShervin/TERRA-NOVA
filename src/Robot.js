// Placeholder differential-drive mobile robot (no motion logic yet). Modelled in robot-local ROS axes.
import * as THREE from 'three';
import { box, cylinder } from './models/parts.js';
import { rosAxes } from './core/axes.js';
import { rosToThree } from './core/coords.js';

// Sensor mounting positions in base_link (metres, ROS axes) - reuse these when writing the URDF/SDF.
export const SENSOR_FRAMES = {
  base_link: [0, 0, 0],
  lidar_link: [0, 0, 0.72],
  camera_link: [0.33, 0, 0.38],
};

/** Visual/physical scale of the robot model (1.0 = 0.7 x 0.5 m base). Sensor offsets below are in the unscaled model frame. */
export const ROBOT_SCALE = 1.35;

export function createRobot(spawn = [0, 0, 0], yaw = 0) {
  const mats = {
    body: new THREE.MeshStandardMaterial({ color: '#e3b51b', roughness: 0.5, metalness: 0.3 }),
    dark: new THREE.MeshStandardMaterial({ color: '#202428', roughness: 0.6, metalness: 0.5 }),
    tire: new THREE.MeshStandardMaterial({ color: '#111315', roughness: 0.95 }),
    accent: new THREE.MeshStandardMaterial({ color: '#f26b21', roughness: 0.5 }),
    lens: new THREE.MeshStandardMaterial({ color: '#0b1d33', roughness: 0.1, metalness: 0.9 }),
    lidar: new THREE.MeshStandardMaterial({ color: '#2b2f33', roughness: 0.35, metalness: 0.7 }),
    lidarBand: new THREE.MeshStandardMaterial({ color: '#4cc9f0', emissive: '#1b6e8a', emissiveIntensity: 0.8 }),
  };

  const root = new THREE.Group();
  root.name = 'robot';
  root.userData = { category: 'robot' };
  root.scale.setScalar(ROBOT_SCALE);

  const parts = [
    box(mats.body, [0.7, 0.5, 0.18], [0, 0, 0.22]), // chassis
    box(mats.accent, [0.72, 0.52, 0.03], [0, 0, 0.32]), // trim
    box(mats.body, [0.5, 0.36, 0.1], [-0.05, 0, 0.4]), // electronics deck
    cylinder(mats.dark, 0.035, 0.28, [0, 0, 0.55], 12), // LiDAR mast
    // 3D LiDAR (spinning-head style): base, body, emissive band, cap
    cylinder(mats.lidar, 0.06, 0.04, [0, 0, SENSOR_FRAMES.lidar_link[2] - 0.065]),
    cylinder(mats.lidar, 0.052, 0.09, [0, 0, SENSOR_FRAMES.lidar_link[2]]),
    cylinder(mats.lidar, 0.05, 0.03, [0, 0, SENSOR_FRAMES.lidar_link[2] + 0.06]),
    // RGB camera at the front with sensor mount bracket
    box(mats.dark, [0.05, 0.14, 0.06], [0.31, 0, 0.35]),
    box(mats.lens, [0.04, 0.1, 0.04], [0.345, 0, 0.38]),
  ];
  // Wheels: 4x r=0.12, mounted at the chassis corners
  const wheelGeo = new THREE.CylinderGeometry(0.12, 0.12, 0.08, 20);
  const spokeGeo = new THREE.BoxGeometry(0.2, 0.09, 0.035);
  const wheels = [];
  for (const [x, y] of [[0.22, 0.29], [0.22, -0.29], [-0.22, 0.29], [-0.22, -0.29]]) {
    const w = new THREE.Mesh(wheelGeo, mats.tire);
    w.rotation.x = Math.PI / 2; // axle along ROS y
    w.position.set(x, 0.12, -y);
    const spoke = new THREE.Mesh(spokeGeo, mats.dark);
    w.add(spoke);
    w.userData.side = y > 0 ? 'left' : 'right';
    wheels.push(w);
    parts.push(w);
  }
  const lidarHead = new THREE.Group();
  lidarHead.position.copy(rosToThree([0, 0, SENSOR_FRAMES.lidar_link[2] + 0.01]));
  lidarHead.add(new THREE.Mesh(new THREE.CylinderGeometry(0.053, 0.053, 0.02, 20), mats.lidarBand));
  const window_ = new THREE.Mesh(new THREE.BoxGeometry(0.02, 0.025, 0.03), mats.lidarBand);
  window_.position.set(0.055, 0, 0);
  lidarHead.add(window_);
  parts.push(lidarHead);
  parts.forEach((p) => { p.traverse((o) => { o.castShadow = true; o.receiveShadow = true; o.userData.category = 'robot'; }); root.add(p); });

  // Coordinate frames (base, lidar, camera)
  const frames = new THREE.Group();
  frames.name = 'robot_frames';
  frames.add(rosAxes(0.5));
  for (const key of ['lidar_link', 'camera_link']) {
    const a = rosAxes(0.2);
    a.position.copy(rosToThree(SENSOR_FRAMES[key]));
    frames.add(a);
  }
  root.add(frames);

  const setPose = ([x, y, z = 0], heading = 0) => {
    root.position.copy(rosToThree([x, y, z]));
    root.rotation.y = heading;
  };
  setPose(spawn, yaw);

  const dispose = () => {
    wheelGeo.dispose(); spokeGeo.dispose();
    root.traverse((o) => { if (o.geometry && o.geometry !== wheelGeo) o.geometry.dispose(); });
    Object.values(mats).forEach((m) => m.dispose());
  };
  return { root, frames, setPose, wheels, lidarHead, dispose };
}
