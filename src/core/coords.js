// World convention (matches ROS REP-103 / Gazebo): +X forward, +Y left, +Z up, metres, yaw CCW about +Z.
// Three.js is Y-up, so conversion happens ONLY here: three = (x, z, -y). Yaw maps 1:1 to rotation about three +Y.
import * as THREE from 'three';

export const rosToThree = ([x, y, z]) => new THREE.Vector3(x, z, -y);
export const threeToRos = (v) => [v.x, -v.z, v.y];

/** Object matrix from a layout pose (base-centre position, yaw) and ROS-ordered size [sx,sy,sz]. */
export function poseMatrix({ position, yaw = 0, size = [1, 1, 1] }) {
  const m = new THREE.Matrix4();
  const q = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), yaw);
  const p = rosToThree(position);
  m.compose(p, q, new THREE.Vector3(size[0], size[2], size[1]));
  return m;
}
