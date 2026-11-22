// Differential-drive kinematics following a waypoint route (patrol loop or planner-generated path).
// Visual stand-in: the real robot will be driven by ROS 2 /cmd_vel in Gazebo.
import { ROBOT_SCALE } from './Robot.js';

const WHEEL_RADIUS = 0.12 * ROBOT_SCALE;
const HALF_TRACK = 0.29 * ROBOT_SCALE;
const MAX_ANGULAR = 1.8; // rad/s
const WAYPOINT_TOL = 0.12; // m, intermediate points
const GOAL_TOL = 0.15; // m, final point of a non-looping route

const wrapAngle = (a) => Math.atan2(Math.sin(a), Math.cos(a));

export function createRobotController(robot, initialRoute, spawn) {
  let x = spawn.spawn[0], y = spawn.spawn[1], yaw = spawn.yaw;
  let route = initialRoute;
  let loop = true;
  let target = route.length > 1 ? 1 : 0;
  let speed = 1.2;
  let enabled = true;
  let done = route.length < 2;
  let lidarAngle = 0;
  const pose = { x, y, yaw, v: 0, w: 0 };

  const apply = () => robot.setPose([x, y, 0], yaw);

  function tick(step) {
    lidarAngle += step * 10 * Math.PI; // 5 Hz head spin
    robot.lidarHead.rotation.y = lidarAngle;
    pose.v = pose.w = 0;
    if (!enabled || done || route.length === 0) return;

    const [tx, ty] = route[target];
    const dx = tx - x, dy = ty - y;
    const last = !loop && target === route.length - 1;
    if (Math.hypot(dx, dy) < (last ? GOAL_TOL : WAYPOINT_TOL)) {
      if (last) { done = true; return; }
      target = (target + 1) % route.length;
      return;
    }
    const err = wrapAngle(Math.atan2(dy, dx) - yaw);
    const w = Math.max(-MAX_ANGULAR, Math.min(MAX_ANGULAR, 2.8 * err));
    // Slow down while turning; rotate in place when badly misaligned.
    const v = speed * Math.max(0, Math.cos(Math.min(Math.abs(err), Math.PI / 2))) ** 2;
    yaw = wrapAngle(yaw + w * step);
    x += Math.cos(yaw) * v * step;
    y += Math.sin(yaw) * v * step;
    Object.assign(pose, { x, y, yaw, v, w });

    // Wheel spin from differential-drive kinematics (left/right wheel linear speeds).
    const vl = v - w * HALF_TRACK, vr = v + w * HALF_TRACK;
    for (const wheel of robot.wheels) {
      wheel.rotateY(-((wheel.userData.side === 'left' ? vl : vr) / WHEEL_RADIUS) * step);
    }
  }

  /** Integrate in <=20 ms sub-steps so speed stays real-time even at low frame rates (capped at 0.5 s per frame). */
  function update(dt) {
    let remaining = Math.min(dt, 0.5);
    while (remaining > 1e-6) {
      const h = Math.min(0.02, remaining);
      tick(h);
      remaining -= h;
    }
    apply();
  }

  return {
    update,
    pose,
    setEnabled: (on) => { enabled = on; },
    setSpeed: (v) => { speed = v; },
    /** Replace the route. Planner paths are one-shot (loop=false); the patrol route loops. */
    setRoute(points, { loop: lp = false } = {}) {
      route = points; loop = lp; done = points.length < 2 && !lp;
      target = points.length > 1 ? 1 : 0;
    },
    isDone: () => done,
    getTarget: () => (done || route.length === 0 ? null : route[target]),
    getRoute: () => route,
    teleport(nx, ny, nyaw) { x = nx; y = ny; yaw = nyaw; Object.assign(pose, { x, y, yaw }); done = true; apply(); },
  };
}
