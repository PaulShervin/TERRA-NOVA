// All values in metres. World origin = centre of the factory floor. +X east/forward, +Y north/left, +Z up.
export const HALL = { xMin: -30, xMax: 30, yMin: -20, yMax: 20 };
export const OUTER_WALL_H = 6.5;
export const INNER_WALL_H = 5.0;
export const OUTER_WALL_T = 0.4;
export const INNER_WALL_T = 0.3;
export const ROBOT = { spawn: [-16, -6, 0], yaw: 0, radius: 0.5, bodySize: [0.7, 0.5, 0.9] };
// Placeholder navigation path (ROS x,y waypoints) following painted lanes; replaced by a real planner later.
export const NAV_PATH = [[-16, -6], [4.15, -6], [4.15, 6], [-6, 6], [-6, 10], [-20, 10], [-20, -6]];
