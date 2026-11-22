# Algorithms used in this project (and how to do them for real)

Open the in-app version with the **Algorithms** button – every tab draws the *live* state of the running pipeline.
This file is the written reference.

## Pipeline (runs 5×/s of simulation time)

```
LiDAR sweep ─► 3D point map ─┐
      │                      ├─► frontier detection ─► Dijkstra plan ─► waypoint controller ─► motion
      └──────► occupancy grid ┘            ▲                                                    │
                  ▲                         └──────────────── new pose, new sweep ◄─────────────┘
```

> **Honesty note.** In this Three.js prototype the robot pose is *ground truth from the simulator*. A real GPS-denied
> robot must estimate its pose with SLAM. Sensing, mapping, exploration and planning below are independent of where the pose
> comes from; only the pose source changes in the real system.

## 1. Simulated 3D LiDAR – ray casting (`src/sensing/`)

* Geometry: 16 beams, elevation −15…+15° (2° step), azimuth step 1.5° → 16 × 240 = 3,840 rays/sweep, range 0.3–25 m, 5 sweeps/s.
* Per ray: `d = (cos az·cos el, sin az·cos el, sin el)`, find the nearest hit `t` of `o + t·d`, then `range = t + N(0, 0.02²)`.
* World = oriented boxes + vertical cylinders + floor/ceiling planes built from the layout JSON (`Colliders.js`).
  Broad phase: ray vs world AABB. Narrow phase: slab test in the box's local frame (rotate ray by −yaw); quadratic for cylinders.
* Every hit also stores the object **category** (for semantic colouring / later segmentation evaluation).
* Cost ≈ 12 ms / sweep in JS. Visual effect (sweep reveal, beam lines, visibility fan) is in `src/LidarViz.js`.

## 2. Occupancy grid – log-odds (`src/mapping/OccupancyGrid.js`)

* 0.2 m cells, `l = log(p/(1−p))`, 0 = unknown.
* Update per sweep: `+1.0` where a ray ends (only if the hit is 0.1–1.5 m high), `−0.25` for cells a ray passes through
  (carved every 0.1 m along the horizontal projection, stopping 1.5 cells before an obstacle).
* Each cell is updated at most once per sweep; `l` is clamped to [−2, +5]. Occupied if `l > 0.85`, free if `l < −0.35`.
* Why a height band: overhead pipes and the platform deck must not block the robot.

## 3. 3D point-cloud map (`src/mapping/MapCloud.js`)

Hits go into a hash set of 12 cm voxels (de-duplication) and an append-only GPU buffer (≤ 900k points). Colour = height ramp or
semantic class. This is a *map accumulated with a known pose*, not a SLAM result.

## 4. Frontier exploration – Yamauchi 1997 (`src/mapping/Explorer.js`, `Planner.js`)

1. Frontier cell = free ∧ robot fits (outside the inflated obstacles) ∧ has an unknown 4-neighbour.
2. Cluster frontier cells (8-connected flood fill); drop clusters smaller than 5 cells.
3. For each cluster take its cheapest-to-reach cell. `score = pathCost − 0.12·min(size,40) − 3·[near previous goal]`.
4. Pick the lowest score; skip blacklisted goals and goals closer than 0.6 m.
5. No frontier left ⇒ drive to the start pose ⇒ state `complete`.
6. Stuck detection: < 0.25 m of progress in 4 s ⇒ blacklist the goal for 60 s.

## 5. Path planning (`Planner.js`)

* Inflate occupied cells by the robot radius (0.5 m, then 0.4, then 0.3 m if no route exists).
* Dijkstra with a binary heap, 8-connected, diagonal cost √2, corner cutting forbidden. Unknown cells are blocked.
* Backtrack parents, then **string-pulling** smoothing (keep a cell only when the straight line to the next one is blocked).
* Replan every 1.5 s of sim time or when the route ends.

## 6. Motion control (`src/RobotController.js`)

```
err = wrap(atan2(ty−y, tx−x) − yaw)
w   = clamp(2.8·err, ±1.8 rad/s)
v   = v_max · cos²(min(|err|, 90°))
x += v·cos(yaw)·dt ; y += v·sin(yaw)·dt ; yaw += w·dt          (20 ms sub-steps)
v_L = v − w·track/2 ; v_R = v + w·track/2                      (wheel spin animation)
```

## Measured behaviour (headless test)

`node scripts/sim-explore.mjs 1500 <seed>` – 1.35× robot, 5 noise seeds: ≈ 94–96 % of the free space mapped in ≈ 5–10 simulated
minutes, **0** samples with the robot centre inside an obstacle and **0** body-contact samples, robot back at its start.
The remaining ≈ 5 % is space the LiDAR cannot reach (fully enclosed pockets).

## How to do it for real (ROS 2 + Gazebo)

| This prototype | Real stack |
|---|---|
| Simulated LiDAR | Gazebo `gpu_lidar` → `sensor_msgs/PointCloud2` |
| Ground-truth pose | LiDAR-inertial odometry (FAST-LIO2 / LIO-SAM) or Cartographer/RTAB-Map 3D, with loop closure |
| Point map | SLAM back-end map, OctoMap or voxblox |
| Occupancy grid | `slam_toolbox` or `pointcloud_to_laserscan` + Nav2 costmap → `nav_msgs/OccupancyGrid` |
| Frontier explorer | `explore_lite` (m-explore-ros2) or your own node |
| Dijkstra + smoothing | Nav2 planner (NavFn / Smac) + smoother |
| Waypoint controller | Nav2 controller (Regulated Pure Pursuit / DWB) → `/cmd_vel` |
| `data/environment.json` / `data/cave.json` | Generate an SDF world (one model per object) |

Suggested order: (1) SDF world from the JSON; (2) spawn robot with LiDAR + IMU + camera; (3) bridge topics with `ros_gz_bridge`;
(4) run LiDAR-inertial odometry and compare with Gazebo ground truth (ATE/RPE); (5) feed Nav2 + `explore_lite`;
(6) add semantic segmentation of the cloud and evaluate it against the per-point categories exported here.

## Cave variant

The cave reuses the same mapping/exploration stack. Differences: the LiDAR sphere-traces the cave's signed-distance field
(`src/cave/CaveField.js`), returns are labelled floor / wall / ceiling / pillar / boulder / stalactite, and the occupancy update measures heights
**relative to the local floor** (floors slope), carving free space only while a ray stays within 1.7 m of the terrain. The explorer commits to a goal
until reached / timed out (60 s), blacklists reached-but-unresolvable frontiers, and heads home when coverage gains drop below 0.3 % in 150 s.

## Pose estimation – shadow SLAM (`src/slam/`, cave)

The simulator still drives the robot with its true pose, but an independent estimator now runs next to it and is scored against the truth.

1. **Simulated wheel odometry**: true increments + 2 % scale error, 3 % turn slip, 0.0008 rad/m yaw bias and random-walk noise. It drifts.
2. **LiDAR odometry** (`ScanMatcher.js`): each sweep (non-floor, non-ceiling returns projected to 2D) is aligned to a submap of the last 3 keyframes with point-to-line ICP
   (Gauss-Newton, Huber weights) *plus an odometry prior*, so long straight tunnels (where the scan cannot observe the along-tunnel direction) fall back on odometry.
3. **Keyframes** every 1.5 m / 0.4 rad, linked by odometry edges.
4. **Loop closure**: keyframes that are ≥ 30 m of path apart but < 7 m apart in the estimate are matched from a grid of start offsets.
   A match is rejected when the fit is poor, when the scan is degenerate (translation information too low, e.g. a corridor), when a second, different pose fits almost as well
   (perceptual aliasing - the look-alike tunnels), or when it asks for a correction > 5 m.
5. **Pose graph** (`PoseGraph.js`): SE(2) Gauss-Newton with matrix-free preconditioned CG straightens the whole trajectory after each accepted closure.

The true pose is used only to create the noisy odometry increments and to compute ATE / RPE. In the app: *Localisation* card (bottom left) and the red (odometry) / green (SLAM) / gold (loop closure) overlays.

## Label-free segmentation (`src/slam/Segmenter.js`)

No training data. Per 0.3 m voxel: PCA of the neighbourhood gives a normal; the normal is oriented toward the nearest sensor position; then
`normal.z > 0.7` → floor, `< -0.5` → ceiling, otherwise wall, and wall voxels whose neighbourhood bulges toward the sensor (convex, ≥ 6 connected voxels) → object.
Simulator categories are used only to score the result (accuracy, IoU). In the app: *Segment current map* button.

## Evaluation (`npm run evaluate`, writes `data/eval/cave_eval.csv` and `_summary.json`)

Cave, 3 seeds × 3 frontier strategies (`score = path cost − w·min(size,40)`; nearest w=0, balanced w=0.12 = default, large w=0.5), mean over seeds:

| strategy | sim time | coverage | distance | ATE odometry | ATE SLAM | final error odom → SLAM | seg. accuracy | mean IoU |
|---|---|---|---|---|---|---|---|---|
| nearest | 400 s | 84.8 % | 447 m | 8.5 m | 1.07 m | 19.6 → 0.94 m | 76.7 % | 57.5 % |
| balanced | 584 s | 92.7 % | 657 m | 6.8 m | 1.42 m | 13.9 → 0.14 m | 79.9 % | 62.0 % |
| large | 577 s | 89.9 % | 648 m | 11.7 m | 0.71 m | 28.4 → 0.72 m | 79.0 % | 60.7 % |

Honest reading: ATE drops 5-16× and the end-of-run error to < 1.5 m; the 10 s relative error (RPE) is only sometimes better (0.27-0.52 m vs 0.37 m) because graph corrections add small jumps.
"nearest" often stops early (coverage ≈ 80 % in 2 of 3 seeds, via the stall finisher) so its times are not comparable. Segmentation: floor/wall/ceiling IoU ≈ 0.7-0.8 but **objects (boulders, pillars, stalactites) ≈ 0.15** - the convexity cue is weak on noisy rock.
Limits: 2D matching only (no roll/pitch estimation, ≤ 28° slopes), the map and explorer still use the true pose, and one simulator world.
