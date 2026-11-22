# Gazebo (gz-sim) + ROS 2 port

Everything the web simulator does, as a ROS 2 Jazzy / Gazebo Harmonic package (`ros2_ws/src/mining_sim_gz`):

| Web simulator (`src/`) | Gazebo / ROS 2 package |
|---|---|
| industrial facility layout (`layout/`, `sensing/Colliders.js`) | `worlds/industrial.sdf` - the same 152 boxes / cylinders, 12 lamps, ceiling |
| procedural cave (signed-distance field + surface nets) | `worlds/cave.sdf` (full shell) and `worlds/cave_cutaway.sdf` (roof left out of the visual, like the web roof toggle); `models/cave_shell*/meshes/*.obj` |
| robot (scale 1.35, VLP-16-like LiDAR, IMU, camera) | `models/mapping_robot` - diff drive, `gpu_lidar` 16 x 240 rays @ 5 Hz, 2 cm noise, IMU, RGB camera |
| LiDAR ray casting / sphere tracing | Gazebo `gpu_lidar` (renders the same geometry) -> `/lidar/points` |
| simulated wheel odometry (drift) | `odom_noise_node` (`/odom` -> `/odom_noisy`, same error model) |
| `slam/SlamEstimator.js` (ICP + loop closure + pose graph) | `scan_slam_node` + `slam_core.py`, `scan_matcher.py`, `pose_graph.py` |
| `mapping/*`, `RobotController.js` (grid, frontiers, Dijkstra, controller) | `explorer_node` + `explorer_core.py` |
| `slam/Segmenter.js` (label-free segmentation) | `segmentation_node` + `segmenter.py` (service `/segment_map`) |
| `scripts/evaluate.mjs` (ATE / RPE / coverage / IoU) | metrics topic `/slam/metrics`, `slam_metrics.json`, `segmentation.json` |

## Regenerating the assets

The worlds, meshes, robot model and ground-truth masks are generated from the web project's data, so both simulators stay in sync:

```
node scripts/export-gazebo.mjs        # from industrial-sim-3d/
```

## Run (Ubuntu 24.04 with ROS 2 Jazzy + `ros-jazzy-ros-gz`, or the Docker image below)

```
cd gazebo/ros2_ws && colcon build --packages-select mining_sim_gz && source install/setup.bash
ros2 launch mining_sim_gz sim.launch.py world:=cave_cutaway            # GUI + RViz
ros2 launch mining_sim_gz sim.launch.py world:=industrial
ros2 launch mining_sim_gz sim.launch.py world:=cave headless:=true rviz:=false log_dir:=/tmp/run1
ros2 service call /segment_map std_srvs/srv/Trigger "{}"              # label-free segmentation of the map so far
```

Launch arguments: `pose_source:=ground_truth|slam` (the pose the explorer plans with; `ground_truth` mirrors the web "shadow mode",
`slam` closes the loop on the estimate), `explore:=false` (drive yourself with `/cmd_vel`), `seed`, `size_weight` (frontier strategy: 0 nearest, 0.12 default, 0.5 large).

RViz: blue = ground truth, red = odometry only, green = SLAM estimate, gold lines = accepted loop closures, plus the occupancy map,
the LiDAR cloud and the segmentation cloud (floor tan, ceiling violet, wall blue, object orange).

## Docker (no local ROS needed)

```
cd gazebo
docker build -t mining-sim-gz -f docker/Dockerfile .
docker run --rm -v "$PWD/out:/out" mining-sim-gz run_headless.sh cave 600 /out
```

## What has and has not been verified

**Runs in real Gazebo (Harmonic 8.11, ROS 2 Jazzy, Docker, software rendering):**
* all four SDF files pass `gz sdf -k` with 0 errors (that check caught a duplicate-name bug in the robot model);
* `ros2 launch mining_sim_gz sim.launch.py world:=cave headless:=true` starts the world, spawns the robot, bridges 7 topics; the LiDAR publishes about 4.2 Hz;
* the explorer drives the robot autonomously: 51 % coverage / 121 m in a 5-minute run (still exploring when stopped); the segmentation service works (61 % accuracy on that map);
* `slam_metrics.json` and `segmentation.json` are written to `log_dir`.

**Offline tests (`python -m pytest test`, 6 tests):** the Python SLAM replays a recorded web run (ATE 0.57 m vs 2.61 m for odometry only; the JS estimator 1.67 m),
the Python segmenter matches the JS one (80.0 % vs 76.9 %), the mesh-based ground truth agrees 96.9 % with the simulator labels, the explorer maps a synthetic world to 100 % with no collisions,
and the point-cloud helpers are unit-tested. The scripts in `tools/` replay real Gazebo recordings made with `tools/record_run.py`.

**Problems the real runs found, and what was done**

| Finding | Fix |
|---|---|
| wheel odometry heading is wrong on the rough floor (446 deg reported vs 332 deg real; translation is fine) | heading now comes from the IMU gyro (bias and noise set in the SDF), distance from the wheels |
| floor on slopes was counted as obstacle (the explorer gave up at 23 %) | obstacles = vertical structure only (cells whose points span more than 25 cm in height): 99.3 % of the mapped floor was correct on a real recording |
| a crashed explorer leaves the last `/cmd_vel` running (DiffDrive keeps executing it) | the crash (int vs float) is fixed; **a watchdog that zeroes the command is still missing** |
| **scan-matching ICP is not robust on Gazebo scans**: along a smooth tunnel it jitters about 11 cm per step (about 70 % of the step); across the tunnel it is accurate (3 cm). Priors helped but never beat plain gyro odometry, and results were erratic | `scan_matching` is a switch and is **off by default in the node**: the Gazebo estimate is currently gyro + wheel odometry + keyframes + loop closure + pose graph. The web version and the offline replay tests still use ICP |

So in Gazebo the localisation currently drifts like dead reckoning (about 4 m ATE after 100 s, 9 m after 300 s with the configured gyro bias), apart from the loop closures it accepts.
Open work, in order: make scan matching robust on Gazebo scans (for example use only strongly observable directions, or a 3D / NDT matcher), add the cmd_vel watchdog, run the industrial world,
and evaluate across seeds as the web version does. Segmentation scores lower than in the web version (61 % vs 78 %) and has not been tuned for Gazebo clouds.

Other differences from the web simulator: Gazebo's `gpu_lidar` also sees the cave ceiling and the full-shell roof, missed rays are not reported (free space is only carved up to returns),
the explorer's blacklist radius is 1.0 m instead of 1.5 m, and Docker runs at well below real time with software rendering.
