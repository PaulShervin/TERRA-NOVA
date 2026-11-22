# Underground cave – GPS-denied SLAM test environment

*Visual prototype for design and demonstration. Three.js is NOT performing SLAM here: the robot pose is ground truth from the simulator.
The real SLAM, LiDAR processing, IMU integration and localisation will be done in the ROS 2 / Gazebo stack.*

## Layout (`src/cave/CaveConfiguration.json`, metres, ROS axes: x east, y north, z up)

```
                    T2 ── C2 Crystal Room ── T3
                   /                           \
  T1 (look-alike A)                              JE ── T5 ── C3 Lower Hall ── T11 ─ dead end
 /                 \                            /  \          |
A ── T0 main ──────  C1 Cathedral ── T4 ───────/    T8 ─ JS ─ T9   (southern loop)
 \                  |  \ T10 (look-alike A) → JN  
  T7 squeeze        T6 (look-alike B)
  → pocket C4       → dead end
```

| Feature | Where |
|---|---|
| Start | `N_ENTRANCE` (−42, 0); robot spawns at (−40.5, 0) |
| Main passage | T0 → C1 → T4 → JE → T5 → C3 |
| Chambers | Cathedral (24×18×12 m), Crystal Room (16×13×8 m), Lower Hall (20×17×10 m), Squeeze Pocket |
| Branching routes (≥ 4) | T1, T7, T6, T8, T3, T10, T11 off the main line |
| Intersections | junctions/chambers with ≥ 3 tunnels (exported in `graph.intersections`) |
| Narrow passages | T6, T7, T11 (2.4–3.0 m wide) |
| Dead ends | T6 (south), T7 → Squeeze Pocket, T11 (east) |
| Alternative routes | T10 (Cathedral ↔ north junction), T9 (south ↔ Lower Hall) |
| **Loops** | `LOOP_A_GRAND` (A → north → Crystal Room → east → Cathedral → A), `LOOP_B_SHORT` (through look-alike T1/T10), `LOOP_C_SOUTH` |
| Look-alike sections | T1 & T10 (3.2–3.6 m curved), T6 & T7 (≈ 2.5 m) – perceptual-aliasing tests |
| Landmarks | 3 pillars, 2 boulders, 1 stalactite (`landmarks` in the config / export) |

All floors are ≤ 29° locally (design slopes ≈ 8°); every drivable passage is ≥ 1.4 m wide, the robot is ≈ 1.0 × 0.7 m.

## How the geometry is made (and why LiDAR matches the picture)

1. `CaveTunnelNetwork.js` – Catmull-Rom centre-lines through `[x, y, floorZ, width, height]` control points, chambers, formations, graph analysis.
2. `CaveField.js` – signed-distance field (air < 0): super-ellipse (p = 4: flat floor, arched ceiling) cross-sections splatted along the
   centre-lines, pillars / boulders / stalactites carved out, then 3-octave value noise whose amplitude grows with height above the floor
   (rough walls/ceiling, drivable floor).
3. `CaveMesh.js` – surface nets → triangle mesh with gradient normals; triangles facing down form a separately hideable *roof*.
4. `CaveField.cast` – sphere tracing of the same field = LiDAR ray casting (≈ 15 ms / 3,840-ray sweep).
5. `buildFloorMap` – per-column interpolated floor height, clearance and drivable mask (used for robot terrain-following and ground-truth coverage).

## Sensor frames (identical to the industrial robot, plus IMU)

`base_link (0,0,0)` · `lidar_link (0,0,0.72)` · `camera_link (0.33,0,0.38)` · `imu_link (0,0,0.30)` – model units before the 1.35× robot scale.

## Making it in Gazebo

* `data/cave.json` (from `npm run export-cave`) holds tunnel control points, nodes, graph, loops, formations with bounding boxes, landmarks and lamps.
* Easiest faithful route: export the surface mesh (`extractSurface(field)` in `CaveMesh.js` returns positions/indices in ROS axes) to OBJ/STL/DAE and use it as
  both visual and collision mesh in an SDF world; place the robot at the spawn pose; add the same sensor frames.
* Gazebo's `gpu_lidar` then sees the same geometry the Three.js LiDAR sphere-traces here.
