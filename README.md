# Industrial Mining Facility – Three.js Simulation Prototype

Visual prototype of the world a mobile robot will later explore (ROS 2 + Gazebo) for
*Label-Free Semantic 3D Mapping and Active Exploration in GPS-Denied Environments*.
This stage is **environment design only** – no ROS 2, Gazebo physics, SLAM or navigation is included.

## Run

```bash
npm install
npm run dev          # http://localhost:5173
npm run build        # production bundle in dist/
```

Requires Node 18+.

## Controls

| Action | Input |
|---|---|
| Orbit / pan / zoom | left drag / right drag / wheel (damped) |
| Reset to isometric | `R` or *Isometric / Reset* |
| Top-down, robot view | HUD buttons |
| Debug layer | World axes, 1 m grid, bounding boxes, category labels, robot + sensor markers, nav-path placeholder |
| Semantic colours | Flat per-category colours (segmentation ground-truth preview) |
| Ambient occlusion | Optional GTAO post-process (off by default) |
| Export layout JSON | Downloads the full environment description |

## World & coordinates

- 60 m × 40 m hall, origin at the floor centre, **metres**, ROS REP-103 axes (+X forward, +Y left, +Z up, yaw CCW about +Z).
- Three.js is Y-up; the swizzle `three = (x, z, -y)` lives only in `src/core/coords.js` (and `src/models/parts.js` for composites).
- Layout: west pillared hall with container rows (1.2 m narrow passage), north-west storage room with rubble,
  central junction with a raised platform + jersey-barrier chicane, a 4 m wide **dead-end pump corridor**,
  south alcove, east ball-mill hall with fenced machinery and conveyors, and two south-east bays
  (jaw crusher; tanks + generators). Doorways are ≥ 4 m wide, forming a loop (west hall → centre → NW room → west hall) plus branches and dead ends.
- Object categories: `floor, pathway, wall, pillar, machinery, equipment, storage, fence, barrier, pipe, platform, obstacle`.

## Layout JSON (Gazebo-ready)

`data/environment.json` (regenerate with `npm run export-layout`) lists every object:
`id, name, category, model, material, position [x,y,z] (base centre), size [x,y,z], yaw, params`,
plus robot spawn, lamp positions and a placeholder nav path. The 3D scene is built **from** this JSON, so
the two cannot drift apart. Each object can later become one Gazebo model/SDF `<include>`.

## Project structure

```
src/
  main.js              HUD + wiring
  Environment.js       renderer, scene assembly, AO, render loop, dispose
  FactoryLayout.js     layout JSON -> factory scene graph
  Lighting.js          hemisphere + shadow sun + ceiling lamps
  Robot.js             placeholder robot (wheels, 3D LiDAR, RGB camera, frames)
  CameraControls.js    OrbitControls + named views
  DebugLayer.js        optional overlays (separate Group)
  SemanticView.js      category-colour material swap
  core/                coords, materials, axes helpers
  models/              instanced primitives + composite machinery/structures
  layout/              pure-data layout (dimensions, structure, zones)
scripts/
  validate-layout.mjs  overlap + robot-inflated reachability check (headless)
  export-layout.mjs    writes data/environment.json
  smoke.mjs            headless-Chrome smoke test + screenshots (needs dev server running)
```

## Verification scripts

```bash
npm run validate     # 0 overlaps; every zone reachable for a 0.45 m radius robot
npm run export-layout
npm run dev &        # then:
npm run smoke        # console errors + screenshots in ./screenshots
```

## Robot placeholder

Four-wheel base, 3D LiDAR on a mast (`lidar_link` at z = 0.72 m), RGB camera at the front
(`camera_link` at x = 0.33 m, z = 0.38 m). Sensor offsets are exported from `src/Robot.js`
(`SENSOR_FRAMES`) for reuse in the future URDF/SDF. Spawn: (-24, -6) m.

## Notes / limits

- Repeated single-geometry objects (pillars, crates, barriers, drums, containers, rocks, pipes) are `InstancedMesh`es.
- Models are visual approximations (box/cylinder composites); Gazebo collision and physics are not provided by this app.
- Stairs and the platform deck are visual only for now; the validator treats the platform as passable underneath (1.85 m clearance).

## Dashboard view (matches the reference mock-up)

Title, zone callouts (Z1 Maintenance, Z2 Processing, Z3 Utility, Z4 Storage) and a robot callout track their 3D anchors;
an axis gizmo follows the camera; and three live insets render into the same canvas via scissor viewports:

- **RGB Camera View** – rendered from the robot's `camera_link` pose (also shows the ceiling/roof).
- **LiDAR Point Cloud (3D)** – height-coloured points sampled from the layout surfaces plus 24-beam floor rings
  (an illustrative ground-truth cloud, *not* a sensor simulation).
- **Semantic Segmentation** – flat category colours with legend.

Use the **Controls** button for views, overlays, debug layer, AO, roof and JSON export. Zones are defined in `src/layout/index.js`.

## Live mapping & autonomous exploration

On load the robot starts exploring **on its own** and builds a map as it drives:

1. **Simulated 3D LiDAR** (`src/sensing/`): 16 beams (-15..+15 deg), 1.5 deg azimuth step, 25 m range, 2 cm Gaussian noise,
   5 Hz sweeps. Rays are cast analytically against box/cylinder colliders derived from the layout JSON
   (composite machines use box approximations - it is a simplified sensor world, not Gazebo physics).
2. **3D point-cloud map** (`src/mapping/MapCloud.js`): hits are voxel-deduplicated (12 cm) and colourised by height or semantic class
   (each ray knows the category of what it hit). Shown in the *LiDAR Map (live 3D)* inset and, optionally, over the main view.
3. **2D occupancy grid** (`OccupancyGrid.js`): 0.2 m log-odds grid; obstacles = hits between 0.1 and 1.5 m, so overhead pipes and the platform deck stay traversable.
   Shown in the *Occupancy Map (2D)* panel with the planned path, current goal and the robot.
4. **Frontier-based explorer** (`Explorer.js`, `Planner.js`): finds frontier clusters, plans with Dijkstra on an obstacle-inflated map (0.4 m, relaxing
   to 0.2 m to escape tight pockets), smooths the path, drives it, replans every 1.5 s, blacklists unreachable goals, and returns to the start when no frontiers remain.

Headless test of the whole loop (no browser): `node scripts/sim-explore.mjs 1500 <seed>` - reaches about 95% of the free space in roughly 5-10 simulated
minutes with 0 centre-in-obstacle samples (the last ~5% is space the LiDAR cannot reach, e.g. fully enclosed pockets). The **Controls** panel has the mode
(explore / patrol / stopped), sim speed up to 8x, chase camera, map colour mode and *Reset map & restart*.

## Realism

World-space (triplanar) procedural concrete/plaster textures with bump + roughness variation, wall/floor dirt gradients, image-based lighting for
reflections, bloom on lamps, optional GTAO, ribbed shipping containers, floor oil/skid decals, hazard-striped columns. It is a convincing *visual prototype*,
not a photogrammetry-grade scan - the final physics-accurate world will be the Gazebo model built from `data/environment.json`.

## Two environments (switch in the app)

Use the **Environment** selector (top right) to switch between

* **Industrial facility** – the original environment, unchanged (open `http://localhost:5173/`),
* **Underground cave (GPS-denied SLAM)** – new, independent (open `http://localhost:5173/?env=cave`).

Switching reloads the page, so the two never share state. `src/entry.js` picks the environment; `src/main.js` (industrial) is untouched and the cave
lives entirely in `src/cave/`. A full copy of the project from *before* the cave was added is in `../industrial-sim-3d_backup_before_cave/`.

### Cave in one paragraph

A procedurally generated natural cave (≈ 100 × 70 × 14 m): 12 tunnels (2.4–5 m wide, 2.6–4.6 m high), 4 chambers, 5 intersections with 3+ branches (A, Cathedral, north junction, east junction, Lower Hall),
3 dead ends, a grand loop, a short loop and a southern loop (the short loop runs through two deliberately look-alike tunnels to provoke perceptual
aliasing), design slopes ≈ 8° (local floor ≤ 29°), 8 pillars, 15 boulders, 6 stalactites and 6 named landmarks. It is built from a **signed-distance field** (tunnel/chamber cross-sections
splatted along Catmull-Rom centre-lines, formations carved in, height-aware fractal noise added) meshed with **surface nets**. The *same* field is
sphere-traced by the simulated LiDAR, so what you see is what the sensor measures. Everything is configured in `src/cave/CaveConfiguration.json`
(metres, ROS axes); `npm run export-cave` writes the structured description for Gazebo to `data/cave.json`. See **CAVE.md**.

Cave controls: isometric / top-down / robot-chase / robot-camera / reset, **roof visibility toggle**, orbit/zoom/pan, LiDAR visualisation, map colouring, and a
SLAM-debug layer (tunnel graph, intersections, landmarks, start, trajectory, loop-closure *candidates*, sensor markers, bounding boxes, axes, explored/unexplored tint).
These are **visualisation placeholders computed from the simulator's ground-truth pose – not SLAM output**.

Headless checks: `npm run test:cave` (generation time, mesh size, connectivity of every chamber/junction/dead end from the entrance),
`npm run sim:cave` (closed-loop LiDAR → map → exploration in the cave), `npm run sim:industrial`.

## SLAM estimate, segmentation and evaluation (cave)

`src/slam/` adds a shadow pose estimator (simulated odometry → LiDAR scan matching → loop closure → pose graph) and a label-free geometric segmenter; see `ALGORITHMS.md`
for the method and the measured results. Run `npm run sim:slam` (one run with drift/error log), `npm run evaluate` (seeds × exploration strategies → CSV) and `npm run smoke:slam` (browser check, needs `npm run dev`).
