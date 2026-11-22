// In-app "How it works" guide: one tab per algorithm, each with the maths, our parameters and a LIVE drawing
// of the real pipeline state (not a canned animation).
import { drawLidar, drawOccupancy, drawFrontiers, drawPlanning, drawControl } from './guideDraw.js';

const TABS = [
  {
    id: 'overview', title: '0 · Pipeline', draw: null,
    html: `<h3>The loop the robot runs, 5 times per second</h3>
<ol class="flow">
  <li><b>Sense</b> – simulated 3D LiDAR sweep (ray casting)</li>
  <li><b>Map</b> – 3D point-cloud map + 2D occupancy grid (log-odds)</li>
  <li><b>Decide</b> – frontier detection &amp; goal selection</li>
  <li><b>Plan</b> – Dijkstra on an obstacle-inflated grid + path smoothing</li>
  <li><b>Act</b> – differential-drive controller follows the waypoints</li>
</ol>
<div class="live" id="g-overview-live"></div>
<p class="note"><b>Important honesty note:</b> in this Three.js prototype the robot's pose comes from the simulation itself (ground truth).
A real GPS-denied robot must <b>estimate</b> its pose with SLAM (scan matching / LiDAR-inertial odometry + loop closure).
Everything below the "Sense" step stays the same in the real system; only the pose source changes.</p>`,
  },
  {
    id: 'lidar', title: '1 · LiDAR ray casting', draw: (c, ctx) => drawLidar(c, ctx.mapper),
    html: `<h3>Sensor model</h3>
<p>A spinning 16-beam LiDAR (Velodyne-VLP-16-like): elevations −15°…+15° in 2° steps, azimuth step 1.5°, range 0.3–25 m, σ = 2 cm noise, 5 sweeps/s.
That is 16 × 240 = <b>3,840 rays per sweep</b>.</p>
<pre>for beam b, azimuth a:
  d = (cos(az)·cos(el), sin(az)·cos(el), sin(el))
  t = closest intersection of  o + t·d  with the scene
  range = t + N(0, σ²)        // measurement noise
  hit   = o + range·d         // world-frame 3D point</pre>
<p><b>Scene intersection.</b> Each layout object becomes oriented boxes / vertical cylinders (plus floor &amp; ceiling planes).
A cheap AABB test rejects most objects, then an exact <i>slab test</i> is done in the box's local frame
(rotate the ray by −yaw). Cylinders solve a quadratic. Cost ≈ 12 ms per sweep in JavaScript.</p>
<p><b>Why it matters:</b> each ray also carries the <i>category</i> of what it hit, which lets us colour the map semantically.</p>
<p class="note">Left: what the horizontal beam sees (range per azimuth). Right: the 16 vertical beams straight ahead.</p>`,
  },
  {
    id: 'occ', title: '2 · Occupancy grid', draw: (c, ctx) => drawOccupancy(c, ctx.mapper, ctx.pose),
    html: `<h3>Log-odds occupancy mapping</h3>
<p>The floor plan is a 0.2 m grid (300 × 200 cells). Every cell stores a <b>log-odds</b> value <i>l = log(p / (1−p))</i>;
0 means "unknown". Each sweep adds evidence per cell:</p>
<pre>l(cell) += 1.00    if a ray ENDS in the cell          (occupied evidence)
l(cell) -= 0.25    if a ray PASSES THROUGH the cell    (free evidence)
l = clamp(l, −2, +5)
occupied if l &gt; 0.85   free if l &lt; −0.35   else uncertain</pre>
<ul>
<li><b>Ray carving:</b> we step along each ray's horizontal projection every 0.1 m, marking cells free up to the hit
(stopping 1.5 cells early so thin walls aren't erased).</li>
<li><b>Height band:</b> only hits between 0.1 m and 1.5 m high count as obstacles – overhead pipes and the platform deck stay traversable.</li>
<li><b>Once per sweep:</b> each cell is updated at most once per scan, so 16 beams don't over-count.</li>
<li><b>Asymmetric clamps</b> make occupied cells "sticky" so grazing rays can't erase 6 cm fences.</li>
</ul>`,
  },
  {
    id: 'frontier', title: '3 · Frontier exploration', draw: (c, ctx) => drawFrontiers(c, ctx.mapper, ctx.pose),
    html: `<h3>Frontier-based exploration (Yamauchi, 1997)</h3>
<p>A <b>frontier</b> is a known-free, reachable cell that touches unknown space. Driving to a frontier reveals new map, so
"explore" becomes "keep going to the best frontier until none are left".</p>
<pre>1. cells = free ∧ robot-fits ∧ has an unknown 4-neighbour
2. cluster cells (8-connected flood fill), drop clusters &lt; 5 cells
3. for each cluster: nearest cell by PATH cost c
4. score = c − 0.12·min(size, 40)            // prefer near &amp; large
        − 3 if close to the previous goal     // hysteresis, no dithering
5. goal = argmin score   (skip blacklisted / too-close goals)</pre>
<p>When no frontier remains the explorer sets the start pose as goal and finishes with <i>Mapping complete</i>.
If the robot stops making progress for 4 s the goal is blacklisted for 60 s.</p>`,
  },
  {
    id: 'plan', title: '4 · Path planning', draw: (c, ctx) => drawPlanning(c, ctx.mapper, ctx.pose),
    html: `<h3>Dijkstra on an inflated grid + string-pulling</h3>
<ol>
<li><b>Inflate obstacles</b> by the robot radius (0.5 m; relaxes to 0.4 → 0.3 m to escape tight pockets): any free cell closer than that to an occupied cell is "robot would not fit".</li>
<li><b>Dijkstra</b> (binary-heap priority queue, 8-connected, √2 diagonal cost, no corner cutting) from the robot gives the cost-to-reach of every cell – that heat map is what you see.</li>
<li><b>Backtrack</b> parents from the goal to get a cell path.</li>
<li><b>Smooth</b> (string-pulling): drop intermediate cells while a straight line between the kept ones stays free, leaving a few long segments.</li>
<li>Replan every 1.5 s of sim time, or immediately when the route ends.</li>
</ol>
<p>Unknown space is treated as blocked, so the robot only ever drives through cells it has already seen to be free.</p>`,
  },
  {
    id: 'control', title: '5 · Motion control', draw: (c, ctx) => drawControl(c, ctx.driver),
    html: `<h3>Differential-drive waypoint following</h3>
<pre>err = wrap(atan2(ty − y, tx − x) − yaw)
w   = clamp(2.8 · err, ±1.8 rad/s)             // proportional turn
v   = v_max · cos²(min(|err|, 90°))             // slow when misaligned
yaw += w·dt ;  x += v·cos(yaw)·dt ;  y += v·sin(yaw)·dt
v_left  = v − w·(track/2)     v_right = v + w·(track/2)
wheel angle += (v_wheel / r) · dt</pre>
<p>A waypoint counts as reached within 12 cm (15 cm for the final goal). Physics is integrated in 20 ms sub-steps so speed is
real-time even at low frame-rate. This is a kinematic stand-in – the real robot will receive the same <code>v, w</code> as a
<code>/cmd_vel</code> message in ROS 2.</p>`,
  },
  {
    id: 'ros', title: '6 · Doing it for real (ROS 2)', draw: null,
    html: `<h3>Mapping this prototype onto the real stack</h3>
<table class="map">
<tr><th>Prototype stage</th><th>Real system (ROS 2 / Gazebo)</th></tr>
<tr><td>Simulated LiDAR</td><td>Gazebo <code>gpu_lidar</code> sensor → <code>sensor_msgs/PointCloud2</code> on <code>/points</code></td></tr>
<tr><td>Pose from simulator</td><td><b>SLAM</b>: LiDAR-inertial odometry (FAST-LIO2 / LIO-SAM) or 3D SLAM (RTAB-Map, Cartographer 3D) + IMU; loop closure corrects drift</td></tr>
<tr><td>Voxel point map</td><td>Accumulated map in the SLAM back-end, or OctoMap / voxblox</td></tr>
<tr><td>2D occupancy grid</td><td><code>nav_msgs/OccupancyGrid</code> from slam_toolbox or <code>pointcloud_to_laserscan</code> + Nav2 costmap</td></tr>
<tr><td>Frontier explorer</td><td><code>explore_lite</code> (m-explore-ros2) or your own node on the occupancy grid</td></tr>
<tr><td>Dijkstra + smoothing</td><td>Nav2 planner server (NavFn / SmacPlanner) + smoother</td></tr>
<tr><td>Waypoint controller</td><td>Nav2 controller (Regulated Pure Pursuit / DWB) → <code>/cmd_vel</code></td></tr>
<tr><td><code>data/environment.json</code></td><td>Generate an SDF world: one <code>&lt;model&gt;</code> per object (box / cylinder collisions)</td></tr>
</table>
<h3>Suggested order of work</h3>
<ol>
<li>Build the SDF world from <code>environment.json</code>, spawn a differential-drive robot with LiDAR + IMU + camera.</li>
<li>Bridge sensors with <code>ros_gz_bridge</code>; verify <code>/points</code>, <code>/imu</code>, <code>/odom</code>.</li>
<li>Run LiDAR-inertial odometry; compare its pose with Gazebo ground truth (ATE / RPE).</li>
<li>Feed Nav2 with the SLAM map + pose; add <code>explore_lite</code> for autonomous exploration.</li>
<li>Add semantic segmentation of the point cloud (e.g. RandLA-Net / label-free clustering) – this prototype's per-point categories are the ground truth to evaluate against.</li>
</ol>`,
  },
];

export function createAlgorithmGuide(env) {
  const root = document.createElement('div');
  root.id = 'guide';
  root.hidden = true;
  root.innerHTML = `
    <div class="guide-card panel">
      <button class="guide-close" aria-label="Close">&times;</button>
      <h2>How the mapping &amp; exploration algorithms work</h2>
      <div class="guide-body">
        <nav>${TABS.map((t) => `<button data-tab="${t.id}">${t.title}</button>`).join('')}</nav>
        <section>
          <div class="guide-text"></div>
          <canvas class="guide-canvas"></canvas>
        </section>
      </div>
    </div>`;
  document.body.appendChild(root);

  const text = root.querySelector('.guide-text');
  const canvas = root.querySelector('.guide-canvas');
  let current = TABS[0];
  let timer = 0;

  function ctxObj() { return { mapper: env.mapper, driver: env.driver, pose: env.driver.pose }; }

  function updateLive() {
    if (current.draw) { canvas.style.display = ''; current.draw(canvas, ctxObj()); } else canvas.style.display = 'none';
    const live = root.querySelector('#g-overview-live');
    if (live) {
      const s = env.mapper.stats();
      live.innerHTML = `<div><span>${s.state}</span>state</div><div><span>${(s.coverage * 100).toFixed(0)}%</span>explored</div>
        <div><span>${(s.points / 1000).toFixed(1)}k</span>map points</div><div><span>${s.frontiers}</span>frontiers</div><div><span>${s.travelled.toFixed(0)} m</span>driven</div>`;
    }
  }
  function select(id) {
    current = TABS.find((t) => t.id === id) ?? TABS[0];
    text.innerHTML = current.html;
    root.querySelectorAll('nav button').forEach((b) => b.classList.toggle('on', b.dataset.tab === current.id));
    updateLive();
  }
  function open() { root.hidden = false; select(current.id); timer = setInterval(updateLive, 500); }
  function close() { root.hidden = true; clearInterval(timer); }

  root.querySelector('nav').addEventListener('click', (e) => { if (e.target.dataset.tab) select(e.target.dataset.tab); });
  root.querySelector('.guide-close').addEventListener('click', close);
  root.addEventListener('click', (e) => { if (e.target === root) close(); });
  window.addEventListener('keydown', (e) => { if (e.key === 'Escape' && !root.hidden) close(); });

  return { open, close, toggle: () => (root.hidden ? open() : close()), dispose: () => { clearInterval(timer); root.remove(); } };
}
