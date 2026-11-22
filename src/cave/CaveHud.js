// DOM heads-up display for the cave environment (kept separate from the industrial dashboard).
const DEBUG_TOGGLES = [
  ['network', 'Tunnel network + intersections', true], ['landmarks', 'Landmark markers', true], ['start', 'Start position', true],
  ['trail', 'Trajectory (visited path)', true], ['loops', 'Loop-closure candidates (heuristic)', true], ['sensors', 'Sensor positions', false],
  ['boxes', 'Formation bounding boxes', false], ['axes', 'Coordinate axes', false],
];

export function createCaveHud(handlers) {
  const root = document.createElement('div');
  root.id = 'cave-hud';
  root.innerHTML = `
    <header id="cave-title">
      <h1>Underground Cave – GPS-denied SLAM Test Environment</h1>
      <p>3D LiDAR mapping · loop closure · autonomous exploration &nbsp;<em>(the simulator drives the robot; a SLAM estimate runs alongside in shadow mode)</em></p>
    </header>
    <button id="cave-controls-toggle" class="panel">&#9881; Controls</button>
    <div id="cave-controls" class="panel" hidden></div>
    <aside id="cave-map" class="panel">
      <h3>Occupancy Map (2D)</h3>
      <div class="cave-map-view"><canvas id="cave-minimap"></canvas><div id="cave-map-status"></div></div>
      <ul class="cave-legend">
        <li><i style="background:#3a4a62"></i>explored (known free)</li>
        <li><i style="background:#0e1420;border:1px solid #2a3a55"></i>unexplored (unknown)</li>
        <li><i style="background:#ebf0fa"></i>obstacle / rock</li>
        <li><i style="background:#39ff88"></i>planned path</li>
        <li><i style="background:#ffcc00"></i>robot (current position)</li>
        <li><i style="background:#5a8bff"></i>previously visited (3D trail)</li>
      </ul>
      <p class="cave-note">This map, the 3D trail and the yellow-ringed loop markers use the simulator's <b>ground-truth</b> pose. The SLAM estimate (green) is separate and only scored against it.</p>
    </aside>
    <div id="cave-slam-card" class="panel" hidden><h3>Localisation (no GPS)</h3><div id="cave-slam-stats"></div></div>
    <div id="cave-loading" hidden><div class="spinner"></div><span id="cave-loading-text">Generating cave...</span></div>`;
  document.body.appendChild(root);

  const controls = root.querySelector('#cave-controls');
  controls.innerHTML = `
    <div>
      <button data-view="isometric">Isometric</button><button data-view="top">Top-down</button>
      <button data-view="chase">Robot (chase)</button><button data-view="fpv">Robot camera</button>
      <button data-view="reset">Reset camera</button>
    </div>
    <div><b>Environment</b></div>
    <label><input type="checkbox" id="cave-roof"> Cave roof visible</label>
    <label><input type="checkbox" id="cave-lidarviz" checked> LiDAR sweep visualisation</label>
    <div><b>Robot</b></div>
    <label>Mode <select id="cave-mode"><option value="explore">Autonomous exploration (mapping)</option><option value="idle">Stopped</option></select></label>
    <label>Sim speed <select id="cave-speed"><option value="1">1x</option><option value="2">2x</option><option value="4">4x</option><option value="8">8x</option></select></label>
    <div style="margin-top:4px"><button id="cave-reset">Reset map &amp; restart</button></div>
    <div><b>Map</b></div>
    <label><input type="checkbox" id="cave-mappoints" checked> Show LiDAR map points</label>
    <label>Colour <select id="cave-mapcolor"><option value="height">Height</option><option value="semantic">Semantic class</option></select></label>
    <div><b>SLAM debug layer (placeholders)</b></div>
    ${DEBUG_TOGGLES.map(([k, l, on]) => `<label><input type="checkbox" data-debug="${k}" ${on ? 'checked' : ''}> ${l}</label>`).join('')}
    <label><input type="checkbox" id="cave-tint"> Tint rock: explored vs unexplored</label>
    <div><b>SLAM estimate (shadow mode)</b></div>
    <label><input type="checkbox" data-slam="est" checked> <span style="color:#35ff8c">&#9632;</span> LiDAR odometry + pose graph</label>
    <label><input type="checkbox" data-slam="odom" checked> <span style="color:#ff4d4d">&#9632;</span> Odometry only (drifts)</label>
    <label><input type="checkbox" data-slam="loops" checked> <span style="color:#ffd23a">&#9632;</span> Accepted loop closures</label>
    <div><b>Label-free segmentation</b></div>
    <div><button id="cave-segment">Segment current map</button> <label><input type="checkbox" id="cave-segview" checked> show</label></div>
    <div id="cave-seg-result"></div>
    <div style="margin-top:6px"><button id="cave-export">Export cave JSON</button></div>
    <div id="cave-readout"></div>`;

  const $ = (sel) => root.querySelector(sel);
  $('#cave-controls-toggle').addEventListener('click', () => { controls.hidden = !controls.hidden; });
  controls.addEventListener('click', (e) => {
    if (e.target.dataset?.view) handlers.view(e.target.dataset.view);
    if (e.target.id === 'cave-reset') handlers.reset();
    if (e.target.id === 'cave-export') handlers.export();
    if (e.target.id === 'cave-segment') handlers.segment?.();
  });
  controls.addEventListener('change', (e) => {
    const t = e.target;
    if (t.dataset?.debug) handlers.debug(t.dataset.debug, t.checked);
    if (t.dataset?.slam) handlers.slamTraj?.(t.dataset.slam, t.checked);
    if (t.id === 'cave-segview') handlers.segView?.(t.checked);
    if (t.id === 'cave-roof') handlers.roof(t.checked);
    if (t.id === 'cave-lidarviz') handlers.lidarviz(t.checked);
    if (t.id === 'cave-mode') handlers.mode(t.value);
    if (t.id === 'cave-speed') handlers.speed(Number(t.value));
    if (t.id === 'cave-mappoints') handlers.mapPoints(t.checked);
    if (t.id === 'cave-mapcolor') handlers.mapColor(t.value);
    if (t.id === 'cave-tint') handlers.tint(t.checked);
  });

  return {
    root,
    slamStats: $('#cave-slam-stats'), slamCard: $('#cave-slam-card'), minimap: $('#cave-minimap'), mapStatus: $('#cave-map-status'), readout: $('#cave-readout'), segResult: $('#cave-seg-result'),
    debugDefaults: DEBUG_TOGGLES.map(([k, , on]) => [k, on]),
    setLoading(on, text) { $('#cave-loading').hidden = !on; if (text) $('#cave-loading-text').textContent = text; },
    dispose() { root.remove(); },
  };
}
