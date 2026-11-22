import './ui.css';
import { createEnvironment } from './Environment.js';
import { createOverlay } from './Overlay.js';
import { createAlgorithmGuide } from './guide/AlgorithmGuide.js';

const env = createEnvironment(document.getElementById('app'));
const overlay = createOverlay(env);
const guide = createAlgorithmGuide(env);
document.getElementById('algo-btn').addEventListener('click', () => guide.toggle());

const debugToggles = [
  ['axes', 'World axes'], ['grid', 'Grid (1 m / 5 m)'], ['boxes', 'Bounding boxes'],
  ['labels', 'Category labels'], ['robot', 'Robot + sensor markers'], ['path', 'Nav-path placeholder'],
];

const controls = document.getElementById('controls');
controls.innerHTML = `
  <div>
    <button data-view="isometric">Isometric / Reset</button>
    <button data-view="top">Top-down</button>
    <button data-view="robot">Robot view</button>
  </div>
  <div><b>Robot</b></div>
  <label>Mode <select id="mode"><option value="explore">Autonomous exploration (mapping)</option><option value="patrol">Patrol loop</option><option value="idle">Stopped</option></select></label>
  <label>Sim speed <select id="simspeed"><option value="1">1x</option><option value="2">2x</option><option value="4">4x</option><option value="8">8x</option></select></label>
  <label><input type="checkbox" id="follow"> Chase camera</label>
  <label>Robot speed <input type="range" id="speed" min="0.4" max="2.0" step="0.1" value="1.2" style="vertical-align:middle;width:100px"> <span id="speed-val">1.2</span> m/s</label>
  <div style="margin-top:4px"><button id="resetmap">Reset map &amp; restart</button></div>
  <div><b>Map</b></div>
  <label><input type="checkbox" id="mapmain" checked> Show map points in main view</label>
  <label>Colour <select id="mapcolor"><option value="height">Height</option><option value="semantic">Semantic class</option></select></label>
  <div><b>Overlays</b></div>
  <label><input type="checkbox" id="callouts" checked> Zone &amp; robot callouts</label>
  <label><input type="checkbox" id="scan" checked> LiDAR sweep visualisation</label>
  <div><b>Debug layer</b></div>
  ${debugToggles.map(([k, l]) => `<label><input type="checkbox" data-debug="${k}"> ${l}</label>`).join('')}
  <div><b>Rendering</b></div>
  <label><input type="checkbox" id="semantic"> Semantic colours (main view)</label>
  <label><input type="checkbox" id="ao"> Ambient occlusion (GTAO)</label>
  <label><input type="checkbox" id="roof"> Roof (ceiling + beams)</label>
  <div style="margin-top:6px"><button id="export">Export layout JSON</button></div>
  <div id="readout"></div>`;

document.getElementById('controls-toggle').addEventListener('click', () => { controls.hidden = !controls.hidden; });
controls.addEventListener('click', (ev) => {
  const view = ev.target.dataset?.view;
  if (view) env.cameraRig.setView(view);
});
controls.addEventListener('input', (ev) => { if (ev.target.id === 'speed') ev.target.dispatchEvent(new Event('change', { bubbles: true })); });
controls.addEventListener('change', (ev) => {
  const t = ev.target;
  if (t.dataset?.debug) env.debug.setPart(t.dataset.debug, t.checked);
  if (t.id === 'semantic') env.setSemantic(t.checked);
  if (t.id === 'ao') env.setAO(t.checked);
  if (t.id === 'roof') env.setRoof(t.checked);
  if (t.id === 'mode') env.mapper.setMode(t.value);
  if (t.id === 'simspeed') env.mapper.setSimSpeed(Number(t.value));
  if (t.id === 'mapmain') env.setMapVisible(t.checked);
  if (t.id === 'mapcolor') env.mapper.cloud.setColorMode(t.value);
  if (t.id === 'follow') env.setFollow(t.checked);
  if (t.id === 'speed') { env.driver.setSpeed(Number(t.value)); document.getElementById('speed-val').textContent = t.value; }
  if (t.id === 'callouts') overlay.setVisible(t.checked);
  if (t.id === 'scan') env.setOverlay(t.checked);
});
document.getElementById('resetmap').addEventListener('click', () => env.mapper.reset());
document.getElementById('export').addEventListener('click', () => {
  const blob = new Blob([JSON.stringify(env.layout, null, 2)], { type: 'application/json' });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = 'environment.json';
  a.click();
  URL.revokeObjectURL(a.href);
});

const readout = document.getElementById('readout');
setInterval(() => {
  const [x, y, z] = env.getCameraRos();
  readout.textContent = `cam (ROS) x ${x.toFixed(1)} y ${y.toFixed(1)} z ${z.toFixed(1)} m\n`
    + `${env.stats.fps.toFixed(0)} fps · ${env.stats.calls} draw calls`;
}, 500);

window.addEventListener('keydown', (e) => { if (e.key === 'r' || e.key === 'R') env.cameraRig.reset(); });
window.__env = env; // handy for console inspection during development
