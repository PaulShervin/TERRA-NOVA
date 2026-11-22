// DOM overlay: zone/robot callouts with leader lines tracking 3D anchors, details panel, legend.
import * as THREE from 'three';
import { rosToThree } from './core/coords.js';
import { CATEGORY_COLORS } from './core/materials.js';

const NS = 'http://www.w3.org/2000/svg';
const LEGEND = [['Floor', 'floor'], ['Wall', 'wall'], ['Pillar', 'pillar'], ['Machine', 'machinery'],
  ['Container', 'container'], ['Rack', 'rack'], ['Barrier', 'barrier']];

export function createOverlay(env) {
  const { layout, camera, robot } = env;
  const b = layout.meta.bounds;
  const height = 6.5;

  document.getElementById('legend').innerHTML = LEGEND
    .map(([label, cat]) => `<li><i style="background:${CATEGORY_COLORS[cat]}"></i>${label}</li>`).join('');
  document.getElementById('details-list').innerHTML = [
    ['&#9635;', `Area : ${b.xMax - b.xMin} m &times; ${b.yMax - b.yMin} m &times; ${height} m`],
    ['&#9776;', `Zones : ${layout.zones.length} (${layout.zones[0].id}&ndash;${layout.zones.at(-1).id})`],
    ['&#10059;', 'Interconnected Paths'],
    ['&#10010;', 'Intersections &amp; Dead Ends'],
    ['&#10070;', 'Assets : Pillars, Machinery,<br>Tanks, Racks, Barriers, Containers'],
  ].map(([i, t]) => `<li><span class="ico">${i}</span><span>${t}</span></li>`).join('');

  const host = document.getElementById('callouts');
  const svg = document.getElementById('leaders');
  const items = [
    ...layout.zones.map((z) => ({
      html: `${z.name}<small>(${z.id})</small>`, kind: 'zone',
      anchor: () => rosToThree(z.anchor),
    })),
    {
      html: 'Autonomous Mobile Robot<small>(3D LiDAR + RGB Camera)</small>', kind: 'robot',
      anchor: () => robot.root.position.clone().add(new THREE.Vector3(0, 0.5, 0)),
    },
  ].map((it) => {
    const el = document.createElement('div');
    el.className = 'callout';
    el.innerHTML = it.html;
    host.appendChild(el);
    const line = document.createElementNS(NS, 'line');
    const dot = document.createElementNS(NS, 'circle');
    dot.setAttribute('r', '3.5');
    svg.append(line, dot);
    return { ...it, el, line, dot };
  });

  let visible = true;
  let raf = 0;
  const tick = () => {
    raf = requestAnimationFrame(tick);
    const W = window.innerWidth, H = window.innerHeight;
    // The main view is shifted left on wide screens (see CameraControls), so the "centre" is left of canvas centre.
    const cx = W > 1100 ? W / 2 - 120 : W / 2, cy = H / 2;
    for (const it of items) {
      const v = it.anchor().project(camera);
      const show = visible && v.z < 1 && Math.abs(v.x) < 1.2 && Math.abs(v.y) < 1.2;
      it.el.style.display = it.line.style.display = it.dot.style.display = show ? '' : 'none';
      if (!show) continue;
      const ax = (v.x * 0.5 + 0.5) * W, ay = (-v.y * 0.5 + 0.5) * H;
      let dx, dy;
      if (it.kind === 'robot') { dx = 120; dy = 105; } else {
        dx = Math.max(-110, Math.min(110, ((ax - cx) / W) * 320)); dy = -85; // above the target, leaning outward
      }
      const ly = Math.min(H - 40, Math.max(150, ay + dy));
      const lx = Math.min(W - 430, Math.max(ly > H - 300 ? 420 : 110, ax + dx));
      it.el.style.left = `${lx}px`; it.el.style.top = `${ly}px`;
      const hh = it.el.offsetHeight / 2;
      it.line.setAttribute('x1', lx); it.line.setAttribute('y1', ly + Math.sign(ay - ly || 1) * hh);
      it.line.setAttribute('x2', ax); it.line.setAttribute('y2', ay);
      it.dot.setAttribute('cx', ax); it.dot.setAttribute('cy', ay);
    }
  };
  tick();

  return {
    setVisible: (on) => { visible = on; },
    dispose: () => { cancelAnimationFrame(raf); host.replaceChildren(); svg.replaceChildren(); },
  };
}
