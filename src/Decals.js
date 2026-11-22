// Floor wear decals (oil stains, tyre marks, scuffs): transparent textured quads just above the slab.
// Purely visual; they are not part of the layout JSON or the LiDAR world.
import * as THREE from 'three';
import { rosToThree } from './core/coords.js';

function blotchTexture(kind, seed) {
  let s = seed;
  const rnd = () => ((s = (s * 16807) % 2147483647) / 2147483647);
  const S = 256, c = document.createElement('canvas');
  c.width = c.height = S;
  const g = c.getContext('2d');
  if (kind === 'oil') {
    for (let i = 0; i < 26; i++) {
      const r = 18 + rnd() * 70, x = S / 2 + (rnd() - 0.5) * 90, y = S / 2 + (rnd() - 0.5) * 90;
      const grad = g.createRadialGradient(x, y, 0, x, y, r);
      grad.addColorStop(0, 'rgba(8,8,10,0.28)'); grad.addColorStop(1, 'rgba(8,8,10,0)');
      g.fillStyle = grad; g.beginPath(); g.arc(x, y, r, 0, Math.PI * 2); g.fill();
    }
  } else { // tyre / skid marks: two parallel streaks
    g.strokeStyle = 'rgba(10,10,10,0.35)'; g.lineWidth = 9; g.lineCap = 'round';
    for (const off of [-24, 24]) {
      g.beginPath(); g.moveTo(10, S / 2 + off); g.bezierCurveTo(S * 0.35, S / 2 + off + 18, S * 0.65, S / 2 + off - 18, S - 10, S / 2 + off); g.stroke();
    }
    g.globalCompositeOperation = 'destination-in';
    const fade = g.createLinearGradient(0, 0, S, 0);
    fade.addColorStop(0, 'rgba(0,0,0,0)'); fade.addColorStop(0.2, 'rgba(0,0,0,1)'); fade.addColorStop(0.8, 'rgba(0,0,0,1)'); fade.addColorStop(1, 'rgba(0,0,0,0)');
    g.fillStyle = fade; g.fillRect(0, 0, S, S);
  }
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  return t;
}

// [kind, x, y, size, yaw] in ROS metres
const PLACEMENTS = [
  ['oil', -22, -3, 3.0, 0.4], ['oil', -8, -6.5, 2.4, 1.2], ['oil', 14, -13, 3.4, 0.2], ['oil', 20, 7, 3.0, 2.0],
  ['oil', 26, -9.5, 2.2, 0.9], ['oil', -26, 9, 2.6, 0.1], ['oil', 4, 9, 2.0, 0.7], ['oil', 12, 15, 2.8, 1.6],
  ['skid', -14, -6, 6.0, 0.0], ['skid', 4, -1, 5.0, Math.PI / 2], ['skid', 16, -10, 6.0, 0.0], ['skid', 18, 6, 6.0, 0.0],
  ['skid', -20, 2, 5.5, Math.PI / 2], ['skid', 0, 12, 5.0, Math.PI / 2],
];

export function createDecals() {
  const group = new THREE.Group();
  group.name = 'floor_decals';
  const textures = { oil: [blotchTexture('oil', 11), blotchTexture('oil', 29)], skid: [blotchTexture('skid', 5)] };
  const mats = [];
  const geo = new THREE.PlaneGeometry(1, 1).rotateX(-Math.PI / 2);
  PLACEMENTS.forEach(([kind, x, y, size, yaw], i) => {
    const list = textures[kind];
    const mat = new THREE.MeshStandardMaterial({
      map: list[i % list.length], transparent: true, depthWrite: false, roughness: 0.45,
      polygonOffset: true, polygonOffsetFactor: -4, polygonOffsetUnits: -4,
    });
    mats.push(mat);
    const m = new THREE.Mesh(geo, mat);
    m.position.copy(rosToThree([x, y, 0.015]));
    m.rotation.y = yaw;
    m.scale.set(kind === 'skid' ? size : size, 1, kind === 'skid' ? size * 0.4 : size);
    m.receiveShadow = true;
    m.userData.category = 'floor'; // shows as floor in the semantic view
    group.add(m);
  });
  return {
    group,
    dispose: () => { geo.dispose(); mats.forEach((m) => m.dispose()); Object.values(textures).flat().forEach((t) => t.dispose()); },
  };
}
