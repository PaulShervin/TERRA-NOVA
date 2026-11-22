// Procedural PBR-ish surface textures (canvas-generated, no external assets).
import * as THREE from 'three';

function makeRng(seed) {
  let s = seed >>> 0 || 1;
  return () => ((s = (s * 1664525 + 1013904223) >>> 0) / 4294967296);
}

/** Tileable value-noise lattice; returns sample(x, y) in 0..1 for x,y in [0,1) wrapping seamlessly. */
function makeNoise(rng, cells) {
  const g = new Float32Array(cells * cells);
  for (let i = 0; i < g.length; i++) g[i] = rng();
  const smooth = (t) => t * t * (3 - 2 * t);
  return (x, y) => {
    const fx = x * cells, fy = y * cells;
    const x0 = Math.floor(fx), y0 = Math.floor(fy);
    const tx = smooth(fx - x0), ty = smooth(fy - y0);
    const at = (i, j) => g[((j % cells) + cells) % cells * cells + (((i % cells) + cells) % cells)];
    return (at(x0, y0) * (1 - tx) + at(x0 + 1, y0) * tx) * (1 - ty) + (at(x0, y0 + 1) * (1 - tx) + at(x0 + 1, y0 + 1) * tx) * ty;
  };
}

/**
 * Concrete / plaster texture: multi-octave mottling, fine aggregate speckle, stains, hairline cracks and
 * optional slab joints on the tile border.
 */
export function concreteTexture({ size = 512, base = [128, 128, 126], seed = 1, contrast = 1, stains = 0.5, cracks = 5, joints = false } = {}) {
  const rng = makeRng(seed);
  const octaves = [[4, 0.34], [8, 0.26], [16, 0.17], [32, 0.09], [64, 0.04]].map(([c, a]) => [makeNoise(rng, c), a]);
  const stain = makeNoise(rng, 3);
  const c = document.createElement('canvas');
  c.width = c.height = size;
  const ctx = c.getContext('2d');
  const img = ctx.createImageData(size, size);
  for (let y = 0; y < size; y++) {
    for (let x = 0; x < size; x++) {
      const u = x / size, v = y / size;
      let n = 0;
      for (const [f, a] of octaves) n += (f(u, v) - 0.5) * a;
      const speck = (rng() - 0.5) * 0.045;
      const st = stains * Math.max(0, stain(u, v) - 0.55) * 1.6;
      const k = 1 + (n * 0.85 + speck) * contrast - st * 0.6;
      const o = (y * size + x) * 4;
      img.data[o] = Math.min(255, Math.max(0, base[0] * k));
      img.data[o + 1] = Math.min(255, Math.max(0, base[1] * k));
      img.data[o + 2] = Math.min(255, Math.max(0, base[2] * k * 0.99));
      img.data[o + 3] = 255;
    }
  }
  ctx.putImageData(img, 0, 0);
  ctx.strokeStyle = 'rgba(20,20,20,0.35)';
  ctx.lineCap = 'round';
  for (let i = 0; i < cracks; i++) { // random-walk hairline cracks
    let x = rng() * size, y = rng() * size, a = rng() * Math.PI * 2;
    ctx.lineWidth = 0.6 + rng() * 0.9;
    ctx.beginPath(); ctx.moveTo(x, y);
    for (let s = 0; s < 40; s++) { a += (rng() - 0.5) * 0.9; x += Math.cos(a) * 7; y += Math.sin(a) * 7; ctx.lineTo(x, y); }
    ctx.stroke();
  }
  if (joints) {
    ctx.strokeStyle = 'rgba(10,10,10,0.55)'; ctx.lineWidth = 3;
    ctx.strokeRect(0, 0, size, size);
    ctx.strokeStyle = 'rgba(255,255,255,0.06)'; ctx.lineWidth = 1;
    ctx.strokeRect(3, 3, size - 6, size - 6);
  }
  const t = new THREE.CanvasTexture(c);
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.colorSpace = THREE.SRGBColorSpace;
  t.anisotropy = 8;
  return t;
}
