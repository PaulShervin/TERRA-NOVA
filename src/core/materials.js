import * as THREE from 'three';
import { concreteTexture } from './textures.js';
import { applyTriplanar } from './triplanar.js';

/** Category colours used by the semantic-segmentation view (and debug labels). */
export const CATEGORY_COLORS = {
  floor: '#8c8c8c', pathway: '#cfcf9a', wall: '#8a3ffc', pillar: '#ffe14a',
  machinery: '#20c8ff', equipment: '#3a8dff', container: '#ff2a2a', rack: '#ff8a1f',
  storage: '#c9a063', fence: '#ff9ae8', barrier: '#ff5fd2', pipe: '#3be0a0',
  platform: '#7dff7a', obstacle: '#b0b0b0', robot: '#ffffff',
};

/** Yellow/black diagonal hazard stripes (pillar collars, barriers). */
function hazardTexture() {
  const c = document.createElement('canvas');
  c.width = c.height = 128;
  const g = c.getContext('2d');
  g.fillStyle = '#e8b81c'; g.fillRect(0, 0, 128, 128);
  g.fillStyle = '#16181a';
  for (let i = -128; i < 256; i += 64) {
    g.beginPath(); g.moveTo(i, 128); g.lineTo(i + 32, 128); g.lineTo(i + 160, 0); g.lineTo(i + 128, 0); g.fill();
  }
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  return t;
}

export function createMaterials() {
  const floorTex = concreteTexture({ base: [118, 118, 114], seed: 7, joints: true, cracks: 9, stains: 0.9 });
  const hazardTex = hazardTexture();
  const wallTex = concreteTexture({ base: [142, 143, 144], seed: 3, cracks: 4, stains: 0.7, contrast: 0.9 });
  const std = (o) => new THREE.MeshStandardMaterial(o);
  const m = {
    floor: applyTriplanar(std({ map: floorTex, roughness: 0.85, metalness: 0.0 }), { tile: 3, bump: 0.25 }),
    pathway: std({ color: '#c9a92a', roughness: 0.8, polygonOffset: true, polygonOffsetFactor: -2 }),
    wall: applyTriplanar(std({ map: wallTex, color: '#b4b7b9', roughness: 0.95 }), { tile: 4, bump: 0.35, dirt: true }),
    concrete: applyTriplanar(std({ map: wallTex, color: '#8f9193', roughness: 0.9 }), { tile: 3, bump: 0.3, dirt: true }),
    pillar: applyTriplanar(std({ map: wallTex, color: '#b06a45', roughness: 0.85 }), { tile: 2.5, bump: 0.3, dirt: true }),
    steel: std({ color: '#5d656c', roughness: 0.45, metalness: 0.85 }),
    darkSteel: std({ color: '#2f3438', roughness: 0.5, metalness: 0.8 }),
    machineBlue: std({ color: '#2c5f96', roughness: 0.55, metalness: 0.5 }),
    machineYellow: std({ color: '#b8932a', roughness: 0.6, metalness: 0.4 }),
    rubber: std({ color: '#1b1d1f', roughness: 0.95 }),
    pipe: std({ color: '#6a7d80', roughness: 0.4, metalness: 0.8 }),
    pipeRed: std({ color: '#8c3328', roughness: 0.5, metalness: 0.6 }),
    fence: std({ color: '#b9bec2', roughness: 0.5, metalness: 0.7 }),
    barrier: std({ color: '#c9531f', roughness: 0.8 }),
    barrierGrey: std({ color: '#9a9c9c', roughness: 0.9 }),
    crate: std({ color: '#7a5a38', roughness: 0.9 }),
    contGreen: std({ color: '#35553f', roughness: 0.65, metalness: 0.4 }),
    contRust: std({ color: '#7a3b2a', roughness: 0.7, metalness: 0.4 }),
    contBlue: std({ color: '#2f4a68', roughness: 0.65, metalness: 0.4 }),
    drum: std({ color: '#4d5f3a', roughness: 0.55, metalness: 0.6 }),
    rubble: std({ color: '#5b5650', roughness: 1.0, flatShading: true }),
    hazard: std({ map: hazardTex, roughness: 0.7 }),
    rackOrange: std({ color: '#d9661a', roughness: 0.55, metalness: 0.5 }),
    rackBlue: std({ color: '#1f4f9c', roughness: 0.55, metalness: 0.5 }),
    toolRed: std({ color: '#b3231c', roughness: 0.5, metalness: 0.4 }),
    door: std({ color: '#3d5560', roughness: 0.55, metalness: 0.6 }),
    wood: std({ color: '#9b7347', roughness: 0.85 }),
    contRed: std({ color: '#a32a22', roughness: 0.6, metalness: 0.4 }),
    ceiling: std({ color: '#2a2d30', roughness: 0.95, side: THREE.DoubleSide }),
    lamp: new THREE.MeshStandardMaterial({ color: '#fff3d0', emissive: '#ffe9b0', emissiveIntensity: 2.0 }),
  };
  m._textures = [floorTex, wallTex, hazardTex];
  return m;
}

export function disposeMaterials(m) {
  m._textures.forEach((t) => t.dispose());
  Object.values(m).forEach((v) => v?.dispose && v.dispose());
}
