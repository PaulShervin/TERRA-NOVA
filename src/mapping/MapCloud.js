// Growing 3D point-cloud map: voxel-deduplicated, append-only GPU buffers, switchable height / semantic colouring.
import * as THREE from 'three';
import { heightColor, categoryColor } from '../core/colors.js';

const VOXEL = 0.12;
const CAPACITY = 900000;

export function createMapCloud() {
  const pos = new Float32Array(CAPACITY * 3);
  const colH = new Float32Array(CAPACITY * 3);
  const colS = new Float32Array(CAPACITY * 3);
  const attrPos = new THREE.BufferAttribute(pos, 3).setUsage(THREE.DynamicDrawUsage);
  const attrH = new THREE.BufferAttribute(colH, 3).setUsage(THREE.DynamicDrawUsage);
  const attrS = new THREE.BufferAttribute(colS, 3).setUsage(THREE.DynamicDrawUsage);
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', attrPos);
  geo.setAttribute('color', attrH);
  geo.setDrawRange(0, 0);
  geo.boundingSphere = new THREE.Sphere(new THREE.Vector3(), 200); // fixed: skip per-update recompute

  const seen = new Set();
  let count = 0;
  let dirtyFrom = Infinity; // first point index not yet uploaded to the GPU
  const c = new THREE.Color();

  function add(hits) {
    const start = count;
    for (const [x, y, z, cat] of hits) {
      if (count >= CAPACITY || z < -0.05) continue;
      const key = ((Math.floor(x / VOXEL) + 1024) * 2048 + (Math.floor(y / VOXEL) + 1024)) * 256 + Math.floor(Math.max(z, 0) / VOXEL);
      if (seen.has(key)) continue;
      seen.add(key);
      const o = count * 3;
      pos[o] = x; pos[o + 1] = z; pos[o + 2] = -y; // ROS -> three
      heightColor(z, c); colH[o] = c.r; colH[o + 1] = c.g; colH[o + 2] = c.b;
      categoryColor(cat, c); colS[o] = c.r; colS[o + 1] = c.g; colS[o + 2] = c.b;
      count++;
    }
    const added = count - start;
    if (added > 0) { dirtyFrom = Math.min(dirtyFrom, start); geo.setDrawRange(0, count); }
    return added;
  }

  /** Upload everything added since the last flush as ONE range (call once per rendered frame). */
  function flush() {
    if (dirtyFrom >= count) { dirtyFrom = Infinity; return; }
    for (const a of [attrPos, attrH, attrS]) { a.clearUpdateRanges(); a.addUpdateRange(dirtyFrom * 3, (count - dirtyFrom) * 3); a.needsUpdate = true; }
    dirtyFrom = Infinity;
  }

  const makePoints = (size, depthTest, opacity = 1) => {
    // Hide ceiling returns (z ~ 6.6 m) in the viewers so the interior stays visible; they stay in the stored map.
    const mat = new THREE.PointsMaterial({
      size, sizeAttenuation: false, vertexColors: true, depthTest, transparent: opacity < 1, opacity, toneMapped: false,
      clippingPlanes: [new THREE.Plane(new THREE.Vector3(0, -1, 0), 6.2)],
    });
    const p = new THREE.Points(geo, mat);
    p.frustumCulled = false;
    return p;
  };

  return {
    add, flush,
    count: () => count,
    /** Two renderable views of the same buffers: one for the inset scene, one overlaid on the main scene. */
    insetPoints: makePoints(2.2, false),
    mainPoints: makePoints(1.7, true, 0.95),
    setColorMode(mode) { geo.setAttribute('color', mode === 'semantic' ? attrS : attrH); },
    clear() { seen.clear(); count = 0; dirtyFrom = Infinity; geo.setDrawRange(0, 0); },
    dispose() { geo.dispose(); this.insetPoints.material.dispose(); this.mainPoints.material.dispose(); },
  };
}
