// Naive Surface Nets over the cave field -> triangle mesh (ROS axes), plus conversion to Three.js geometry with a
// separately-hideable roof (triangles facing down). Pure array code except the Three conversion at the bottom.
import * as THREE from 'three';

const CORNERS = [[0, 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 0], [0, 0, 1], [1, 0, 1], [0, 1, 1], [1, 1, 1]];
const EDGES = [[0, 1], [2, 3], [4, 5], [6, 7], [0, 2], [1, 3], [4, 6], [5, 7], [0, 4], [1, 5], [2, 6], [3, 7]];

export function extractSurface(field) {
  const { nx, ny, nz, voxel, data } = field;
  const cellIdx = new Int32Array(nx * ny * nz).fill(-1);
  const pos = [];
  const at = (i, j, k) => (k * ny + j) * nx + i;

  // 1. one vertex per cell that straddles the surface (mean of its edge crossings)
  const v = new Float32Array(8);
  for (let k = 0; k < nz - 1; k++) {
    for (let j = 0; j < ny - 1; j++) {
      for (let i = 0; i < nx - 1; i++) {
        let neg = 0;
        for (let c = 0; c < 8; c++) { v[c] = data[at(i + CORNERS[c][0], j + CORNERS[c][1], k + CORNERS[c][2])]; if (v[c] < 0) neg++; }
        if (neg === 0 || neg === 8) continue;
        let sx = 0, sy = 0, sz = 0, n = 0;
        for (const [a, b] of EDGES) {
          if ((v[a] < 0) === (v[b] < 0)) continue;
          const t = v[a] / (v[a] - v[b]);
          sx += CORNERS[a][0] + (CORNERS[b][0] - CORNERS[a][0]) * t;
          sy += CORNERS[a][1] + (CORNERS[b][1] - CORNERS[a][1]) * t;
          sz += CORNERS[a][2] + (CORNERS[b][2] - CORNERS[a][2]) * t;
          n++;
        }
        cellIdx[at(i, j, k)] = pos.length / 3;
        pos.push(field.ox + (i + sx / n) * voxel, field.oy + (j + sy / n) * voxel, field.oz + (k + sz / n) * voxel);
      }
    }
  }

  // 2. one quad per sign-changing grid edge, joining the 4 cells that share it
  const idx = [];
  const quad = (a, b, c, d, axisDir) => {
    if (a < 0 || b < 0 || c < 0 || d < 0) return;
    // orient so the geometric normal points toward the air (decreasing field) i.e. against axisDir
    const ax = pos[a * 3], ay = pos[a * 3 + 1], az = pos[a * 3 + 2];
    const ux = pos[b * 3] - ax, uy = pos[b * 3 + 1] - ay, uz = pos[b * 3 + 2] - az;
    const wx = pos[c * 3] - ax, wy = pos[c * 3 + 1] - ay, wz = pos[c * 3 + 2] - az;
    const nxv = uy * wz - uz * wy, nyv = uz * wx - ux * wz, nzv = ux * wy - uy * wx;
    const dot = nxv * axisDir[0] + nyv * axisDir[1] + nzv * axisDir[2];
    if (dot > 0) idx.push(a, c, b, a, d, c); else idx.push(a, b, c, a, c, d);
  };
  for (let k = 1; k < nz - 1; k++) {
    for (let j = 1; j < ny - 1; j++) {
      for (let i = 1; i < nx - 1; i++) {
        const here = data[at(i, j, k)] < 0;
        if (here !== (data[at(i + 1, j, k)] < 0)) quad(cellIdx[at(i, j - 1, k - 1)], cellIdx[at(i, j, k - 1)], cellIdx[at(i, j, k)], cellIdx[at(i, j - 1, k)], here ? [1, 0, 0] : [-1, 0, 0]);
        if (here !== (data[at(i, j + 1, k)] < 0)) quad(cellIdx[at(i - 1, j, k - 1)], cellIdx[at(i, j, k - 1)], cellIdx[at(i, j, k)], cellIdx[at(i - 1, j, k)], here ? [0, 1, 0] : [0, -1, 0]);
        if (here !== (data[at(i, j, k + 1)] < 0)) quad(cellIdx[at(i - 1, j - 1, k)], cellIdx[at(i, j - 1, k)], cellIdx[at(i, j, k)], cellIdx[at(i - 1, j, k)], here ? [0, 0, 1] : [0, 0, -1]);
      }
    }
  }

  // 3. normals from the field gradient (smooth shading), pointing into the air
  const positions = new Float32Array(pos);
  const normals = new Float32Array(pos.length);
  for (let p = 0; p < positions.length; p += 3) {
    const g = field.gradient(positions[p], positions[p + 1], positions[p + 2], voxel * 0.6);
    normals[p] = -g[0]; normals[p + 1] = -g[1]; normals[p + 2] = -g[2];
  }
  return { positions, normals, indices: new Uint32Array(idx) };
}

/**
 * Three.js geometry in Y-up coordinates (x, z, -y). The index buffer is split in two:
 * `body` (floor + walls) and `roof` (triangles whose normal points down) so the roof can be hidden.
 */
export function toThreeGeometries(mesh, colors) {
  const { positions, normals, indices } = mesh;
  const n = positions.length / 3;
  const pos = new Float32Array(n * 3), nor = new Float32Array(n * 3);
  for (let i = 0; i < n; i++) {
    pos[i * 3] = positions[i * 3]; pos[i * 3 + 1] = positions[i * 3 + 2]; pos[i * 3 + 2] = -positions[i * 3 + 1];
    nor[i * 3] = normals[i * 3]; nor[i * 3 + 1] = normals[i * 3 + 2]; nor[i * 3 + 2] = -normals[i * 3 + 1];
  }
  const body = [], roof = [];
  for (let t = 0; t < indices.length; t += 3) {
    const a = indices[t], b = indices[t + 1], c = indices[t + 2];
    const nzRos = (normals[a * 3 + 2] + normals[b * 3 + 2] + normals[c * 3 + 2]) / 3; // ROS z of the normal
    (nzRos < -0.3 ? roof : body).push(a, b, c);
  }
  const base = new THREE.BufferGeometry();
  base.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  base.setAttribute('normal', new THREE.BufferAttribute(nor, 3));
  const colorAttr = new THREE.BufferAttribute(new Float32Array(colors ?? n * 3).fill(1), 3);
  if (colors) colorAttr.array.set(colors);
  base.setAttribute('color', colorAttr);
  const bodyGeo = base.clone(); bodyGeo.setIndex(body);
  const roofGeo = base.clone(); roofGeo.setIndex(roof);
  // both geometries must share the SAME color attribute object so a single update tints both
  roofGeo.setAttribute('color', bodyGeo.getAttribute('color'));
  base.dispose();
  for (const g of [bodyGeo, roofGeo]) g.computeBoundingSphere();
  return { bodyGeo, roofGeo, vertexCount: n, triangles: { body: body.length / 3, roof: roof.length / 3 } };
}
