// Grid planning on the occupancy map: obstacle inflation, Dijkstra, frontier clustering, path smoothing.
import { FREE, OCC, UNKNOWN } from './OccupancyGrid.js';

const SQRT2 = Math.SQRT2;
const N8 = [[1, 0, 1], [-1, 0, 1], [0, 1, 1], [0, -1, 1], [1, 1, SQRT2], [1, -1, SQRT2], [-1, 1, SQRT2], [-1, -1, SQRT2]];

/** traversable[k] = 1 when known-free and farther than `radius` metres from any occupied cell. */
export function buildTraversable(grid, radius, keepClear) {
  const { W, H, res } = grid;
  const R = Math.max(1, Math.round(radius / res));
  const offsets = [];
  for (let dj = -R; dj <= R; dj++) for (let di = -R; di <= R; di++) if (di * di + dj * dj <= R * R) offsets.push([di, dj]);
  const blocked = new Uint8Array(W * H);
  for (let j = 0; j < H; j++) {
    for (let i = 0; i < W; i++) {
      if (grid.stateAt(grid.idx(i, j)) !== OCC) continue;
      for (const [di, dj] of offsets) {
        const ni = i + di, nj = j + dj;
        if (ni >= 0 && nj >= 0 && ni < W && nj < H) blocked[grid.idx(ni, nj)] = 1;
      }
    }
  }
  const trav = new Uint8Array(W * H);
  for (let k = 0; k < W * H; k++) trav[k] = grid.stateAt(k) === FREE && !blocked[k] ? 1 : 0;
  if (keepClear) { // always allow the robot's own neighbourhood (sensor noise can mark it blocked)
    const [ci, cj] = grid.toCell(keepClear[0], keepClear[1]);
    const r = Math.ceil(0.35 / res);
    for (let dj = -r; dj <= r; dj++) for (let di = -r; di <= r; di++) {
      const ni = ci + di, nj = cj + dj;
      if (grid.inside(ni, nj) && grid.stateAt(grid.idx(ni, nj)) !== OCC) trav[grid.idx(ni, nj)] = 1;
    }
  }
  return trav;
}

class MinHeap {
  constructor() { this.k = []; this.v = []; }
  get size() { return this.k.length; }
  peekKey() { return this.k[0]; }
  push(key, val) {
    const { k, v } = this; let i = k.length; k.push(key); v.push(val);
    while (i > 0) { const p = (i - 1) >> 1; if (k[p] <= k[i]) break; [k[p], k[i]] = [k[i], k[p]]; [v[p], v[i]] = [v[i], v[p]]; i = p; }
  }
  pop() {
    const { k, v } = this; const top = v[0]; const lk = k.pop(), lv = v.pop();
    if (k.length) {
      k[0] = lk; v[0] = lv; let i = 0;
      for (;;) {
        const l = 2 * i + 1, r = l + 1; let m = i;
        if (l < k.length && k[l] < k[m]) m = l; if (r < k.length && k[r] < k[m]) m = r;
        if (m === i) break; [k[m], k[i]] = [k[i], k[m]]; [v[m], v[i]] = [v[i], v[m]]; i = m;
      }
    }
    return top;
  }
}

/** Dijkstra from the start cell over traversable cells. Returns cost (metres) and parent arrays. */
export function dijkstra(grid, trav, start) {
  const { W, H, res } = grid;
  const cost = new Float64Array(W * H).fill(Infinity);
  const parent = new Int32Array(W * H).fill(-1);
  const heap = new MinHeap();
  const s = grid.idx(start[0], start[1]);
  cost[s] = 0; heap.push(0, s);
  while (heap.size) {
    const key = heap.peekKey();
    const k = heap.pop();
    const i = k % W, j = (k / W) | 0;
    const c = cost[k];
    if (key > c + 1e-9) continue; // stale heap entry
    for (const [di, dj, w] of N8) {
      const ni = i + di, nj = j + dj;
      if (ni < 0 || nj < 0 || ni >= W || nj >= H) continue;
      const nk = nj * W + ni;
      if (!trav[nk]) continue;
      if (di && dj && (!trav[j * W + ni] || !trav[nj * W + i])) continue; // no corner cutting
      const nc = c + w * res;
      if (nc < cost[nk]) { cost[nk] = nc; parent[nk] = k; heap.push(nc, nk); }
    }
  }
  return { cost, parent };
}

/** Frontier cells: traversable cells with an unknown 4-neighbour, grouped into 8-connected clusters. */
export function findFrontiers(grid, trav, minSize = 5) {
  const { W, H } = grid;
  const isFrontier = new Uint8Array(W * H);
  for (let j = 1; j < H - 1; j++) {
    for (let i = 1; i < W - 1; i++) {
      const k = j * W + i;
      if (!trav[k]) continue;
      if (grid.stateAt(k - 1) === UNKNOWN || grid.stateAt(k + 1) === UNKNOWN || grid.stateAt(k - W) === UNKNOWN || grid.stateAt(k + W) === UNKNOWN) isFrontier[k] = 1;
    }
  }
  const visited = new Uint8Array(W * H);
  const clusters = [];
  for (let k0 = 0; k0 < W * H; k0++) {
    if (!isFrontier[k0] || visited[k0]) continue;
    const cells = [], stack = [k0]; visited[k0] = 1;
    while (stack.length) {
      const k = stack.pop(); cells.push(k);
      const i = k % W, j = (k / W) | 0;
      for (const [di, dj] of N8) {
        const ni = i + di, nj = j + dj;
        if (ni < 0 || nj < 0 || ni >= W || nj >= H) continue;
        const nk = nj * W + ni;
        if (isFrontier[nk] && !visited[nk]) { visited[nk] = 1; stack.push(nk); }
      }
    }
    if (cells.length >= minSize) clusters.push(cells);
  }
  return clusters;
}

/** Walk parents back from `goal` to the start and return grid cells [i,j] start->goal. */
export function extractPath(grid, parent, goalK) {
  const out = [];
  for (let k = goalK; k !== -1; k = parent[k]) out.push([k % grid.W, (k / grid.W) | 0]);
  return out.reverse();
}

function lineFree(grid, trav, a, b) {
  const n = Math.ceil(Math.hypot(b[0] - a[0], b[1] - a[1]) * 2);
  for (let s = 0; s <= n; s++) {
    const i = Math.round(a[0] + ((b[0] - a[0]) * s) / n), j = Math.round(a[1] + ((b[1] - a[1]) * s) / n);
    if (!trav[grid.idx(i, j)]) return false;
  }
  return true;
}

/** String-pulling: drop intermediate cells whenever a straight line between survivors stays traversable. */
export function smoothPath(grid, trav, cells) {
  if (cells.length < 3) return cells;
  const out = [cells[0]];
  let anchor = 0;
  for (let i = 2; i < cells.length; i++) {
    if (!lineFree(grid, trav, cells[anchor], cells[i])) { out.push(cells[i - 1]); anchor = i - 1; }
  }
  out.push(cells[cells.length - 1]);
  return out;
}
