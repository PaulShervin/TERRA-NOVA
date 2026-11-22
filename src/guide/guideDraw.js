// Live drawings for the algorithm guide. Each function renders the *current* internal state of the real pipeline.
import { UNKNOWN, FREE, OCC } from '../mapping/OccupancyGrid.js';
import { LIDAR } from '../sensing/Lidar.js';

const BG = '#0b1120';

function prep(canvas) {
  const w = canvas.clientWidth || 460, h = canvas.clientHeight || 300;
  if (canvas.width !== w || canvas.height !== h) { canvas.width = w; canvas.height = h; }
  const ctx = canvas.getContext('2d');
  ctx.fillStyle = BG; ctx.fillRect(0, 0, w, h);
  return { ctx, w, h };
}

/** Map a world (x,y) to canvas pixels for a full-grid view. */
function gridView(grid, w, h) {
  const scale = Math.min(w / grid.W, h / grid.H);
  const ox = (w - grid.W * scale) / 2, oy = (h - grid.H * scale) / 2;
  return {
    scale, ox, oy,
    px: ([x, y]) => [ox + ((x - grid.bounds.xMin) / grid.res) * scale, oy + (grid.H - (y - grid.bounds.yMin) / grid.res) * scale],
    cell: (k) => [ox + (k % grid.W) * scale, oy + (grid.H - 1 - ((k / grid.W) | 0)) * scale],
  };
}

function paintGrid(ctx, grid, view, colorOf) {
  for (let k = 0; k < grid.W * grid.H; k++) {
    const c = colorOf(k);
    if (!c) continue;
    const [x, y] = view.cell(k);
    ctx.fillStyle = c;
    ctx.fillRect(x, y, Math.ceil(view.scale), Math.ceil(view.scale));
  }
}

function robotMarker(ctx, x, y, yaw, size = 7) {
  ctx.save(); ctx.translate(x, y); ctx.rotate(-yaw);
  ctx.fillStyle = '#ffcc00'; ctx.beginPath(); ctx.moveTo(size, 0); ctx.lineTo(-size * 0.7, size * 0.7); ctx.lineTo(-size * 0.7, -size * 0.7); ctx.closePath(); ctx.fill();
  ctx.restore();
}

function star(ctx, x, y, r, color) {
  ctx.strokeStyle = color; ctx.lineWidth = 2; ctx.beginPath();
  for (let i = 0; i < 4; i++) { const a = (i * Math.PI) / 4; ctx.moveTo(x - Math.cos(a) * r, y - Math.sin(a) * r); ctx.lineTo(x + Math.cos(a) * r, y + Math.sin(a) * r); }
  ctx.stroke();
}

function pathLine(ctx, view, path, color = '#39ff88') {
  if (!path || path.length < 2) return;
  ctx.strokeStyle = color; ctx.lineWidth = 2; ctx.beginPath();
  path.forEach((p, i) => { const [x, y] = view.px(p); if (i) ctx.lineTo(x, y); else ctx.moveTo(x, y); });
  ctx.stroke();
}

function label(ctx, text, x, y, color = '#cfe0ff') {
  ctx.fillStyle = color; ctx.font = '12px Segoe UI, sans-serif'; ctx.fillText(text, x, y);
}

// ---- 1. LiDAR: polar top view (beam closest to horizontal) + side view of the 16 beams ----
export function drawLidar(canvas, mapper) {
  const { ctx, w, h } = prep(canvas);
  const s = mapper.lastScan;
  if (!s) { label(ctx, 'waiting for first sweep...', 20, 30); return; }
  const azCount = Math.round(360 / LIDAR.azStepDeg);
  const half = w * 0.5;
  const cx = half * 0.5, cy = h * 0.52, R = Math.min(half * 0.46, h * 0.42);
  label(ctx, 'Top view: horizontal beam', 10, 16);
  ctx.strokeStyle = '#223454'; ctx.lineWidth = 1;
  for (const r of [5, 10, 15, 20, 25]) { ctx.beginPath(); ctx.arc(cx, cy, (r / LIDAR.maxRange) * R, 0, Math.PI * 2); ctx.stroke(); label(ctx, `${r} m`, cx + (r / LIDAR.maxRange) * R * 0.71, cy - (r / LIDAR.maxRange) * R * 0.71, '#4f6a94'); }
  const mid = Math.round((0 - LIDAR.elevMinDeg) / ((LIDAR.elevMaxDeg - LIDAR.elevMinDeg) / (LIDAR.beams - 1)));
  for (let a = 0; a < azCount; a++) {
    const r = s.rays[mid * azCount + a];
    const ang = s.pose.yaw + (a * LIDAR.azStepDeg * Math.PI) / 180;
    const rr = (Math.min(r.range * Math.hypot(r.dx, r.dy), LIDAR.maxRange) / LIDAR.maxRange) * R;
    ctx.strokeStyle = r.hit ? 'rgba(130,100,255,0.35)' : 'rgba(80,90,120,0.2)';
    ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(cx + Math.cos(ang) * rr, cy - Math.sin(ang) * rr); ctx.stroke();
    if (r.hit) { ctx.fillStyle = '#7fe3ff'; ctx.fillRect(cx + Math.cos(ang) * rr - 1, cy - Math.sin(ang) * rr - 1, 2.2, 2.2); }
  }
  robotMarker(ctx, cx, cy, s.pose.yaw, 6);
  // side view: all 16 beams at azimuth index 0 (straight ahead)
  const sx = half + 20, sy = h * 0.55, sw = half - 40, hs = sw / 14;
  label(ctx, 'Side view: 16 beams (-15..+15 deg)', half + 10, 34);
  ctx.strokeStyle = '#223454'; ctx.beginPath(); ctx.moveTo(sx, sy + 0.72 * 1.35 * hs); ctx.lineTo(sx + sw, sy + 0.72 * 1.35 * hs); ctx.stroke();
  for (let b = 0; b < LIDAR.beams; b++) {
    const r = s.rays[b * azCount + 0];
    const horiz = r.range * Math.hypot(r.dx, r.dy), vert = r.range * r.dz;
    const x1 = sx + Math.min(horiz, 14) * hs, y1 = sy - vert * hs;
    ctx.strokeStyle = r.hit ? 'rgba(255,120,200,0.8)' : 'rgba(120,130,160,0.4)';
    ctx.beginPath(); ctx.moveTo(sx, sy); ctx.lineTo(x1, y1); ctx.stroke();
    if (r.hit && horiz < 14) { ctx.fillStyle = '#fff'; ctx.fillRect(x1 - 1.5, y1 - 1.5, 3, 3); }
  }
  ctx.fillStyle = '#ffcc00'; ctx.fillRect(sx - 6, sy - 3, 6, 6);
  label(ctx, `${LIDAR.beams} beams x ${azCount} az = ${LIDAR.beams * azCount} rays / sweep`, half + 10, h - 30);
  label(ctx, `${s.hits} returns, noise sigma ${LIDAR.noiseSigma * 100} cm, max ${LIDAR.maxRange} m`, half + 10, h - 12);
}

// ---- 2. Occupancy grid: log-odds heat map ----
export function drawOccupancy(canvas, mapper, pose) {
  const { ctx, w, h } = prep(canvas);
  const { grid } = mapper;
  const view = gridView(grid, w, h);
  paintGrid(ctx, grid, view, (k) => {
    const st = grid.stateAt(k);
    if (!grid.seen[k]) return null;
    const lo = grid.logOdds[k];
    if (st === OCC) return `rgb(255,${Math.round(255 - Math.min(lo, 5) * 30)},${Math.round(240 - Math.min(lo, 5) * 45)})`;
    if (st === FREE) return `rgb(20,${Math.round(70 + Math.min(-lo, 2) * 40)},${Math.round(130 + Math.min(-lo, 2) * 55)})`;
    return 'rgb(70,70,90)';
  });
  const [rx, ry] = view.px([pose.x, pose.y]);
  robotMarker(ctx, rx, ry, pose.yaw);
  label(ctx, 'white = occupied (log-odds high)   blue = free (log-odds low)   grey = uncertain   black = never seen', 8, h - 8, '#8fa6c8');
}

// ---- 3. Frontiers ----
export function drawFrontiers(canvas, mapper, pose) {
  const { ctx, w, h } = prep(canvas);
  const { grid } = mapper;
  const view = gridView(grid, w, h);
  paintGrid(ctx, grid, view, (k) => { const s = grid.stateAt(k); return s === OCC ? '#8d99ad' : s === FREE ? '#17314f' : null; });
  const dbg = mapper.explorer.getDebug();
  if (dbg) {
    dbg.clusters.forEach((cells, i) => {
      ctx.fillStyle = `hsl(${(i * 67) % 360},90%,60%)`;
      for (const k of cells) { const [x, y] = view.cell(k); ctx.fillRect(x - 1, y - 1, Math.ceil(view.scale) + 2, Math.ceil(view.scale) + 2); }
    });
    if (dbg.goalCell != null) { const [gx, gy] = view.cell(dbg.goalCell); star(ctx, gx, gy, 10, '#ffffff'); }
  }
  const [rx, ry] = view.px([pose.x, pose.y]);
  robotMarker(ctx, rx, ry, pose.yaw);
  label(ctx, `${dbg ? dbg.clusters.length : 0} frontier clusters (colours)   white cross = chosen goal`, 8, h - 8, '#8fa6c8');
}

// ---- 4. Planning: Dijkstra cost field + inflated obstacles + path ----
export function drawPlanning(canvas, mapper, pose) {
  const { ctx, w, h } = prep(canvas);
  const { grid } = mapper;
  const view = gridView(grid, w, h);
  const dbg = mapper.explorer.getDebug();
  paintGrid(ctx, grid, view, (k) => {
    const st = grid.stateAt(k);
    if (st === OCC) return '#f1f5ff';
    if (st !== FREE) return null;
    if (dbg && dbg.trav[k] && Number.isFinite(dbg.cost[k])) return `hsl(${Math.max(0, 240 - dbg.cost[k] * 4)},75%,45%)`;
    return '#5a1d28'; // free but inside the inflation radius (robot would not fit)
  });
  pathLine(ctx, view, mapper.explorer.getPath());
  const g = mapper.explorer.getGoal();
  if (g) { const [gx, gy] = view.px(g); star(ctx, gx, gy, 10, '#fff'); }
  const [rx, ry] = view.px([pose.x, pose.y]);
  robotMarker(ctx, rx, ry, pose.yaw);
  label(ctx, `colour = path cost from robot (blue near -> red far)   dark red = too close to obstacles (r = ${dbg ? dbg.inflation : '-'} m)`, 8, h - 8, '#8fa6c8');
}

// ---- 5. Control: zoomed view of the robot, its next waypoint and the heading error ----
export function drawControl(canvas, driver) {
  const { ctx, w, h } = prep(canvas);
  const scale = 38; // px per metre
  const cx = w * 0.4, cy = h * 0.55;
  const p = driver.pose;
  const px = (x, y) => [cx + (x - p.x) * scale, cy - (y - p.y) * scale];
  ctx.strokeStyle = '#162441'; ctx.lineWidth = 1;
  for (let i = -8; i <= 8; i++) { ctx.beginPath(); ctx.moveTo(cx + i * scale, 0); ctx.lineTo(cx + i * scale, h); ctx.moveTo(0, cy + i * scale); ctx.lineTo(w, cy + i * scale); ctx.stroke(); }
  const route = driver.getRoute();
  if (route.length > 1) { ctx.strokeStyle = '#39ff88'; ctx.lineWidth = 2; ctx.beginPath(); route.forEach(([x, y], i) => { const [a, b] = px(x, y); if (i) ctx.lineTo(a, b); else ctx.moveTo(a, b); }); ctx.stroke(); }
  const tgt = driver.getTarget();
  if (tgt) {
    const [tx, ty] = px(tgt[0], tgt[1]);
    ctx.strokeStyle = '#ff6bd6'; ctx.setLineDash([5, 4]); ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(tx, ty); ctx.stroke(); ctx.setLineDash([]);
    ctx.fillStyle = '#ff6bd6'; ctx.beginPath(); ctx.arc(tx, ty, 5, 0, Math.PI * 2); ctx.fill();
    const desired = Math.atan2(tgt[1] - p.y, tgt[0] - p.x);
    const err = Math.atan2(Math.sin(desired - p.yaw), Math.cos(desired - p.yaw));
    ctx.strokeStyle = '#ffd60a'; ctx.beginPath(); ctx.arc(cx, cy, 34, -p.yaw, -p.yaw - err, err > 0); ctx.stroke();
    label(ctx, `heading error = ${(err * 57.2958).toFixed(0)} deg`, 12, 20, '#ffd60a');
  }
  robotMarker(ctx, cx, cy, p.yaw, 12);
  const vl = p.v - p.w * 0.29 * 1.35, vr = p.v + p.w * 0.29 * 1.35;
  const bx = w * 0.78;
  label(ctx, `v = ${p.v.toFixed(2)} m/s`, bx, 40); label(ctx, `w = ${p.w.toFixed(2)} rad/s`, bx, 58);
  label(ctx, `left wheel  ${vl.toFixed(2)} m/s`, bx, 90); label(ctx, `right wheel ${vr.toFixed(2)} m/s`, bx, 108);
  ctx.fillStyle = '#3a7bd5'; ctx.fillRect(bx, 118, Math.max(0, vl) * 40, 8); ctx.fillStyle = '#d55a3a'; ctx.fillRect(bx, 132, Math.max(0, vr) * 40, 8);
}
