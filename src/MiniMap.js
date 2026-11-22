// 2D occupancy-grid minimap (DOM canvas): unknown / free / occupied, planned path, goal and the robot.
import { UNKNOWN, FREE, OCC } from './mapping/OccupancyGrid.js';

const COLORS = { [UNKNOWN]: [14, 20, 32], [FREE]: [58, 74, 98], [OCC]: [235, 240, 250] };

const STATE_LABELS = {
  exploring: 'Exploring', explore: 'Exploring', returning: 'Returning home',
  complete: 'Mapping complete', patrol: 'Patrol', idle: 'Stopped',
};

export function createMiniMap(canvas, statusEl) {
  const ctx = canvas.getContext('2d');
  const off = document.createElement('canvas');
  const offCtx = off.getContext('2d');

  function draw(mapper, pose, path, goal) {
    const { grid } = mapper;
    if (off.width !== grid.W) { off.width = grid.W; off.height = grid.H; }
    const img = offCtx.createImageData(grid.W, grid.H);
    for (let j = 0; j < grid.H; j++) {
      for (let i = 0; i < grid.W; i++) {
        const [r, g, b] = COLORS[grid.stateAt(grid.idx(i, j))];
        const o = ((grid.H - 1 - j) * grid.W + i) * 4; // flip so +Y is up
        img.data[o] = r; img.data[o + 1] = g; img.data[o + 2] = b; img.data[o + 3] = 255;
      }
    }
    offCtx.putImageData(img, 0, 0);
    const cw = canvas.clientWidth, ch = canvas.clientHeight;
    if (canvas.width !== cw || canvas.height !== ch) { canvas.width = cw; canvas.height = ch; }
    ctx.imageSmoothingEnabled = false;
    ctx.clearRect(0, 0, cw, ch);
    const scale = Math.min(cw / grid.W, ch / grid.H);
    const ox = (cw - grid.W * scale) / 2, oy = (ch - grid.H * scale) / 2;
    ctx.drawImage(off, ox, oy, grid.W * scale, grid.H * scale);
    const toPx = ([x, y]) => [
      ox + ((x - grid.bounds.xMin) / grid.res) * scale,
      oy + (grid.H - (y - grid.bounds.yMin) / grid.res) * scale,
    ];

    if (path?.length > 1) {
      ctx.strokeStyle = '#39ff88'; ctx.lineWidth = 1.5; ctx.beginPath();
      path.forEach((p, i) => { const [px, py] = toPx(p); if (i) ctx.lineTo(px, py); else ctx.moveTo(px, py); });
      ctx.stroke();
    }
    if (goal) {
      const [gx, gy] = toPx(goal);
      ctx.strokeStyle = '#ffd60a'; ctx.lineWidth = 1.5;
      ctx.strokeRect(gx - 3, gy - 3, 6, 6);
    }
    const [rx, ry] = toPx([pose.x, pose.y]);
    ctx.save(); ctx.translate(rx, ry); ctx.rotate(-pose.yaw);
    ctx.fillStyle = '#ffcc00'; ctx.beginPath(); ctx.moveTo(6, 0); ctx.lineTo(-4, 4); ctx.lineTo(-4, -4); ctx.closePath(); ctx.fill();
    ctx.restore();

    const s = mapper.stats();
    statusEl.textContent = `${STATE_LABELS[s.state] ?? s.state} · explored ${(s.coverage * 100).toFixed(0)}% · ${(s.points / 1000).toFixed(1)}k pts · ${s.travelled.toFixed(0)} m`;
  }
  return { draw };
}
