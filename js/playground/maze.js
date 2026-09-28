import { isDark, canvas2d, star } from '../util.js';

export function bounds(task) {
  if (!task.layout) return [-7, 7, -7, 7];
  const s = task.scale;
  return [-s / 2, task.layout.length * s - s / 2, -s / 2, task.layout[0].length * s - s / 2];
}

export function freeGoal(task, xy) {
  if (!task.layout) {
    const r = Math.hypot(xy[0], xy[1]);
    return r > 14 ? [xy[0] * 14 / r, xy[1] * 14 / r] : xy;
  }
  const s = task.scale, L = task.layout, i = Math.round(xy[0] / s), j = Math.round(xy[1] / s);
  const free = (a, b) => a >= 0 && b >= 0 && a < L.length && b < L[0].length && L[a][b] !== 1;
  if (free(i, j)) {
    const m = .35 * s;
    let [x, y] = xy;
    if (!free(i - 1, j)) x = Math.max(x, i * s - m);
    if (!free(i + 1, j)) x = Math.min(x, i * s + m);
    if (!free(i, j - 1)) y = Math.max(y, j * s - m);
    if (!free(i, j + 1)) y = Math.min(y, j * s + m);
    return [x, y];
  }
  let best = null, bd = Infinity;
  L.forEach((row, a) => row.forEach((v, b) => {
    const d = Math.hypot(a * s - xy[0], b * s - xy[1]);
    if (v !== 1 && d < bd) { bd = d; best = [a * s, b * s]; }
  }));
  return best;
}

export function drawPoster(cv, task) {
  const { x, W, H } = canvas2d(cv);
  x.fillStyle = isDark() ? '#1f2423' : '#dfe6e4';
  x.fillRect(0, 0, W, H);
  if (!task.layout) return;
  const rows = task.layout.length, cols = task.layout[0].length;
  const cell = Math.min((W - 40) / cols, (H - 40) / rows), ox = (W - cols * cell) / 2, oy = (H - rows * cell) / 2;
  x.fillStyle = isDark() ? '#5d5953' : '#cfc8bc';
  task.layout.forEach((row, i) => row.forEach((v, j) => { if (v === 1) x.fillRect(ox + j * cell, oy + i * cell, cell + .5, cell + .5); }));
}

export class Minimap {
  constructor(cv) { this.cv = cv; }

  transform(task) {
    const W = this.cv.clientWidth, H = this.cv.clientHeight, [x0, x1, y0, y1] = bounds(task);
    const sc = Math.min((W - 12) / (x1 - x0), (H - 12) / (y1 - y0));
    const ox = (W - (x1 - x0) * sc) / 2, oy = (H - (y1 - y0) * sc) / 2;
    return { sc, toPx: (x, y) => [ox + (x - x0) * sc, H - oy - (y - y0) * sc], toWorld: (px, py) => [x0 + (px - ox) / sc, y0 + (H - oy - py) / sc] };
  }

  draw(task, trail, goal, agent) {
    const { x, W, H } = canvas2d(this.cv), tr = this.transform(task), dark = isDark();
    x.clearRect(0, 0, W, H);
    x.fillStyle = dark ? 'rgba(34,38,37,.92)' : 'rgba(245,247,246,.9)';
    x.fillRect(0, 0, W, H);
    if (task.layout) {
      const s = task.scale;
      x.fillStyle = dark ? '#5d5953' : '#cbc3b6';
      task.layout.forEach((row, i) => row.forEach((v, j) => {
        if (v !== 1) return;
        const [a, b] = tr.toPx(i * s - s / 2, j * s + s / 2);
        x.fillRect(a, b, s * tr.sc + .6, s * tr.sc + .6);
      }));
    } else {
      x.strokeStyle = '#d5dcd9';
      const [cx, cy] = tr.toPx(0, 0);
      for (const r of [1, 5]) { x.beginPath(); x.arc(cx, cy, r * tr.sc, 0, 7); x.stroke(); }
    }
    if (trail.count > 1) {
      x.beginPath();
      for (let k = 0; k < trail.count; k++) {
        const [a, b] = tr.toPx(trail.pos[3 * k], trail.pos[3 * k + 1]);
        if (k) x.lineTo(a, b); else x.moveTo(a, b);
      }
      x.strokeStyle = 'rgba(17,145,126,.55)';
      x.lineWidth = 1.5;
      x.stroke();
    }
    const [gx, gy] = tr.toPx(goal[0], goal[1]);
    x.fillStyle = 'rgba(17,145,126,.25)';
    x.beginPath(); x.arc(gx, gy, Math.max(3, .5 * tr.sc), 0, 7); x.fill();
    star(x, gx, gy, 6.5);
    x.fillStyle = dark ? '#5cd0b8' : '#0b6e60';
    x.fill();
    const [ax, ay] = tr.toPx(agent[0], agent[1]);
    x.fillStyle = dark ? '#f4f4f2' : '#1f2a2e';
    x.beginPath(); x.arc(ax, ay, 3.6, 0, 7); x.fill();
    x.strokeStyle = dark ? '#1f2423' : '#fff';
    x.lineWidth = 1.5;
    x.stroke();
  }
}
