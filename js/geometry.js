import { fetchJSON, el, segmented, tooltip, tealRamp, isDark, onThemeChange, css, fmt, canvas2d, star } from './util.js';

const MEASURES = [
  { value: 'space', label: 'Distance in space', title: 'Distance in space', unit: 'm' },
  { value: 'chronosrl', label: 'ChronoSRL', title: "ChronoSRL's distance", unit: 'steps' },
  { value: 'without_geometric_losses', label: 'Without the geometric losses', title: 'Distance without the geometric losses', unit: 'steps' },
];
const PAD = { l: 46, r: 10, t: 10, b: 32 };
const CELL = 6;

const format = (v, unit) => (unit === 'm' ? `${v.toFixed(v < 10 ? 1 : 0)} m` : `${fmt.steps(v)} steps`);

function ranks(v) {
  const idx = v.map((x, i) => i).sort((a, b) => v[a] - v[b]), r = new Array(v.length);
  for (let i = 0; i < idx.length;) {
    let j = i;
    while (j + 1 < idx.length && v[idx[j + 1]] === v[idx[i]]) j++;
    for (let k = i; k <= j; k++) r[idx[k]] = (i + j) / 2;
    i = j + 1;
  }
  return r;
}

function spearman(a, b) {
  const ra = ranks(a), rb = ranks(b), n = a.length, m = (n - 1) / 2;
  let s = 0, sa = 0, sb = 0;
  for (let i = 0; i < n; i++) {
    s += (ra[i] - m) * (rb[i] - m);
    sa += (ra[i] - m) ** 2;
    sb += (rb[i] - m) ** 2;
  }
  return s / Math.sqrt(sa * sb);
}

export async function init() {
  const mz = await fetchJSON('assets/data/maze.json');
  const nx = mz.nx, ny = mz.ny, steps = mz.maps.steps;
  const [gi, gj] = mz.goal, gx = gi * mz.scale, gy = gj * mz.scale;
  const space = steps.map((s, k) => {
    if (s < 0) return -1;
    const a = Math.floor(k / ny), b = k % ny;
    return Math.hypot(-mz.scale / 2 + (a + .5) * mz.res - gx, -mz.scale / 2 + (b + .5) * mz.res - gy);
  });
  const values = { space, chronosrl: mz.maps.chronosrl, without_geometric_losses: mz.maps.without_geometric_losses };
  const idx = [];
  for (let k = 0; k < steps.length; k++) if (steps[k] >= 0 && MEASURES.every(m => values[m.value][k] >= 0)) idx.push(k);
  const rho = Object.fromEntries(MEASURES.map(m => [m.value, spearman(idx.map(k => values[m.value][k]), idx.map(k => steps[k]))]));

  let measure = 'chronosrl', hover = null;
  const host = document.getElementById('maze-maps');
  const seg = segmented(MEASURES.map(m => ({ value: m.value, label: m.label })), measure, (v) => { measure = v; setMeasure(); }, { label: 'Distance shown' });
  document.getElementById('maze-measure').replaceWith(seg.el);
  seg.el.id = 'maze-measure';
  host.replaceChildren();
  const tip = tooltip(host);

  const mapCell = (title) => {
    const cv = el('canvas', { width: ny * CELL, height: nx * CELL, 'aria-label': title });
    const h = el('h4', { text: title }), lo = el('span'), hi = el('span'), ramp = el('i');
    host.append(el('div', { class: 'map-cell' }, h, cv, el('div', { class: 'scale' }, lo, ramp, hi)));
    return { cv, h, lo, hi, ramp };
  };
  const stepsMap = mapCell('Steps until the goal'), distMap = mapCell('');
  const scatterTitle = el('h4'), rhoText = el('b', { class: 'rho' });
  scatterTitle.append(el('span', { text: 'Distance against the steps, per cell' }), rhoText);
  const sc = el('canvas', { 'aria-label': 'Scatter of the selected distance against the steps until the goal, per cell' });
  host.append(el('div', { class: 'map-cell scatter-cell' }, scatterTitle, sc));

  const range = (vals) => {
    const pos = idx.map(k => vals[k]).filter(v => v > 0).sort((a, b) => a - b);
    return [Math.max(pos[0], pos[Math.floor(pos.length * .05)]), pos[Math.floor(pos.length * .95)]];
  };
  const stepsRange = range(steps);
  const current = () => MEASURES.find(x => x.value === measure);

  function setMeasure() {
    const m = current();
    distMap.h.textContent = m.title;
    distMap.cv.setAttribute('aria-label', m.title);
    rhoText.textContent = `rank correlation ${rho[measure].toFixed(2)}`;
    draw(hover);
  }

  function drawMap(c, vals, [lo, hi], unit, h, dark) {
    const ctx = c.cv.getContext('2d'), s = CELL;
    ctx.fillStyle = dark ? '#2b2b29' : '#f1f0ec';
    ctx.fillRect(0, 0, c.cv.width, c.cv.height);
    const L = Math.log(lo), R = Math.log(hi);
    for (const k of idx) {
      const i = Math.floor(k / ny), j = k % ny;
      ctx.fillStyle = tealRamp((Math.log(Math.max(vals[k], 1e-3)) - L) / (R - L), dark);
      ctx.fillRect(j * s, i * s, s, s);
    }
    const w = mz.scale / mz.res;
    ctx.fillStyle = dark ? '#555550' : '#c9c8c2';
    mz.layout.forEach((row, i) => row.forEach((v, j) => { if (v === 1) ctx.fillRect(j * w * s, i * w * s, w * s, w * s); }));
    star(ctx, (gj * w + w / 2) * s, (gi * w + w / 2) * s, 12);
    ctx.fillStyle = dark ? '#fff' : '#111';
    ctx.fill();
    ctx.lineWidth = 1.5;
    ctx.strokeStyle = 'rgba(255,255,255,.9)';
    ctx.stroke();
    if (h) {
      ctx.strokeStyle = dark ? 'rgba(255,255,255,.55)' : 'rgba(0,0,0,.45)';
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(0, h.i * s + s / 2);
      ctx.lineTo(c.cv.width, h.i * s + s / 2);
      ctx.moveTo(h.j * s + s / 2, 0);
      ctx.lineTo(h.j * s + s / 2, c.cv.height);
      ctx.stroke();
      ctx.strokeStyle = dark ? '#fff' : '#000';
      ctx.lineWidth = 2.5;
      ctx.strokeRect(h.j * s - 2, h.i * s - 2, s + 4, s + 4);
    }
    c.lo.textContent = unit === 'm' ? `${lo.toFixed(1)}` : fmt.steps(lo);
    c.hi.textContent = format(hi, unit);
    c.ramp.style.background = `linear-gradient(90deg, ${tealRamp(0, dark)}, ${tealRamp(.5, dark)}, ${tealRamp(1, dark)})`;
  }

  function axes() {
    const W = sc.clientWidth, H = sc.clientHeight, ys = values[measure];
    let xm = 0, ym = 0;
    for (const k of idx) { xm = Math.max(xm, steps[k]); ym = Math.max(ym, ys[k]); }
    xm *= 1.03;
    ym *= 1.05;
    return { W, H, xm, ym, px: v => PAD.l + v / xm * (W - PAD.l - PAD.r), py: v => H - PAD.b - v / ym * (H - PAD.t - PAD.b) };
  }

  function draw(h = null) {
    const dark = isDark(), m = current();
    drawMap(stepsMap, steps, stepsRange, 'steps', h, dark);
    drawMap(distMap, values[measure], range(values[measure]), m.unit, h, dark);
    const g = axes();
    if (!g.W || !g.H) return;
    const { x } = canvas2d(sc, g.W, g.H);
    x.clearRect(0, 0, g.W, g.H);
    x.font = `11px ${css('--font')}`;
    x.lineWidth = 1;
    for (const f of [0, .25, .5, .75, 1]) {
      const yv = f * g.ym, yy = g.py(yv), xv = f * g.xm;
      x.strokeStyle = css('--grid');
      x.beginPath();
      x.moveTo(PAD.l, yy);
      x.lineTo(g.W - PAD.r, yy);
      x.stroke();
      x.fillStyle = css('--ink-3');
      x.textAlign = 'right';
      x.fillText(m.unit === 'm' ? yv.toFixed(0) : fmt.steps(yv), PAD.l - 6, yy + 4);
      x.textAlign = 'center';
      x.fillText(fmt.steps(xv), g.px(xv), g.H - PAD.b + 14);
    }
    x.fillStyle = css('--ink-2');
    x.textAlign = 'center';
    x.fillText('steps until the goal', (PAD.l + g.W - PAD.r) / 2, g.H - 3);
    x.save();
    x.translate(11, (PAD.t + g.H - PAD.b) / 2);
    x.rotate(-Math.PI / 2);
    x.fillText(m.unit === 'm' ? 'distance (m)' : 'distance (steps)', 0, 0);
    x.restore();
    const ys = values[measure];
    x.globalAlpha = h ? .45 : .85;
    for (const k of idx) {
      x.fillStyle = tealRamp(Math.min(1, steps[k] / g.xm), dark);
      x.beginPath();
      x.arc(g.px(steps[k]), g.py(ys[k]), 2.2, 0, 7);
      x.fill();
    }
    x.globalAlpha = 1;
    if (h) {
      const hx = g.px(steps[h.k]), hy = g.py(ys[h.k]);
      x.strokeStyle = css('--ink-3');
      x.beginPath();
      x.moveTo(hx, PAD.t);
      x.lineTo(hx, g.H - PAD.b);
      x.moveTo(PAD.l, hy);
      x.lineTo(g.W - PAD.r, hy);
      x.stroke();
      x.fillStyle = css('--surface');
      x.beginPath();
      x.arc(hx, hy, 7, 0, 7);
      x.fill();
      x.fillStyle = dark ? '#fff' : '#111';
      x.beginPath();
      x.arc(hx, hy, 4.5, 0, 7);
      x.fill();
    }
  }

  const pick = (h, e) => {
    hover = h;
    draw(hover);
    if (!h) { tip.hide(); return; }
    const r = host.getBoundingClientRect(), m = current();
    tip.show(e.clientX - r.left, e.clientY - r.top, 'Ant Hardest Maze, 8 layers', [
      { label: 'Steps until the goal', value: `${fmt.steps(steps[h.k])} steps` },
      { label: m.title, value: format(values[measure][h.k], m.unit) },
    ]);
  };
  const leave = (e) => {
    if (e.pointerType === 'touch') return;
    hover = null;
    draw();
    tip.hide();
  };
  for (const c of [stepsMap, distMap]) {
    const onCell = (e) => {
      const r = c.cv.getBoundingClientRect();
      const j = Math.floor((e.clientX - r.left) / r.width * ny), i = Math.floor((e.clientY - r.top) / r.height * nx);
      let best = null, bd = 10;
      for (let a = Math.max(0, i - 3); a <= Math.min(nx - 1, i + 3); a++) {
        for (let b = Math.max(0, j - 3); b <= Math.min(ny - 1, j + 3); b++) {
          const k = a * ny + b, d = (a - i) ** 2 + (b - j) ** 2;
          if (steps[k] >= 0 && values[measure][k] >= 0 && d < bd) { bd = d; best = { i: a, j: b, k }; }
        }
      }
      pick(best, e);
    };
    c.cv.addEventListener('pointermove', onCell);
    c.cv.addEventListener('pointerdown', onCell);
    c.cv.addEventListener('pointerleave', leave);
  }
  const onScatter = (e) => {
    const r = sc.getBoundingClientRect(), mx = e.clientX - r.left, my = e.clientY - r.top, g = axes(), ys = values[measure];
    let best = -1, bd = 16 * 16;
    for (const k of idx) {
      const dx = g.px(steps[k]) - mx, dy = g.py(ys[k]) - my, d = dx * dx + dy * dy;
      if (d < bd) { bd = d; best = k; }
    }
    pick(best < 0 ? null : { i: Math.floor(best / ny), j: best % ny, k: best }, e);
  };
  sc.addEventListener('pointermove', onScatter);
  sc.addEventListener('pointerdown', onScatter);
  sc.addEventListener('pointerleave', leave);
  new ResizeObserver(() => draw(hover)).observe(sc);
  onThemeChange(() => draw(hover));
  setMeasure();
  return { state: () => ({ measure, rho, cells: idx.length }) };
}
