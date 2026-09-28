import { svg, tooltip } from './util.js';

export function niceTicks(min, max, n = 4) {
  const step0 = (max - min || 1) / n, mag = Math.pow(10, Math.floor(Math.log10(step0))), err = step0 / mag;
  const step = (err >= 7.5 ? 10 : err >= 3.5 ? 5 : err >= 1.5 ? 2 : 1) * mag, out = [];
  for (let v = Math.ceil(min / step) * step; v <= max + 1e-9; v += step) out.push(+v.toFixed(10));
  return out;
}

function title(s, text, x0, room) {
  const t = svg('text', { x: x0, y: 14, class: 'title' }, s), fs = 12.5;
  t.textContent = text;
  const len = t.getComputedTextLength(), words = text.split(' ');
  if (len <= room) return 0;
  if (room / len >= .86 || words.length < 3) {
    t.style.fontSize = `${(fs * room / len).toFixed(1)}px`;
    return 0;
  }
  let best = null;
  for (let k = 1; k < words.length; k++) {
    const a = words.slice(0, k).join(' '), b = words.slice(k).join(' ');
    t.textContent = a;
    const la = t.getComputedTextLength();
    t.textContent = b;
    const w = Math.max(la, t.getComputedTextLength());
    if (!best || w < best.w) best = { a, b, w };
  }
  t.textContent = '';
  svg('tspan', { x: x0, dy: 0 }, t).textContent = best.a;
  svg('tspan', { x: x0, dy: '1.2em' }, t).textContent = best.b;
  if (best.w > room) t.style.fontSize = `${(fs * room / best.w).toFixed(1)}px`;
  return 15;
}

export class Chart {
  constructor(host, opts) {
    this.host = host;
    this.opts = opts;
    host.classList.add('chart-host');
    host.replaceChildren();
    this.root = svg('svg', { class: 'chart', role: 'img', 'aria-label': opts.title || 'chart' }, host);
    this.tip = tooltip(host);
    new ResizeObserver((entries) => {
      if (Math.round(entries[entries.length - 1].contentRect.width) === this.width || this.pending) return;
      this.pending = true;
      requestAnimationFrame(() => { this.pending = false; this.render(); });
    }).observe(host);
    this.render();
  }

  update(opts) {
    Object.assign(this.opts, opts);
    this.render();
  }

  render() {
    const o = this.opts, s = this.root, W = this.width = Math.max(160, this.host.clientWidth || 300);
    const H = Math.round(Math.min(Math.max(W * (o.aspect || .72), 150), o.maxHeight || 420));
    const m = { l: 40, r: 10, t: o.title ? 24 : 8, b: o.xLabel ? 34 : 22, ...o.margin };
    s.replaceChildren();
    s.setAttribute('viewBox', `0 0 ${W} ${H}`);
    s.setAttribute('width', W);
    s.setAttribute('height', H);
    if (o.title) m.t += title(s, o.title, Math.min(m.l, 4), W - Math.min(m.l, 4) - 2);
    const iw = W - m.l - m.r, ih = H - m.t - m.b, [x0, x1] = o.x.domain, [y0, y1] = o.y.domain;
    const xs = o.x.type === 'log2' ? v => m.l + (Math.log2(v) - Math.log2(x0)) / (Math.log2(x1) - Math.log2(x0)) * iw : v => m.l + (v - x0) / (x1 - x0) * iw;
    const ys = v => m.t + ih - (v - y0) / (y1 - y0) * ih;
    const text = (content, attrs) => { svg('text', attrs, s).textContent = content; };
    const grid = svg('g', { class: 'grid' }, s);
    for (const v of Array.isArray(o.y.ticks) ? o.y.ticks : niceTicks(y0, y1, o.y.ticks || 4)) {
      svg('line', { x1: m.l, x2: m.l + iw, y1: ys(v), y2: ys(v) }, grid);
      text(o.y.fmt ? o.y.fmt(v) : String(v), { x: m.l - 6, y: ys(v) + 3.5, 'text-anchor': 'end' });
    }
    for (const v of o.x.ticks || niceTicks(x0, x1, 4)) text(o.x.fmt ? o.x.fmt(v) : String(v), { x: xs(v), y: m.t + ih + 15, 'text-anchor': 'middle' });
    svg('line', { class: 'axis', x1: m.l, x2: m.l + iw, y1: m.t + ih, y2: m.t + ih }, s);
    if (o.xLabel) text(o.xLabel, { x: m.l + iw / 2, y: H - 4, 'text-anchor': 'middle', class: 'axis-label' });
    if (o.yLabel) text(o.yLabel, { x: 11, y: m.t + ih / 2, 'text-anchor': 'middle', class: 'axis-label', transform: `rotate(-90 11 ${m.t + ih / 2})` });
    for (const h of o.hlines || []) {
      if (h.y < y0 || h.y > y1) continue;
      svg('line', { x1: m.l, x2: m.l + iw, y1: ys(h.y), y2: ys(h.y), stroke: 'currentColor', 'stroke-width': 1.2, 'stroke-dasharray': '5 3', opacity: .75, style: 'color: var(--ink-2)' }, s);
      text(h.label, { x: h.left ? m.l + 4 : m.l + iw - 2, y: ys(h.y) - 4, 'text-anchor': h.left ? 'start' : 'end', class: 'axis-label' });
    }
    const clip = `clip${Math.random().toString(36).slice(2, 8)}`;
    svg('rect', { x: m.l - 6, y: m.t - 6, width: iw + 12, height: ih + 12 }, svg('clipPath', { id: clip }, svg('defs', {}, s)));
    const plot = svg('g', { 'clip-path': `url(#${clip})` }, s);
    for (const se of o.series) {
      const pts = se.points.filter(p => p.y != null && isFinite(p.y));
      if (!pts.length) continue;
      if (se.band) {
        const up = pts.map(p => `${xs(p.x)},${ys(p.hi)}`), down = pts.slice().reverse().map(p => `${xs(p.x)},${ys(p.lo)}`);
        svg('polygon', { points: up.concat(down).join(' '), fill: se.color, opacity: .12 }, plot);
      }
      if (se.errorbars) {
        for (const p of pts) if (p.lo != null) svg('line', { x1: xs(p.x), x2: xs(p.x), y1: ys(p.lo), y2: ys(p.hi), stroke: se.color, 'stroke-width': 1.4, opacity: .8 }, plot);
      }
      if (se.line !== false) {
        svg('polyline', { points: pts.map(p => `${xs(p.x)},${ys(p.y)}`).join(' '), fill: 'none', stroke: se.color, 'stroke-width': se.width || 2,
          'stroke-linejoin': 'round', 'stroke-linecap': 'round', opacity: se.opacity ?? 1 }, plot);
      }
      if (se.marker) {
        for (const p of pts) {
          const cx = xs(p.x), cy = ys(p.y);
          const paint = p.open ? { fill: 'var(--surface)', stroke: se.color, 'stroke-width': 1.8 } : { fill: se.color, stroke: 'var(--surface)', 'stroke-width': 2 };
          if (se.marker === 'diamond') {
            const r = p.open ? 4.5 : 5;
            svg('polygon', { points: `${cx},${cy - r} ${cx + r},${cy} ${cx},${cy + r} ${cx - r},${cy}`, ...paint }, plot);
          } else {
            svg('circle', { cx, cy, r: p.open ? 3.6 : 4, ...paint }, plot);
          }
        }
      }
    }
    if (o.hover) this.hover(s, o, xs, ys, m, iw, ih, W);
  }

  hover(s, o, xs, ys, m, iw, ih, W) {
    const cross = svg('line', { class: 'cross', y1: m.t, y2: m.t + ih, visibility: 'hidden' }, s);
    const rings = svg('g', { class: 'rings' }, s);
    const hit = svg('rect', { class: 'hit' + (o.onClick ? ' clickable' : ''), x: m.l, y: m.t, width: iw, height: ih, tabindex: 0,
      role: o.onClick ? 'button' : null, 'aria-label': o.onClick ? `${o.title || 'chart'}: choose a point with the arrow keys and press Enter` : null }, s);
    const list = o.hover.xs;
    const pick = (e) => {
      const r = s.getBoundingClientRect(), px = (e.clientX - r.left) * W / r.width;
      let best = null, bd = Infinity;
      for (const v of list) { const d = Math.abs(xs(v) - px); if (d < bd) { bd = d; best = v; } }
      return best;
    };
    const show = (x) => {
      const cx = xs(x);
      cross.setAttribute('x1', cx);
      cross.setAttribute('x2', cx);
      cross.setAttribute('visibility', 'visible');
      if (o.onClick) {
        rings.replaceChildren();
        for (const se of o.series) {
          const q = se.marker && se.points.find(p => p.x === x && p.y != null && isFinite(p.y));
          if (q) svg('circle', { cx, cy: ys(q.y), r: 7.5, fill: 'none', stroke: se.color, 'stroke-width': 1.6, opacity: .9 }, rings);
        }
      }
      const info = o.hover.format(x);
      this.tip.show(cx, m.t + 8, info.title, info.rows);
      this.hovered = x;
    };
    const hide = () => { cross.setAttribute('visibility', 'hidden'); rings.replaceChildren(); this.tip.hide(); };
    hit.addEventListener('pointermove', (e) => show(pick(e)));
    hit.addEventListener('pointerleave', hide);
    hit.addEventListener('focus', () => show(list[list.length - 1]));
    hit.addEventListener('blur', hide);
    hit.addEventListener('keydown', (e) => {
      const i = list.indexOf(this.hovered);
      if (e.key === 'ArrowRight' && i < list.length - 1) show(list[i + 1]);
      if (e.key === 'ArrowLeft' && i > 0) show(list[i - 1]);
      if (e.key === 'Enter' && o.onClick) o.onClick(this.hovered);
    });
    if (o.onClick) hit.addEventListener('click', (e) => o.onClick(pick(e)));
  }
}
