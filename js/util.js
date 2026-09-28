const jsonCache = new Map();

export function fetchJSON(url) {
  if (!jsonCache.has(url)) {
    const p = fetch(url).then(r => {
      if (!r.ok) throw new Error(`${url}: ${r.status}`);
      return r.json();
    });
    p.catch(() => { if (jsonCache.get(url) === p) jsonCache.delete(url); });
    jsonCache.set(url, p);
  }
  return jsonCache.get(url);
}

function concat(parts, n) {
  const out = new Uint8Array(n);
  let o = 0;
  for (const p of parts) { out.set(p, o); o += p.length; }
  return out.buffer;
}

// Some servers decompress .gz files on the way, GitHub Pages does not.
async function gunzip(buffer) {
  const b = new Uint8Array(buffer);
  if (b[0] !== 0x1f || b[1] !== 0x8b) return buffer;
  return new Response(new Blob([b]).stream().pipeThrough(new DecompressionStream('gzip'))).arrayBuffer();
}

export async function readStream(response, onProgress) {
  const total = +response.headers.get('Content-Length') || 0;
  if (!onProgress || !total || !response.body) return response.arrayBuffer();
  const reader = response.body.getReader(), parts = [];
  let n = 0;
  for (;;) {
    const { done, value } = await reader.read();
    if (done) break;
    parts.push(value);
    n += value.length;
    onProgress(Math.min(.99, n / total));
  }
  return concat(parts, n);
}

export async function fetchBuffer(url, onProgress = null) {
  const r = await fetch(url);
  if (!r.ok) throw new Error(`${url}: ${r.status}`);
  return gunzip(await readStream(r, onProgress));
}

export async function fetchYielding(url, gate) {
  const parts = [];
  let n = 0;
  for (;;) {
    await gate();
    const ctl = new AbortController();
    let paused = false;
    const watch = setInterval(() => { if (gate() && !paused) { paused = true; ctl.abort(); } }, 100);
    try {
      const r = await fetch(url, { priority: 'low', signal: ctl.signal, headers: n ? { Range: `bytes=${n}-` } : {} });
      if (!r.ok) throw new Error(`${url}: ${r.status}`);
      if (n && r.status !== 206) { parts.length = 0; n = 0; }
      const reader = r.body.getReader();
      for (;;) {
        const { done, value } = await reader.read();
        if (done) break;
        parts.push(value);
        n += value.length;
      }
      return concat(parts, n);
    } catch (err) {
      if (!paused) throw err;
    } finally {
      clearInterval(watch);
    }
  }
}

export const retryDelay = (attempt) => Math.min(30, 2 * 2 ** attempt);

export const sleep = (seconds) => new Promise(r => setTimeout(r, seconds * 1000));

export async function retry(fn, onError = null) {
  for (let attempt = 0; ; attempt++) {
    try {
      return await fn();
    } catch (err) {
      console.warn(err);
      const wait = retryDelay(attempt);
      onError?.(wait);
      await sleep(wait);
    }
  }
}

export function whenVisible(node, fn, margin = '600px') {
  const io = new IntersectionObserver((entries) => {
    if (entries.some(e => e.isIntersecting)) { io.disconnect(); fn(); }
  }, { rootMargin: margin });
  io.observe(node);
}

export function animate(frame) {
  let last = performance.now();
  const seen = new Set();
  const tick = (now) => {
    const dt = Math.max(0, Math.min(.1, (now - last) / 1000));
    last = now;
    try { frame(dt, now); } catch (err) {
      if (!seen.has(String(err))) { seen.add(String(err)); console.error(err); }
    }
    requestAnimationFrame(tick);
  };
  requestAnimationFrame(tick);
}

export const TASKS = { u4: 'Ant U4-Maze', bigmaze: 'Ant Big Maze', hardest: 'Ant Hardest Maze', u5: 'Ant U5-Maze',
  humanoid: 'Humanoid', humanoidu: 'Humanoid U-Maze', humanoidbig: 'Humanoid Big Maze' };
export const DEPTHS = [1, 2, 4, 8, 16, 32, 64];
export const METHODS = ['CRL', 'AC-CRL', 'SRL', 'ChronoSRL'];
const METHOD_VARS = { 'CRL': '--crl', 'AC-CRL': '--accrl', 'SRL': '--srl', 'ChronoSRL': '--ours' };

export function css(name) {
  return getComputedStyle(document.documentElement).getPropertyValue(name).trim();
}

export function methodColor(m) { return css(METHOD_VARS[m]); }

const darkScheme = () => matchMedia('(prefers-color-scheme: dark)');

export const isDark = () => darkScheme().matches;

export function onThemeChange(fn) { darkScheme().addEventListener('change', fn); }

export function tealRamp(t, dark = false) {
  t = Math.min(1, Math.max(0, t));
  const a = dark ? [36, 62, 58] : [214, 240, 234];
  const b = dark ? [150, 232, 210] : [6, 70, 62];
  const mid = dark ? [40, 160, 140] : [22, 145, 126];
  const [p, q, u] = t < .5 ? [a, mid, t * 2] : [mid, b, (t - .5) * 2];
  return `rgb(${p.map((v, i) => Math.round(v + (q[i] - v) * u)).join(',')})`;
}

export const fmt = {
  int: v => v == null || !isFinite(v) ? '–' : Math.round(v).toLocaleString('en-US'),
  steps: v => v >= 10000 ? `${(v / 1000).toFixed(0)}k` : fmt.int(v),
};

export function mean(a) { let s = 0; for (const v of a) s += v; return s / a.length; }

export function se(a) {
  if (a.length < 2) return 0;
  const m = mean(a);
  let s = 0;
  for (const v of a) s += (v - m) * (v - m);
  return Math.sqrt(s / (a.length - 1)) / Math.sqrt(a.length);
}

export function canvas2d(cv, W = cv.clientWidth, H = cv.clientHeight) {
  const dpr = Math.min(2, devicePixelRatio || 1);
  if (cv.width !== Math.round(W * dpr) || cv.height !== Math.round(H * dpr)) {
    cv.width = Math.round(W * dpr);
    cv.height = Math.round(H * dpr);
  }
  const x = cv.getContext('2d');
  x.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { x, W, H };
}

export function star(x, cx, cy, r) {
  x.beginPath();
  for (let k = 0; k < 10; k++) {
    const a = -Math.PI / 2 + k * Math.PI / 5, rr = k % 2 ? r * .45 : r;
    x.lineTo(cx + Math.cos(a) * rr, cy + Math.sin(a) * rr);
  }
  x.closePath();
}

export function el(tag, attrs = {}, ...kids) {
  const e = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs)) {
    if (v == null || v === false) continue;
    if (k === 'class') e.className = v;
    else if (k === 'text') e.textContent = v;
    else e.setAttribute(k, v === true ? '' : v);
  }
  for (const k of kids) if (k != null) e.append(k);
  return e;
}

export function svg(tag, attrs = {}, parent = null) {
  const e = document.createElementNS('http://www.w3.org/2000/svg', tag);
  for (const [k, v] of Object.entries(attrs)) if (v != null) e.setAttribute(k, v);
  if (parent) parent.appendChild(e);
  return e;
}

export function button(text, onClick, attrs = {}) {
  const b = el('button', { type: 'button', text, ...attrs });
  b.addEventListener('click', onClick);
  return b;
}

export function segmented(items, value, onChange, { small = false, label = '' } = {}) {
  const root = el('div', { class: 'seg' + (small ? ' small' : ''), role: 'radiogroup', 'aria-label': label });
  const buttons = items.map(it => {
    const b = button(it.label, () => { set(it.value); onChange(it.value); },
      { role: 'radio', 'aria-checked': String(it.value === value), title: it.title });
    root.append(b);
    return b;
  });
  const set = (v) => items.forEach((it, i) => buttons[i].setAttribute('aria-checked', String(it.value === v)));
  return { el: root, set };
}

export function row(label, ...kids) {
  return el('div', { class: 'row' }, el('span', { text: label }), ...kids);
}

const openTips = new Set();
let closeTipsOnResize = false;

export function tooltip(container) {
  if (!closeTipsOnResize) {
    closeTipsOnResize = true;
    addEventListener('resize', () => { for (const t of openTips) t.hidden = true; openTips.clear(); }, { passive: true });
  }
  const t = el('div', { class: 'tip', hidden: true });
  container.appendChild(t);
  return {
    show(x, y, title, rows) {
      t.replaceChildren();
      if (title) t.append(el('div', { class: 't', text: title }));
      for (const r of rows) {
        const line = el('div', { class: 'r' });
        if (r.color) { const i = el('i'); i.style.background = r.color; line.append(i); }
        line.append(el('span', { text: r.label }));
        if (r.value != null) line.append(el('b', { text: r.value }));
        t.append(line);
      }
      t.hidden = false;
      openTips.add(t);
      const tw = t.offsetWidth, th = t.offsetHeight;
      let left = x + 14, top = y - th - 10;
      if (left + tw > container.clientWidth) left = x - tw - 14;
      if (left < 0) left = 4;
      const r = container.getBoundingClientRect(), card = container.closest('.card')?.getBoundingClientRect();
      const lo = Math.max(8, card ? card.left + 6 : 8);
      const hi = Math.min(document.documentElement.clientWidth - 8, card ? card.right - 6 : Infinity);
      left = Math.max(lo - r.left, Math.min(left, hi - r.left - tw));
      if (top < 0) top = y + 14;
      t.style.left = left + 'px';
      t.style.top = top + 'px';
    },
    hide() { t.hidden = true; openTips.delete(t); },
  };
}
