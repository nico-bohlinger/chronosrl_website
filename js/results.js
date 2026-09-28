import { TASKS, DEPTHS, METHODS, fetchJSON, methodColor, el, segmented, mean, se, fmt, onThemeChange } from './util.js';
import { pointerVerb } from './hud.js';
import { Chart, niceTicks } from './plots.js';

const state = { stat: 'auc', task: 'hardest', depth: 8, seeds: false, open: false };
const charts = [];
let DATA, curveChart = null, curveDepth = null;

function seedStat(curve, stat) {
  return stat === 'auc' ? mean(curve.slice(0, 100)) : mean(curve.slice(95, 100));
}

function cell(task, method, depth) {
  const c = DATA.curves[task]?.[method]?.[depth];
  return c?.length === 4 ? c : null;
}

function depthPoints(task, method) {
  return DEPTHS.map(d => {
    const c = cell(task, method, d);
    if (!c) return { x: d, y: null };
    const v = c.map(s => seedStat(s, state.stat)), m = mean(v), e = se(v);
    return { x: d, y: m, lo: m - e, hi: m + e };
  });
}

function legend() {
  document.getElementById('method-legend').replaceChildren(...METHODS.map(m => {
    const i = el('i');
    i.style.background = methodColor(m);
    return el('span', {}, i, m);
  }));
}

function renderGrid() {
  const grid = document.getElementById('depth-grid');
  if (!charts.length) {
    grid.replaceChildren();
    for (const task of [...Object.keys(TASKS), null]) {
      const host = el('div');
      grid.append(host);
      charts.push({ task, host, chart: null });
    }
  }
  for (const c of charts) {
    const opts = c.task ? depthOptions(c) : stepsToMatchOptions();
    if (c.chart) c.chart.update(opts); else c.chart = new Chart(c.host, opts);
  }
}

function depthOptions(c) {
  const name = TASKS[c.task];
  const series = METHODS.map(m => ({ name: m, color: methodColor(m), points: depthPoints(c.task, m), errorbars: true,
    marker: m === 'ChronoSRL' ? 'diamond' : 'dot', width: m === 'ChronoSRL' ? 2.4 : 1.8 }));
  const ymax = Math.max(50, ...series.flatMap(s => s.points.map(p => p.hi ?? 0))), top = niceTicks(0, ymax * 1.05, 3);
  return {
    title: name, aspect: .78, x: { type: 'log2', domain: [0.8, 80], ticks: DEPTHS },
    y: { domain: [0, top[top.length - 1] >= ymax ? top[top.length - 1] : ymax * 1.08], ticks: 3, fmt: fmt.int },
    xLabel: 'Network depth', series, margin: { l: 36, r: 6 },
    hover: { xs: DEPTHS, format: (d) => ({
      title: `${name}, ${d} layer${d > 1 ? 's' : ''}`,
      rows: METHODS.slice().reverse().map(m => {
        const p = series.find(s => s.name === m).points.find(q => q.x === d);
        return { color: methodColor(m), label: m, value: p?.y == null ? '–' : `${fmt.int(p.y)} ± ${fmt.int(p.hi - p.y)}` };
      }).concat([{ label: `${pointerVerb().toLowerCase()} for the training curves` }]),
    }) },
    onClick: (d) => { setCurves(c.task, d); openCurves(c.host); },
  };
}

function stepsToMatchOptions() {
  const auc = state.stat === 'auc', rows = auc ? DATA.steps_to_match_auc : DATA.steps_to_match;
  const depths = [8, 16, 32, 64], tasks = Object.keys(TASKS), budget = DATA.budget_m;
  const median = depths.map(d => {
    const v = tasks.map(t => rows[`${t}:${d}`]?.steps_m).filter(x => x != null).sort((a, b) => a - b);
    return { x: d, y: !v.length ? null : v.length % 2 ? v[(v.length - 1) / 2] : (v[v.length / 2 - 1] + v[v.length / 2]) / 2 };
  });
  const series = [{ name: 'median', color: 'var(--ink-2)', points: median, width: 1.6, marker: null }];
  tasks.forEach((t, i) => series.push({ name: t, color: methodColor('ChronoSRL'), line: false, marker: t.startsWith('humanoid') ? 'diamond' : 'dot',
    points: depths.map(d => {
      const r = rows[`${t}:${d}`], x = d * (1 + (i - 3) * 0.045);
      if (!r) return { x: d, y: null };
      return r.steps_m != null ? { x, y: r.steps_m } : { x, y: budget, open: true };
    }) }));
  const target = (r) => `${r.best}'s ${auc ? 'AUC' : 'final'} ${fmt.int(r.threshold)}`;
  return {
    title: auc ? "Steps to match the best baseline's AUC" : "Steps to match the best baseline's final time at goal", aspect: .78,
    x: { type: 'log2', domain: [6, 84], ticks: depths }, y: { domain: [0, 110], ticks: [0, 25, 50, 75, 100], fmt: v => v + 'M' },
    xLabel: 'Network depth', series, margin: { l: 36, r: 6 }, hlines: [{ y: budget, label: 'budget' }],
    hover: { xs: depths, format: (d) => {
      const m = median.find(p => p.x === d).y;
      return { title: `ChronoSRL, ${d} layers`, rows: tasks.map(t => {
        const r = rows[`${t}:${d}`];
        return { label: TASKS[t], value: !r ? '–' : r.steps_m != null ? `${r.steps_m}M (${target(r)})` : `not reached within ${budget}M (${target(r)})` };
      }).concat([{ label: 'median', value: m != null ? `${m}M` : '–' }]) };
    } },
  };
}

function setCurves(task, depth) {
  state.task = task;
  state.depth = depth;
  document.getElementById('curve-task').value = task;
  curveDepth.set(depth);
  if (state.open) renderCurves();
}

function openCurves(anchor) {
  const card = document.getElementById('curve-card'), first = !state.open;
  if (first) {
    state.open = true;
    card.classList.add('open');
    document.getElementById('curve-cta').setAttribute('aria-expanded', 'true');
    renderCurves();
  }
  setTimeout(() => {
    const need = card.getBoundingClientRect().bottom - innerHeight + 16, by = Math.min(need, anchor.getBoundingClientRect().top - 8);
    if (by > 4) scrollBy({ top: by, behavior: matchMedia('(prefers-reduced-motion: reduce)').matches ? 'instant' : 'smooth' });
  }, first ? 480 : 0);
}

function renderCurves() {
  const x = Array.from({ length: 100 }, (_, i) => i + 1), series = [];
  let ymax = 50;
  for (const m of METHODS) {
    const c = cell(state.task, m, state.depth);
    if (!c) continue;
    const color = methodColor(m);
    if (state.seeds) c.forEach(s => series.push({ name: m + ' seed', color, width: 1, opacity: .35, points: s.map((v, i) => ({ x: x[i], y: v })) }));
    const points = x.map((xi, i) => {
      const v = c.map(s => s[i]), mm = mean(v), e = se(v);
      ymax = Math.max(ymax, state.seeds ? Math.max(...v) : mm + e);
      return { x: xi, y: mm, lo: mm - e, hi: mm + e };
    });
    series.push({ name: m, color, points, band: !state.seeds, width: m === 'ChronoSRL' ? 2.4 : 1.8 });
  }
  const top = niceTicks(0, ymax * 1.04, 4);
  const opts = {
    aspect: .42, maxHeight: 380, x: { domain: [0, 100], ticks: [0, 20, 40, 60, 80, 100], fmt: v => v + 'M' },
    y: { domain: [0, Math.max(top[top.length - 1], ymax)], ticks: 4, fmt: fmt.int },
    xLabel: 'Environment steps', yLabel: 'Time at goal', series, margin: { l: 50, r: 10 },
    hover: { xs: x, format: (xi) => ({ title: `${xi}M steps`, rows: METHODS.slice().reverse().map(m => {
      const p = series.find(q => q.name === m)?.points[xi - 1];
      return { color: methodColor(m), label: m, value: p ? fmt.int(p.y) : '–' };
    }) }) },
  };
  if (curveChart) curveChart.update(opts); else curveChart = new Chart(document.getElementById('curve-plot'), opts);
}

function setupControls() {
  legend();
  document.getElementById('curve-cta-text').textContent = `${pointerVerb()} a point above to see its training curves`;
  document.getElementById('curve-cta').addEventListener('click', (e) => { if (!state.open) openCurves(e.currentTarget); });
  const stat = document.getElementById('depth-stat');
  stat.addEventListener('click', (e) => {
    const b = e.target.closest('button');
    if (!b) return;
    stat.querySelectorAll('button').forEach(x => x.setAttribute('aria-checked', String(x === b)));
    state.stat = b.dataset.stat;
    renderGrid();
  });
  const select = document.getElementById('curve-task');
  select.replaceChildren(...Object.entries(TASKS).map(([value, text]) => el('option', { value, text })));
  select.value = state.task;
  select.addEventListener('change', () => { state.task = select.value; renderCurves(); });
  curveDepth = segmented(DEPTHS.map(d => ({ value: d, label: String(d) })), state.depth, (d) => { state.depth = d; renderCurves(); }, { small: true, label: 'Depth' });
  document.getElementById('curve-depth').replaceWith(curveDepth.el);
  curveDepth.el.id = 'curve-depth';
  document.getElementById('curve-seeds').addEventListener('change', (e) => { state.seeds = e.target.checked; renderCurves(); });
}

export async function init() {
  DATA = await fetchJSON('assets/data/bench.json');
  setupControls();
  renderGrid();
  onThemeChange(() => { legend(); renderGrid(); if (state.open) renderCurves(); });
  return { state: () => ({ ...state }) };
}
