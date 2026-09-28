import { DEPTHS, METHODS, fetchJSON, methodColor, mean, se, fmt, el, retry, onThemeChange } from './util.js';
import { Chart, niceTicks } from './plots.js';

const PANELS = [
  { task: 'velocity', field: 'tog', title: 'Velocity: time at goal', fmt: fmt.int, ppo: 'vel_tog' },
  { task: 'velocity', field: 'error', title: 'Velocity: error (m/s)', fmt: v => v.toFixed(2), ppo: 'vel_err' },
  { task: 'position', field: 'tog', title: 'Position: time at goal', fmt: fmt.int },
  { task: 'box', field: 'box_cm', title: 'Box climb: height (cm)', fmt: v => v.toFixed(1) },
];

function points(robots, p, method) {
  return DEPTHS.map(d => {
    const v = (robots[p.task]?.[method]?.[d] || []).filter(r => r[p.field]?.length >= 100).map(r => mean(r[p.field].slice(95, 100)));
    if (v.length < 4) return { x: d, y: null };
    const m = mean(v), e = se(v);
    return { x: d, y: m, lo: m - e, hi: m + e };
  });
}

async function panels() {
  const { robots, ppo } = await fetchJSON('assets/data/robots.json');
  const host = document.getElementById('robot-plots'), charts = PANELS.map(() => null);
  host.replaceChildren(...PANELS.map(() => el('div')));
  const render = () => PANELS.forEach((p, i) => {
    const series = METHODS.map(m => ({ name: m, color: methodColor(m), points: points(robots, p, m), errorbars: true,
      marker: m === 'ChronoSRL' ? 'diamond' : 'dot', width: m === 'ChronoSRL' ? 2.4 : 1.8 }));
    const lines = p.ppo ? [{ y: ppo[p.ppo], label: 'PPO, 1B steps', left: true }] : [];
    const ymax = Math.max(...series.flatMap(s => s.points.map(q => q.hi ?? 0)), ...lines.map(l => l.y)), t = niceTicks(0, ymax * 1.06, 3);
    const opts = {
      title: p.title, aspect: .78, x: { type: 'log2', domain: [.8, 80], ticks: DEPTHS },
      y: { domain: [0, Math.max(t[t.length - 1], ymax * 1.06)], ticks: 3, fmt: v => p.field === 'error' ? v.toFixed(1) : fmt.int(v) },
      xLabel: 'Network depth', series, hlines: lines, margin: { l: 36, r: 6 },
      hover: { xs: DEPTHS, format: d => ({ title: `${d} layer${d > 1 ? 's' : ''}`, rows: METHODS.slice().reverse().map(m => {
        const q = series.find(s => s.name === m).points.find(z => z.x === d);
        return { color: methodColor(m), label: m, value: q?.y == null ? '–' : `${p.fmt(q.y)} ± ${p.fmt(q.hi - q.y)}` };
      }).concat(lines.map(l => ({ label: 'PPO (reward-based, 1B steps)', value: p.fmt(l.y) }))) }) },
    };
    if (charts[i]) charts[i].update(opts); else charts[i] = new Chart(host.children[i], opts);
  });
  render();
  onThemeChange(render);
}

let view = null;

async function load() {
  const [{ Go2View }, { Go2, loadAssets }] = await Promise.all([import('./go2/scene.js'), import('./go2/go2.js')]);
  if (!view) view = Go2View.create(document.getElementById('go2-stage')).catch(err => { view = null; throw err; });
  const [v, assets] = await Promise.all([view, loadAssets()]);
  return { Go2, view: v, assets };
}

export async function init() {
  const status = document.getElementById('go2-status'), stage = document.getElementById('go2-stage');
  status.textContent = 'Loading the Go2…';
  retry(panels);
  const { Go2, view: v, assets } = await retry(load, (wait) => { status.textContent = `The Go2 could not be loaded, retrying in ${wait} s`; });
  try {
    const go2 = new Go2({ view: v, stage, panel: document.getElementById('go2-panel'), assets });
    status.remove();
    return go2;
  } catch (err) {
    console.error(err);
    status.textContent = 'The Go2 viewer could not start in this browser';
    return null;
  }
}
