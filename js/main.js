import { TASKS, DEPTHS, fetchJSON, retry, sleep, whenVisible } from './util.js';
import { MANIFEST, envFile, policyFile, prefetch } from './sim/loader.js';
import { pinSection } from './pin.js';

const sections = {};
let robotFirst = null, robots = null;

window.siteState = () => Object.fromEntries(Object.entries(sections).map(([name, s]) => [name, s?.state()]));

const load = (name, init) => retry(init).then(s => { sections[name] = s; });
const afterRobot = (fn) => () => Promise.resolve(robotFirst).then(fn);

document.getElementById('bib-copy').addEventListener('click', async (e) => {
  try {
    await navigator.clipboard.writeText(document.getElementById('bib-text').textContent);
    e.target.textContent = 'Copied';
  } catch {
    e.target.textContent = 'Select and copy';
  }
  setTimeout(() => { e.target.textContent = 'Copy'; }, 1600);
});
document.getElementById('paper-link').addEventListener('click', (e) => {
  if (e.currentTarget.getAttribute('aria-disabled') === 'true') e.preventDefault();
});

const query = new URLSearchParams(location.search);
const task = Object.hasOwn(TASKS, query.get('task')) ? query.get('task') : 'bigmaze';
const depth = DEPTHS.includes(+query.get('depth')) ? +query.get('depth') : 64;
fetchJSON(envFile(task)).catch(() => {});
const prefetched = fetchJSON(MANIFEST).then(m => prefetch(policyFile(task, depth, m[task][depth])), () => null);
const playground = load('playground', () => import('./playground/playground.js').then(m => m.init({ task, depth, prefetched, gate: () => robotFirst })));
const later = (fn) => () => Promise.race([playground, sleep(8)]).then(fn);

const mathStyles = () => new Promise((resolve, reject) => {
  const link = Object.assign(document.createElement('link'), { rel: 'stylesheet', href: 'vendor/katex/katex.min.css' });
  link.onload = () => { document.getElementById('method-text').classList.add('math-ready'); resolve(); };
  link.onerror = () => { link.remove(); reject(new Error(link.href)); };
  document.head.append(link);
});
whenVisible(document.getElementById('method'), () => retry(mathStyles));
whenVisible(document.getElementById('geometry'), later(afterRobot(() => load('geometry', () => import('./geometry.js').then(m => m.init())))));
whenVisible(document.getElementById('results'), later(afterRobot(() => load('results', () => import('./results.js').then(m => m.init())))));

const loadRobots = () => robots || (robots = load('robots', () => import('./robots.js').then(m => m.init())));
whenVisible(document.getElementById('robots'), later(loadRobots), '400px');

function openRobot() {
  const section = document.getElementById('robots');
  history.replaceState(null, '', '#robots');
  if (!sections.robots?.state().steps && !robotFirst) {
    let release;
    robotFirst = new Promise(r => { release = r; });
    const t0 = performance.now(), poll = setInterval(() => {
      if (sections.robots?.state().steps > 0 || performance.now() - t0 > 20000) {
        clearInterval(poll);
        robotFirst = null;
        release();
      }
    }, 100);
  }
  const smooth = !matchMedia('(prefers-reduced-motion: reduce)').matches;
  section.scrollIntoView({ behavior: smooth ? 'smooth' : 'instant', block: 'start' });
  pinSection(section, smooth);
  loadRobots().then(() => document.getElementById('go2-stage').focus({ preventScroll: true }));
}
document.getElementById('robot-cta').addEventListener('click', (e) => { e.preventDefault(); openRobot(); });

if (location.hash === '#robots') {
  loadRobots();
  pinSection(document.getElementById('robots'), false);
}
