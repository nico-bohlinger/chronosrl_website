import { TASKS, DEPTHS, fetchJSON, el, row, button, segmented, animate, retryDelay } from '../util.js';
import { Hud, HintPill, pointerVerb } from '../hud.js';
import { MANIFEST, PolicySlot, envFile } from '../sim/loader.js';
import { Env } from '../sim/envs.js';
import { bounds, freeGoal, drawPoster, Minimap } from './maze.js';

const S = { task: '', depth: 0, speed: 1, paused: false, acc: 0, env: null, prevX: null, chunk: null, t: 0, tog: 0,
  hist: [], episodes: 0, goals: 0, steps: 0, cue: '', cueUntil: 0, taskToken: 0 };
let manifest, slot, view = null, hud, hint, minimap, stage, status;

function setStatus(text) {
  status.textContent = text;
  status.hidden = !text;
}

function newEpisode(cue = '') {
  const env = S.env;
  env.reset(...env.resetState(Math.random));
  Object.assign(S, { prevX: null, chunk: null, tog: 0, t: 0, hist: [], cue, cueUntil: cue ? performance.now() + 900 : 0 });
  S.episodes++;
  view?.clearTrail();
  view?.setGoal(env.goal());
}

function moveGoal(x, y) {
  if (!S.env) return;
  const g = freeGoal(S.env.task, [x, y]);
  S.env.setGoal(g);
  Object.assign(S, { chunk: null, tog: 0, hist: [] });
  S.goals++;
  view?.setGoal(g);
  hint?.done();
}

function controlStep() {
  const env = S.env, policy = slot.policy, H = policy.header.chunk, A = env.nu;
  if (S.t % H === 0 || !S.chunk) {
    S.chunk = policy.act(env.obs);
    if (slot.criticReady) {
      S.hist.push({ t: S.t, d: policy.observationDistance(env.obs, S.chunk, env.goalDim), e: env.goalDist() });
      if (S.hist.length > 400) S.hist.shift();
    }
  }
  const k = S.t % H;
  S.prevX = env.state.x.map(x => ({ pos: x.pos.slice(), rot: x.rot.slice() }));
  const r = env.step(S.chunk.subarray(k * A, (k + 1) * A));
  S.t++;
  S.steps++;
  if (r.success) S.tog++;
  if (r.done) return newEpisode('fell over, new episode');
  if (!Number.isFinite(env.state.x[0].pos[2])) return newEpisode('new episode');
  if (S.t >= 1000) return newEpisode('time is up, new goal');
  if (S.t % 2 === 0) { const p = env.state.x[0].pos; view?.pushTrail(p[0], p[1]); }
}

function updateHud() {
  const env = S.env, dist = env.goalDist(), reached = dist < env.task.success_radius;
  hud.update({ step: S.t, atGoal: S.tog, dist: `${dist.toFixed(1)} m`, meter: S.tog / 1000,
    state: S.cue && performance.now() < S.cueUntil ? S.cue : !slot.policy ? 'waiting for the policy' : reached ? 'at the goal' : 'walking to the goal',
    ready: slot.criticReady, hist: S.hist, message: slot.criticMessage() });
  minimap?.draw(env.task, view.trail, env.goal(), env.state.x[0].pos);
}

function showHint() {
  hint?.show(`${pointerVerb()} the floor or the map to move the goal`, 'click');
}

async function selectPolicy() {
  if (await slot.select(S.task, S.depth, manifest[S.task][S.depth])) S.chunk = null;
}

function setDepth(depth) {
  S.depth = depth;
  selectPolicy();
}

async function setTask(task, attempt = 0) {
  const token = ++S.taskToken;
  S.task = task;
  S.chunk = null;
  slot.clear();
  let json;
  try {
    json = await fetchJSON(envFile(task));
  } catch (err) {
    console.warn(err);
    if (token !== S.taskToken) return;
    const wait = retryDelay(attempt);
    setStatus(`The task could not be loaded, retrying in ${wait} s`);
    setTimeout(() => { if (token === S.taskToken) setTask(task, attempt + 1); }, wait * 1000);
    return;
  }
  if (token !== S.taskToken) return;
  S.env = new Env(json);
  S.acc = 0;
  view?.setEnv(json);
  newEpisode();
  showHint();
  if (view?.follow) view.chase(S.env.state.x[0].pos); else view?.overview(bounds(json.task));
  await selectPolicy();
}

function buildPanel(panel) {
  const select = el('select', { 'aria-label': 'Task' });
  for (const group of ['Ant', 'Humanoid']) {
    const og = el('optgroup', { label: group });
    for (const [key, name] of Object.entries(TASKS)) {
      if (name.startsWith(group)) og.append(el('option', { value: key, text: key === 'humanoid' ? 'Humanoid Open field' : name }));
    }
    select.append(og);
  }
  select.value = S.task;
  select.addEventListener('change', () => setTask(select.value));
  const depth = segmented(DEPTHS.map(d => ({ value: d, label: String(d) })), S.depth, setDepth, { small: true, label: 'Depth' });
  const pause = button('Pause', () => { S.paused = !S.paused; pause.textContent = S.paused ? 'Play' : 'Pause'; });
  const speed = button('1×', () => { S.speed = { 1: 2, 2: 4, 4: .5, .5: 1 }[S.speed]; speed.textContent = `${S.speed}×`; }, { title: 'Simulation speed' });
  const follow = button('Overview', () => {
    if (view.follow) { view.overview(bounds(S.env.task)); follow.textContent = 'Follow'; }
    else { view.chase(S.env.state.x[0].pos); follow.textContent = 'Overview'; }
  }, { title: 'Switch between following the agent and the whole task' });
  panel.prepend(row('Task', el('label', { class: 'select' }, select)), row('Network depth', depth.el),
    el('div', { class: 'sim-actions' }, button('New goal', () => newEpisode()), pause, speed, follow));
}

function frame(dt, now) {
  const env = S.env;
  if (!env) return;
  const step = env.sys.dt * env.sys.nFrames;
  if (slot.policy?.header.task === env.json.task.key && !S.paused && view.visible) {
    S.acc += dt * S.speed;
    const maxSteps = Math.ceil(.1 / step) * S.speed + 1, t0 = performance.now();
    for (let n = 0; S.acc >= step && n < maxSteps; n++) {
      controlStep();
      S.acc -= step;
      if (performance.now() - t0 > 24) { S.acc = 0; break; }
    }
  }
  const alpha = S.prevX ? Math.min(1, S.acc / step) : 1, x = env.state.x, b = S.prevX ? S.prevX[0].pos : x[0].pos, a = x[0].pos;
  view.setPose(x, S.prevX, alpha);
  view.track([b[0] + (a[0] - b[0]) * alpha, b[1] + (a[1] - b[1]) * alpha, b[2] + (a[2] - b[2]) * alpha]);
  view.setGoal(env.goal(), env.goalDist() < env.task.success_radius, now / 1000);
  view.render();
  if (now - (S.lastHud || 0) > 100) { S.lastHud = now; updateHud(); }
}

export async function init({ task, depth, prefetched, gate }) {
  const [m, json, pre] = await Promise.all([fetchJSON(MANIFEST), fetchJSON(envFile(task)), prefetched]);
  manifest = m;
  Object.assign(S, { task, depth });
  stage = document.getElementById('pg-stage');
  status = document.getElementById('pg-status');
  buildPanel(document.getElementById('pg-panel'));
  hud = new Hud(stage);
  slot = new PolicySlot({ status: setStatus, gate, prefetched: pre, onChange: () => { S.hist = []; S.lastHud = 0; } });
  const poster = document.getElementById('pg-poster');
  drawPoster(poster, json.task);
  S.env = new Env(json);
  newEpisode();
  const policyReady = selectPolicy();
  const { PlaygroundView } = await import('./scene.js');
  view = new PlaygroundView(stage);
  view.onPick = moveGoal;
  view.setEnv(S.env.json);
  view.chase(S.env.state.x[0].pos);
  view.setGoal(S.env.goal());
  const map = el('canvas', { class: 'minimap', 'aria-label': 'Map of the task, click to move the goal', title: 'Click to move the goal' });
  stage.append(map);
  minimap = new Minimap(map);
  map.addEventListener('click', (e) => {
    const r = map.getBoundingClientRect();
    moveGoal(...minimap.transform(S.env.task).toWorld(e.clientX - r.left, e.clientY - r.top));
  });
  poster.remove();
  hint = new HintPill(stage);
  showHint();
  animate(frame);
  await policyReady;
  return {
    state: () => ({ task: S.task, depth: S.depth, seed: manifest[S.task][S.depth], envTask: S.env.json.task.key, paused: S.paused,
      speed: S.speed, t: S.t, steps: S.steps, episodes: S.episodes, goals: S.goals, hist: S.hist.length,
      histT: S.hist.length ? S.hist[S.hist.length - 1].t + 1e4 * S.episodes + 1e7 * S.goals : null,
      latest: S.hist.length ? S.hist[S.hist.length - 1].d.toFixed(2) : null, hud: `${hud.step.textContent}|${hud.state.textContent}`,
      distance: hud.time.textContent, message: hud.message.hidden ? '' : hud.message.textContent,
      visible: view.visible, camera: view.camera.position.toArray(), ...slot.state(), ...view.gpu() }),
  };
}
