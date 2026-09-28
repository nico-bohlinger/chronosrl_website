import { DEPTHS, fetchJSON, fetchBuffer, el, row, button, segmented, animate } from '../util.js';
import { Hud, HintPill, pointerVerb } from '../hud.js';
import { MANIFEST, PolicySlot, policyFile, prefetch } from '../sim/loader.js';
import { loadMuJoCo } from '../sim/mujoco.js';
import { Go2Env } from '../sim/go2env.js';

const TASKS = [{ value: 'velocity', label: 'Velocity' }, { value: 'position', label: 'Position', title: 'Goal position' }, { value: 'box', label: 'Box climb' }];
const DT = 0.02;
const KEYS = { ArrowUp: [0, 1], KeyW: [0, 1], ArrowDown: [0, -1], KeyS: [0, -1], ArrowLeft: [2, 1], KeyA: [2, 1],
  ArrowRight: [2, -1], KeyD: [2, -1], KeyQ: [1, .6], KeyE: [1, -.6] };

const DEFAULT = { task: 'box', depth: 64 };

export async function loadAssets() {
  const manifest = await fetchJSON(MANIFEST);
  const prefetched = prefetch(policyFile(DEFAULT.task, DEFAULT.depth, manifest[DEFAULT.task][DEFAULT.depth]));
  const [M, meta] = await Promise.all([loadMuJoCo(), fetchJSON('assets/go2/sim.json')]);
  const models = {};
  await Promise.all(Object.entries(meta.models).map(async ([k, v]) => { models[k] = new Uint8Array(await fetchBuffer(`assets/go2/${v.file}`)); }));
  return { M, meta, models, manifest, prefetched };
}

export class Go2 {
  constructor({ view, panel, stage, assets }) {
    Object.assign(this, { view, panel, stage, A: assets });
    this.S = { task: DEFAULT.task, depth: DEFAULT.depth, speed: 1, paused: false, acc: 0, k: 0, cmd: [1, 0, 0], boxH: .14,
      atGoalSteps: 0, simTime: 0, cue: '', cueUntil: 0, episodes: 0, goals: 0, steps: 0, ends: { fall: 0, velocity: 0, unstable: 0 }, hist: [] };
    this.status = el('div', { class: 'sim-status', hidden: true });
    this.time = el('div', { class: 'stage-time', text: '0.0 s' });
    stage.append(this.status, this.time);
    this.hud = new Hud(stage);
    this.hint = new HintPill(stage);
    this.slot = new PolicySlot({ status: (t) => { this.status.textContent = t; this.status.hidden = !t; },
      onChange: () => { this.S.hist = []; this.lastStats = 0; }, prefetched: assets.prefetched, cacheSize: 2 });
    view.onPick = (x, y) => this.onClick(x, y);
    this.buildPanel();
    this.keys();
    this.setTask(this.S.task);
    animate((dt) => { if (view.visible) { this.tick(dt); view.render(); } });
  }

  buildPanel() {
    const S = this.S;
    this.taskSeg = segmented(TASKS, S.task, v => this.setTask(v), { label: 'Task' });
    this.taskSeg.el.classList.add('one-row');
    const depth = segmented(DEPTHS.map(d => ({ value: d, label: String(d) })), S.depth, (d) => { S.depth = d; this.selectPolicy(); },
      { small: true, label: 'Network depth' });
    this.controls = el('div', { class: 'sim-controls' });
    const pause = button('Pause', () => { S.paused = !S.paused; pause.textContent = S.paused ? 'Play' : 'Pause'; });
    const speed = button('1×', () => { S.speed = { 1: 2, 2: .5, .5: 1 }[S.speed]; speed.textContent = `${S.speed}×`; }, { title: 'Simulation speed' });
    const follow = button('Overview', () => {
      const goal = this.env.velocity ? null : this.env.goal;
      if (this.view.follow) { this.view.overview(this.curQ, goal); follow.textContent = 'Follow'; }
      else { this.view.chase(this.curQ, goal); follow.textContent = 'Overview'; }
    }, { title: 'Switch between following the robot and a view of the robot and its goal' });
    this.panel.append(row('Task', this.taskSeg.el), row('Network depth', depth.el), this.controls,
      el('div', { class: 'sim-actions' }, pause, button('↺', () => this.newEpisode(true), { title: 'New episode from the start' }), speed, follow));
  }

  buildControls() {
    const S = this.S, c = this.controls;
    c.replaceChildren();
    const slider = (label, min, max, step, value, format, onInput) => {
      const out = el('output', { text: format(value) }), input = el('input', { type: 'range', min, max, step, value, 'aria-label': label });
      input.addEventListener('input', () => { out.textContent = format(+input.value); onInput(+input.value); });
      c.append(el('div', { class: 'row slider' }, el('span', { text: label }), input, out));
      return (v) => { input.value = v; out.textContent = format(v); };
    };
    const buttons = (...b) => c.append(el('div', { class: 'row' }, el('span'), el('div', { class: 'sim-buttons' }, ...b)));
    this.sliders = null;
    if (S.task === 'velocity') {
      const drive = (i) => (v) => { S.cmd[i] = v; this.applyCommand(); this.hint.done(); };
      this.sliders = ['Forward (m/s)', 'Sideways (m/s)', 'Turn (rad/s)'].map((label, i) => slider(label, -1, 1, .05, S.cmd[i], v => v.toFixed(2), drive(i)));
      buttons(button('Random command', () => this.randomCommand()), button('Stop', () => this.setCommand([0, 0, 0])));
      this.hint.show(matchMedia('(pointer: coarse)').matches ? 'Drive with the sliders' : 'Drive with the sliders or the arrow keys', 'drive');
    } else {
      if (S.task === 'box') slider('Box height (cm)', 0, 20, .5, S.boxH * 100, v => v.toFixed(1), v => { S.boxH = v / 100; this.applyBoxHeight(); });
      buttons(button('Random goal', () => this.randomGoal()));
      this.hint.show(`${pointerVerb()} the floor to place the goal`, 'click');
    }
  }

  keys() {
    const command = (e, down) => {
      const k = KEYS[e.code] || KEYS[e.key];
      if (!k || this.S.task !== 'velocity') return;
      if (down) e.preventDefault();
      const c = this.S.cmd.slice();
      c[k[0]] = down ? k[1] : 0;
      this.setCommand(c);
      if (down) this.hint.done();
    };
    this.stage.tabIndex = 0;
    this.stage.addEventListener('keydown', (e) => command(e, true));
    this.stage.addEventListener('keyup', (e) => command(e, false));
  }

  setTask(task) {
    this.S.task = task;
    this.taskSeg.set(task);
    this.buildControls();
    this.slot.clear();
    this.chunk = null;
    this.env = new Go2Env(this.A.M, this.A.meta, this.A.models, task);
    this.newEpisode(true);
    return this.selectPolicy();
  }

  async selectPolicy() {
    const S = this.S;
    if (await this.slot.select(S.task, S.depth, this.A.manifest[S.task][S.depth])) {
      this.chunk = null;
      S.k -= S.k % 2;
    }
  }

  newEpisode(fromStart = false, cue = '') {
    const S = this.S, env = this.env;
    let x = 0, y = 0, yaw = 0;
    if (!fromStart) { x = env.sim.qpos[0]; y = env.sim.qpos[1]; yaw = env.yaw(); }
    if (S.task === 'box' && (fromStart || env.insideBox(x, y, .35))) this.placeGoal(x + 2.5 * Math.cos(yaw), y + 2.5 * Math.sin(yaw), yaw, true);
    const goal = env.velocity ? null : { ...env.goal };
    env.reset(x, y, yaw);
    if (goal) env.setGoal(goal.x, goal.y, goal.heading);
    Object.assign(S, { k: 0, atGoalSteps: 0, hist: [], cue, cueUntil: cue ? performance.now() + 900 : 0 });
    S.episodes++;
    this.chunk = null;
    if (fromStart) S.simTime = 0;
    if (S.task === 'velocity') this.applyCommand();
    if (S.task === 'position' && fromStart) this.randomGoal();
    this.snapPose(fromStart);
    this.view.clearTrail();
    this.syncMarkers();
  }

  snapPose(camera = false) {
    const q = this.env.sim.qpos;
    if (!this.prevQ) { this.prevQ = new Float64Array(19); this.curQ = new Float64Array(19); }
    this.prevQ.set(q);
    this.curQ.set(q);
    if (camera && this.view.follow) this.view.chase(this.curQ, this.env.velocity ? null : this.env.goal);
    this.view.setRobot(this.curQ);
  }

  placeGoal(gx, gy, heading, force = false) {
    const S = this.S, env = this.env;
    env.setGoal(gx, gy, heading);
    S.goals++;
    S.hist = [];
    if (S.task === 'box') {
      const q = env.sim.qpos, onOld = !force && env.insideBox(q[0], q[1], .35);
      env.setBox(gx, gy, heading - Math.PI / 2, S.boxH);
      if (onOld) {
        const x = q[0], y = q[1], yaw = env.yaw();
        env.reset(x, y, yaw);
        env.setGoal(gx, gy, heading);
        S.k = 0;
        this.chunk = null;
        this.snapPose();
      }
    }
    this.syncMarkers();
  }

  onClick(x, y) {
    if (!this.env || this.env.velocity) return;
    const q = this.env.sim.qpos, ang = Math.atan2(y - q[1], x - q[0]), r = Math.min(5, Math.max(1, Math.hypot(x - q[0], y - q[1])));
    this.placeGoal(q[0] + r * Math.cos(ang), q[1] + r * Math.sin(ang), ang);
    this.hint.done();
  }

  randomGoal() {
    const q = this.env.sim.qpos, d = 1 + 4 * Math.random(), ang = this.env.yaw() + 2 * Math.PI * Math.random();
    this.placeGoal(q[0] + d * Math.cos(ang), q[1] + d * Math.sin(ang), ang);
  }

  randomCommand() {
    let g = [0, 1, 2].map(() => -1 + 2 * Math.random()).map(v => (Math.abs(v) < .1 ? 0 : v));
    if (Math.random() < .04) g = [0, 0, 0];
    g = g.map(v => (Math.random() < .005 ? 0 : v));
    if (Math.random() < .04) g = [0, 0, 0];
    this.setCommand(g.map(v => Math.round(v * 20) / 20));
  }

  setCommand(c) {
    this.S.cmd = c.slice();
    this.sliders?.forEach((set, i) => set(c[i]));
    this.applyCommand();
  }

  applyCommand() {
    if (this.S.task !== 'velocity') return;
    this.env.setCommand(...this.S.cmd);
    this.S.hist = [];
  }

  applyBoxHeight() {
    const dz = this.env.setBoxHeight(this.S.boxH);
    if (dz) { this.prevQ[2] += dz; this.curQ[2] += dz; }
    this.syncMarkers();
  }

  syncMarkers() {
    const env = this.env, v = this.view;
    if (env.velocity) {
      v.setGoal(false);
      v.setBox(false);
      return;
    }
    const g = env.goal, b = env.box;
    v.setGoal(true, g.x, g.y, env.task === 'box' && env.insideBox(g.x, g.y) ? b.height : 0, g.heading);
    if (env.task === 'box') v.setBox(true, b.cx, b.cy, b.yaw, b.hx, b.hy, b.height); else v.setBox(false);
  }

  controlStep() {
    const S = this.S, env = this.env, policy = this.slot.policy;
    if (!policy || policy.header.task !== S.task) return;
    if (S.k % 2 === 0 || !this.chunk) {
      this.chunk = policy.act(env.obs);
      if (this.slot.criticReady && (S.k % 8 === 0 || !S.hist.length)) {
        const { state, goal } = env.criticState();
        S.hist.push({ t: S.k, d: policy.distance(state, this.chunk, policy.goalSequence(goal)), e: env.velocity ? env.goalError() : this.spaceDistance() });
        if (S.hist.length > 400) S.hist.shift();
      }
    }
    env.step(this.chunk.subarray(12 * (S.k % 2), 12 * (S.k % 2) + 12));
    S.k++;
    S.steps++;
    S.simTime += DT;
    this.prevQ.set(this.curQ);
    this.curQ.set(env.sim.qpos);
    if (env.done) {
      S.ends[env.endCause]++;
      this.newEpisode(false, env.endCause === 'fall' ? 'fell over, new episode' : 'episode ended, new episode');
      return;
    }
    if (env.atGoal()) S.atGoalSteps++;
    if (S.k % 2 === 0) this.view.pushTrail(env.sim.qpos[0], env.sim.qpos[1]);
  }

  spaceDistance() {
    const q = this.env.sim.qpos, g = this.env.goal;
    return Math.hypot(g.x - q[0], g.y - q[1]);
  }

  tick(dt) {
    const S = this.S;
    if (!S.paused) {
      S.acc += dt * S.speed;
      let n = 0;
      for (; S.acc >= DT && n < 8; n++) {
        this.controlStep();
        S.acc -= DT;
      }
      if (n === 8) S.acc = 0;
    }
    const u = Math.min(1, S.acc / DT), q = this.q || (this.q = new Float64Array(19));
    for (let i = 0; i < 19; i++) q[i] = this.prevQ[i] + (this.curQ[i] - this.prevQ[i]) * u;
    this.view.setRobot(q);
    this.view.setCommand(S.task === 'velocity' ? q : null, ...S.cmd);
    const now = performance.now();
    if (now - (this.lastStats || 0) > 100) { this.lastStats = now; this.renderStats(); }
  }

  renderStats() {
    const S = this.S, env = this.env, q = env.sim.qpos;
    this.time.textContent = `${(S.k * DT).toFixed(1)} s`;
    let state, dist;
    if (env.velocity) {
      dist = `error ${env.goalError().toFixed(2)}`;
      state = env.atGoal() ? 'on command' : 'following the command';
    } else {
      dist = `${this.spaceDistance().toFixed(1)} m`;
      state = env.atGoal() ? 'at the goal' : 'walking to the goal';
      if (env.insideBox(q[0], q[1])) state = q[2] - env.box.height > .2 ? (env.atGoal() ? 'on the box, at the goal' : 'on the box') : 'climbing';
    }
    if (!this.slot.policy) state = 'waiting for the policy';
    else if (S.cue && performance.now() < S.cueUntil) state = S.cue;
    this.hud.update({ step: S.k, atGoal: S.atGoalSteps, dist, meter: S.atGoalSteps / 1000, state, ready: this.slot.criticReady, hist: S.hist,
      message: this.slot.criticMessage(), spaceLabel: env.velocity ? 'command error' : 'distance in space' });
  }

  state() {
    const S = this.S, env = this.env;
    return { task: S.task, depth: S.depth, seed: this.A.manifest[S.task][S.depth], paused: S.paused, speed: S.speed, k: S.k, steps: S.steps,
      simTime: S.simTime, episodes: S.episodes, goals: S.goals, ends: { ...S.ends }, time: this.time.textContent, visible: this.view.visible,
      finite: env.sim.qpos.every(Number.isFinite), onBox: env.task === 'box' ? env.onBox() : null,
      hist: S.hist.length, histT: S.hist.length ? S.hist[S.hist.length - 1].t + 1e4 * S.episodes + 1e7 * S.goals : null,
      latest: S.hist.length ? S.hist[S.hist.length - 1].d.toFixed(2) : null, hud: `${this.hud.step.textContent}|${this.hud.state.textContent}`,
      distance: this.hud.time.textContent, message: this.hud.message.hidden ? '' : this.hud.message.textContent,
      goal: env.velocity ? null : [env.goal.x, env.goal.y], cmd: S.cmd.slice(), box: env.task === 'box' ? env.box.height : null,
      trail: this.view.trail.count, camera: this.view.camera.position.toArray(), ...this.slot.state(), ...this.view.gpu() };
  }
}
