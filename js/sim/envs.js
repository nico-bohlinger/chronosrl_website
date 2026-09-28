import { System, init, step, toTau, rotate, quatMul, cross, quatTo3x3 } from './brax.js';

function gauss(rng) {
  let u = 0, v = 0;
  while (u === 0) u = rng();
  while (v === 0) v = rng();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
}

export class Env {
  constructor(json) {
    this.json = json;
    this.sys = new System(json);
    this.task = json.task;
    this.kind = json.task.kind;
    this.humanoid = this.kind !== 'ant_maze';
    this.nu = this.sys.nu;
    this.goalDim = json.goal_idx[1] - json.goal_idx[0];
    this.obsDim = json.state_dim + this.goalDim;
    this.state = null;
  }

  resetState(rng = Math.random) {
    const s = this.sys, T = this.task;
    const q = Float64Array.from(s.initQ), qd = new Float64Array(s.qdSize);
    const noise = T.reset_noise;
    if (this.kind === 'ant_maze') {
      for (let i = 0; i < q.length; i++) q[i] += (rng() * 2 - 1) * noise;
      for (let i = 0; i < qd.length; i++) qd[i] = noise * gauss(rng);
    } else if (noise > 0) {
      for (let i = 0; i < q.length; i++) q[i] += (rng() * 2 - 1) * noise;
      for (let i = 0; i < qd.length; i++) qd[i] = (rng() * 2 - 1) * noise;
    }
    if (this.kind === 'humanoid_maze') {
      const st = T.starts[Math.floor(rng() * T.starts.length)];
      q[0] = st[0]; q[1] = st[1];
    }
    const tg = this.sampleGoal(rng);
    q[q.length - 2] = tg[0]; q[q.length - 1] = tg[1];
    qd[qd.length - 2] = 0; qd[qd.length - 1] = 0;
    return [q, qd];
  }

  sampleGoal(rng = Math.random) {
    const T = this.task;
    if (this.kind === 'humanoid') {
      const d = T.min_goal_dist + rng() * (T.max_goal_dist - T.min_goal_dist), a = 2 * Math.PI * rng();
      return [d * Math.cos(a), d * Math.sin(a)];
    }
    return T.goals[Math.floor(rng() * T.goals.length)];
  }

  reset(q, qd) {
    this.state = init(this.sys, q, qd);
    this.lastAction = new Float64Array(this.nu);
    this.obs = this.observe(this.lastAction);
    return this.obs;
  }

  setGoal(xy) {
    const st = this.state, n = st.q.length;
    const q = Float64Array.from(st.q), qd = Float64Array.from(st.qd);
    q[n - 2] = xy[0]; q[n - 1] = xy[1]; qd[qd.length - 2] = 0; qd[qd.length - 1] = 0;
    this.state = init(this.sys, q, qd);
    this.obs = this.observe(this.lastAction);
  }

  goal() {
    const q = this.state.q, n = q.length;
    return [q[n - 2], q[n - 1]];
  }

  step(action) {
    let a = action;
    if (this.humanoid) {
      const c = this.sys.act.ctrl;
      a = new Float64Array(this.nu);
      for (let k = 0; k < this.nu; k++) a[k] = (action[k] + 1) * (c[k][1] - c[k][0]) * 0.5 + c[k][0];
    }
    for (let f = 0; f < this.sys.nFrames; f++) step(this.sys, this.state, a);
    this.lastAction = a;
    this.obs = this.observe(a);
    const z = this.state.x[0].pos[2], [lo, hi] = this.task.healthy_z;
    const done = z < lo || z > hi;
    const dist = this.goalDist();
    return { obs: this.obs, done, dist, success: dist < this.task.success_radius };
  }

  goalDist() {
    const o = this.obs, gd = this.goalDim;
    let s = 0;
    for (let k = 0; k < gd; k++) { const d = o[k] - o[o.length - gd + k]; s += d * d; }
    return Math.sqrt(s);
  }

  observe(action) {
    const st = this.state, s = this.sys;
    if (!this.humanoid) {
      const n = st.q.length, m = st.qd.length;
      const o = new Float64Array(this.obsDim);
      for (let i = 0; i < n - 2; i++) o[i] = st.q[i];
      for (let i = 0; i < m - 2; i++) o[n - 2 + i] = st.qd[i];
      const tp = st.x[s.L - 1].pos;
      o[n - 2 + m - 2] = tp[0]; o[n - 2 + m - 1] = tp[1];
      return o;
    }
    const L = s.L, o = [];
    for (const v of st.q) o.push(v);
    for (const v of st.qd) o.push(v);
    const mass = s.link.map(l => Math.pow(l.mass, 1 - s.J.spring_mass_scale));
    const idiag = s.link.map(l => l.idiag.map(v => Math.pow(v, 1 - s.J.spring_inertia_scale)));
    let msum = 0; for (const m of mass) msum += m;
    const xi = [], com = [0, 0, 0];
    for (let i = 0; i < L; i++) {
      const x = st.x[i], l = s.link[i];
      const pos = [x.pos[0], x.pos[1], x.pos[2]];
      const r = rotate(l.ipos, x.rot);
      const p = [pos[0] + r[0], pos[1] + r[1], pos[2] + r[2]];
      xi.push({ pos: p, rot: quatMul(x.rot, l.irot) });
      for (let k = 0; k < 3; k++) com[k] += mass[i] * p[k];
    }
    for (let k = 0; k < 3; k++) com[k] /= msum;
    for (let i = 0; i < L; i++) {
      const pos = [xi[i].pos[0] - com[0], xi[i].pos[1] - com[1], xi[i].pos[2] - com[2]];
      const R = quatTo3x3(xi[i].rot), I = idiag[i], m = mass[i];
      const RI = R.map(row => [row[0] * I[0], row[1] * I[1], row[2] * I[2]]);
      const M = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
      for (let a = 0; a < 3; a++) for (let b = 0; b < 3; b++) M[a][b] = RI[a][0] * R[b][0] + RI[a][1] * R[b][1] + RI[a][2] * R[b][2];
      const h = [cross(pos, [-1, 0, 0]), cross(pos, [0, -1, 0]), cross(pos, [0, 0, -1])];
      for (let a = 0; a < 3; a++) for (let b = 0; b < 3; b++) M[a][b] += (h[a][0] * h[b][0] + h[a][1] * h[b][1] + h[a][2] * h[b][2]) * m;
      for (let a = 0; a < 3; a++) for (let b = 0; b < 3; b++) o.push(M[a][b]);
      o.push(m);
    }
    for (let i = 0; i < L; i++) {
      const x = st.x[i], xd = st.xd[i];
      const off = [xi[i].pos[0] - x.pos[0], xi[i].pos[1] - x.pos[1], xi[i].pos[2] - x.pos[2]];
      const c = cross(off, xd.ang);
      const vel = [xd.vel[0] - c[0], xd.vel[1] - c[1], xd.vel[2] - c[2]];
      o.push(mass[i] * vel[0] / msum, mass[i] * vel[1] / msum, mass[i] * vel[2] / msum, xd.ang[0], xd.ang[1], xd.ang[2]);
    }
    const tau = toTau(s, action, st.q, st.qd);
    for (const v of tau) o.push(v);
    const tp = st.x[L - 1].pos;
    o.push(tp[0], tp[1], this.task.target_z);
    return Float64Array.from(o);
  }
}
