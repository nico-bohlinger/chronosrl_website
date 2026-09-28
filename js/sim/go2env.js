import { MjSim } from './mujoco.js';

const clip = (x, lo, hi) => Math.min(Math.max(x, lo), hi);

export class Go2Env {
  constructor(M, meta, models, task) {
    this.meta = meta;
    this.task = task;
    this.velocity = task === 'velocity';
    this.mm = meta.models[task === 'box' ? 'plane_box' : 'plane'];
    this.sim = new MjSim(M, models[task === 'box' ? 'plane_box' : 'plane']);
    this.sim.timestep[0] = meta.timestep;
    this.radius = meta.goal_radius[task];
    this.obs = new Float32Array(this.velocity ? 47 : 48);
    this.lastAction = new Float64Array(12);
    this.step_count = 0;
    this.goal = this.velocity ? [0, 0, 0] : { x: 2, y: 0, heading: 0 };
    this.feetOn = new Float64Array(4);
    this.feetAir = new Float64Array(4);
    this.feetContact = new Float64Array(4);
    this.imuHeight = 0;
    this.start = { x: 0, y: 0, yaw: 0 };
    this.box = { cx: 3, cy: 0, yaw: 0, hx: meta.box.length / 2, hy: meta.box.width / 2, height: 0 };
    this.done = false;
  }

  setBox(cx, cy, yaw, height) {
    if (this.task !== 'box') return;
    const b = this.box, s = this.sim.check(), g = this.mm.box_geom, hh = Math.max(height / 2, 1e-4);
    Object.assign(b, { cx, cy, yaw, height });
    s.geom_size.set([b.hx, b.hy, hh], 3 * g);
    s.geom_rbound[g] = Math.hypot(b.hx, b.hy, hh);
    s.geom_aabb.set([0, 0, 0, b.hx, b.hy, hh], 6 * g);
    s.mocap_pos.set([cx, cy, hh], 3 * this.mm.box_mocap);
    s.mocap_quat.set([Math.cos(yaw / 2), 0, 0, Math.sin(yaw / 2)], 4 * this.mm.box_mocap);
  }

  updateContacts(advance) {
    const M = this.sim.M, n = M._mjl_dim(8), feet = this.mm.foot_geoms, grounds = [this.meta.floor_geom, this.mm.box_geom ?? -1];
    const buf = this.contactBuffer || (this.contactBuffer = M._malloc(24));
    this.feetContact.fill(0);
    for (let k = 0; k < n; k++) {
      M._mjl_contact(k, buf);
      const c = new Float64Array(M.HEAPF64.buffer, buf, 3), g1 = c[0], g2 = c[1];
      if (!(c[2] < 0)) continue;
      const f = feet.indexOf(grounds.includes(g1) ? g2 : grounds.includes(g2) ? g1 : -1);
      if (f >= 0) this.feetContact[f] = 1;
    }
    this.sim.check();
    if (!advance) return;
    for (let f = 0; f < 4; f++) {
      if (this.feetContact[f]) { this.feetOn[f] += this.meta.control_dt; this.feetAir[f] = 0; }
      else { this.feetOn[f] = 0; this.feetAir[f] += this.meta.control_dt; }
    }
  }

  criticState() {
    const s = this.sim, m = this.meta, q = s.qpos, v = s.qvel, X = s.site_xmat, i9 = 9 * m.imu_site;
    const rl = [];
    for (let j = 0; j < 12; j++) rl.push((q[7 + j] - m.nominal[j]) / 3.14);
    for (let j = 0; j < 12; j++) rl.push(v[6 + j] / 100);
    for (let j = 0; j < 12; j++) rl.push(this.lastAction[j] / 10);
    for (let f = 0; f < 4; f++) rl.push(this.feetContact[f] / 0.5 - 1);
    for (let f = 0; f < 4; f++) rl.push(clip(this.feetOn[f] / 2.5 - 1, -1, 1));
    for (let f = 0; f < 4; f++) rl.push(clip(this.feetAir[f] / 2.5 - 1, -1, 1));
    for (let j = 0; j < 3; j++) rl.push(clip(s.sensordata[m.velo_adr + j] / 10, -1, 1));
    for (let j = 0; j < 3; j++) rl.push(clip(s.sensordata[m.gyro_adr + j] / 50, -1, 1));
    for (let j = 0; j < 3; j++) rl.push(this.velocity && this.step_count > 0 ? this.goal[j] : 0);
    for (let j = 0; j < 3; j++) rl.push(-X[i9 + 6 + j]);
    rl.push(clip(this.imuHeight / 5 - 1, -1, 1));
    const yaw = this.yaw(), ph = this.step_count * m.clock_rate, hs = m.heading_scale;
    let ach, head, rel, goal;
    if (this.velocity) {
      ach = this.achieved();
      head = [Math.cos(yaw), Math.sin(yaw)];
      goal = this.goal.slice();
      rel = goal;
    } else {
      const st = this.start, c = Math.cos(st.yaw), sn = Math.sin(st.yaw), g = this.goal;
      const dx = q[0] - st.x, dy = q[1] - st.y, gx = g.x - st.x, gy = g.y - st.y, h = yaw - st.yaw;
      ach = [c * dx + sn * dy, -sn * dx + c * dy, hs * Math.cos(h), hs * Math.sin(h)];
      goal = [c * gx + sn * gy, -sn * gx + c * gy, hs * Math.cos(g.heading - st.yaw), hs * Math.sin(g.heading - st.yaw)];
      head = [Math.cos(h), Math.sin(h)];
      const d0 = goal[0] - ach[0], d1 = goal[1] - ach[1];
      rel = [Math.cos(h) * d0 + Math.sin(h) * d1, -Math.sin(h) * d0 + Math.cos(h) * d1, goal[2] - ach[2], goal[3] - ach[3]];
    }
    const state = Float32Array.from([...ach, ...head, ...rl.map(x => clip(Number.isFinite(x) ? x : 0, -10, 10)), Math.cos(ph), Math.sin(ph), ...rel]);
    return { state, goal: Float32Array.from(goal) };
  }

  setBoxHeight(h) {
    if (this.task !== 'box') return null;
    const s = this.sim.check(), b = this.box, q = s.qpos, old = b.height;
    if (!(Math.abs(h - old) > 1e-9)) return null;
    s.forward();
    const onBox = this.onBox();
    this.setBox(b.cx, b.cy, b.yaw, h);
    let dz = onBox ? h - old : 0;
    if (dz) { q[2] += dz; s.forward(); }
    const lift = this.penetration();
    if (lift > 0) { q[2] += lift; dz += lift; s.forward(); }
    if (dz) s.qvel[2] = 0;
    return dz;
  }

  onBox() {
    const s = this.sim, b = this.box, q = s.qpos, r = this.meta.foot_radius;
    if (b.height <= 1e-3) return false;
    let onTop = false, onFloor = false;
    for (const g of this.mm.foot_geoms) {
      const x = s.geom_xpos[3 * g], y = s.geom_xpos[3 * g + 1], z = s.geom_xpos[3 * g + 2] - r;
      if (this.insideBox(x, y)) onTop = onTop || Math.abs(z - b.height) < 0.03;
      else onFloor = onFloor || z < 0.03;
    }
    return onTop || (!onFloor && this.insideBox(q[0], q[1]) && q[2] > b.height);
  }

  penetration() {
    const s = this.sim, b = this.box, r = this.meta.foot_radius;
    let need = 0;
    for (const g of this.mm.foot_geoms) {
      const x = s.geom_xpos[3 * g], y = s.geom_xpos[3 * g + 1], z = s.geom_xpos[3 * g + 2] - r;
      need = Math.max(need, (this.insideBox(x, y, r) ? b.height : 0) - z);
    }
    for (const [g, rad, half] of this.mm.calf_geoms) {
      const P = s.geom_xpos, M = s.geom_xmat, i = 3 * g, j = 9 * g;
      for (const sgn of [-1, 1]) {
        const x = P[i] + sgn * half * M[j + 2], y = P[i + 1] + sgn * half * M[j + 5], z = P[i + 2] + sgn * half * M[j + 8] - rad;
        if (this.insideBox(x, y, rad)) need = Math.max(need, b.height - z);
      }
    }
    return need > 0.01 ? need : 0;
  }

  insideBox(x, y, margin = 0) {
    if (this.task !== 'box') return false;
    const b = this.box, dx = x - b.cx, dy = y - b.cy;
    const lx = Math.cos(b.yaw) * dx + Math.sin(b.yaw) * dy, ly = -Math.sin(b.yaw) * dx + Math.cos(b.yaw) * dy;
    return Math.abs(lx) <= b.hx + margin && Math.abs(ly) <= b.hy + margin;
  }

  groundHeight(x, y) { return this.insideBox(x, y) ? this.box.height : 0; }

  yaw() {
    const q = this.sim.qpos, w = q[3], x = q[4], y = q[5], z = q[6];
    return Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  }

  reset(x = 0, y = 0, yaw = 0) {
    const s = this.sim, m = this.meta, q = s.qpos;
    s.reset();
    this.setBox(this.box.cx, this.box.cy, this.box.yaw, this.box.height);
    q.set(m.home);
    q.set([x, y, m.home[2], Math.cos(yaw / 2), 0, 0, Math.sin(yaw / 2)]);
    for (let j = 0; j < 12; j++) q[7 + j] = clip(m.nominal[j], m.joint_range[j][0], m.joint_range[j][1]);
    s.qvel.fill(0);
    s.forward();
    let lift = -Infinity;
    for (const g of this.mm.foot_geoms) {
      const fx = s.geom_xpos[3 * g], fy = s.geom_xpos[3 * g + 1], fz = s.geom_xpos[3 * g + 2] - m.foot_radius;
      lift = Math.max(lift, this.groundHeight(fx, fy) - fz);
    }
    q[2] += lift;
    s.forward();
    this.step_count = 0;
    this.lastAction.fill(0);
    this.done = false;
    this.feetOn.fill(0);
    this.feetAir.fill(0);
    this.updateContacts(false);
    this.imuHeight = this.imuOverGround();
    this.start = { x: q[0], y: q[1], yaw: this.yaw() };
    return this.observe();
  }

  imuOverGround() {
    const p = this.sim.site_xpos, i = 3 * this.meta.imu_site;
    return p[i + 2] - this.groundHeight(p[i], p[i + 1]);
  }

  setCommand(vx, vy, wz) { this.goal = [vx, vy, wz]; }

  setGoal(x, y, heading) {
    const q = this.sim.qpos;
    this.goal = { x, y, heading };
    this.start = { x: q[0], y: q[1], yaw: this.yaw() };
  }

  achieved() {
    const q = this.sim.qpos, v = this.sim.qvel, yaw = this.yaw();
    if (!this.velocity) return { x: q[0], y: q[1], yaw };
    const c = Math.cos(yaw), s = Math.sin(yaw);
    return [c * v[0] + s * v[1], -s * v[0] + c * v[1], v[5]];
  }

  goalError() {
    const a = this.achieved();
    if (this.velocity) return Math.hypot(a[0] - this.goal[0], a[1] - this.goal[1], a[2] - this.goal[2]);
    const g = this.goal, hs = this.meta.heading_scale;
    return Math.hypot(g.x - a.x, g.y - a.y, hs * (Math.cos(g.heading) - Math.cos(a.yaw)), hs * (Math.sin(g.heading) - Math.sin(a.yaw)));
  }

  atGoal() { return this.goalError() < this.radius; }

  observe() {
    const s = this.sim, m = this.meta, q = s.qpos, v = s.qvel, o = this.obs, X = s.site_xmat, i9 = 9 * m.imu_site;
    let k = 0;
    const put = (x) => { o[k++] = clip(Number.isFinite(x) ? x : 0, -10, 10); };
    for (let j = 0; j < 12; j++) put((q[7 + j] - m.nominal[j]) / 3.14);
    for (let j = 0; j < 12; j++) put(v[6 + j] / 100);
    for (let j = 0; j < 12; j++) put(this.lastAction[j] / 10);
    for (let j = 0; j < 3; j++) put(clip(s.sensordata[m.gyro_adr + j] / 50, -1, 1));
    for (let j = 0; j < 3; j++) put(-X[i9 + 6 + j]);
    const ph = this.step_count * m.clock_rate;
    o[k++] = Math.cos(ph);
    o[k++] = Math.sin(ph);
    if (this.velocity) {
      for (let j = 0; j < 3; j++) o[k++] = this.goal[j];
    } else {
      const yaw = this.yaw(), c = Math.cos(yaw), sn = Math.sin(yaw), g = this.goal, hs = m.heading_scale;
      const dx = g.x - q[0], dy = g.y - q[1], rel = g.heading - yaw;
      o[k++] = c * dx + sn * dy;
      o[k++] = -sn * dx + c * dy;
      o[k++] = hs * (Math.cos(rel) - 1);
      o[k++] = hs * Math.sin(rel);
    }
    for (let j = 0; j < o.length; j++) if (!Number.isFinite(o[j])) o[j] = 0;
    return o;
  }

  step(a) {
    const s = this.sim, m = this.meta, ph = this.step_count * m.clock_rate;
    for (let j = 0; j < 12; j++) {
      const act = clip(a[j], -1, 1) + m.gait_amplitude * m.gait_gain[j] * Math.sin(ph + m.gait_phase[j]);
      this.lastAction[j] = act;
      s.ctrl[j] = m.nominal[j] + m.action_scale * act;
    }
    this.setBox(this.box.cx, this.box.cy, this.box.yaw, this.box.height);
    const bad = s.step(m.substeps), v = s.qvel, q = s.qpos;
    let big = false, finite = bad === 0;
    for (let k = 0; k < v.length; k++) {
      v[k] = clip(v[k], -100, 100);
      if (Math.abs(v[k]) >= 100) big = true;
      if (!Number.isFinite(v[k])) finite = false;
    }
    for (let k = 0; k < q.length; k++) if (!Number.isFinite(q[k])) finite = false;
    const height = this.imuHeight = this.imuOverGround();
    this.updateContacts(true);
    this.step_count += 1;
    this.done = height < 0 || big || !finite;
    this.endCause = height < 0 ? 'fall' : big ? 'velocity' : !finite ? 'unstable' : null;
    return this.observe();
  }
}
