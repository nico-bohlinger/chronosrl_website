// brax 0.10.1's spring pipeline with the MJX collisions of mujoco 3.2.6, ported line by line: the policies need these exact dynamics.
const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const scl = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
export const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const neg = (a) => [-a[0], -a[1], -a[2]];
const any3 = (a) => a[0] !== 0 || a[1] !== 0 || a[2] !== 0;
const allZero = (a) => { for (const v of a) if (Math.abs(v) > 1e-8) return false; return true; };
function safeNorm(a) {
  if (allZero(a)) return 0;
  let s = 0; for (const v of a) s += v * v;
  return Math.sqrt(s);
}
function normalizeWithNorm(a) {
  const n = safeNorm(a), d = n + (n === 0 ? 1e-6 : 0);
  return [a.map(v => v / d), n];
}
const normalize = (a) => normalizeWithNorm(a)[0];

export function rotate(v, q) {
  const s = q[0], u = [q[1], q[2], q[3]];
  const uv = dot(u, v), uu = dot(u, u), c = cross(u, v);
  return [2 * uv * u[0] + (s * s - uu) * v[0] + 2 * s * c[0],
          2 * uv * u[1] + (s * s - uu) * v[1] + 2 * s * c[1],
          2 * uv * u[2] + (s * s - uu) * v[2] + 2 * s * c[2]];
}
const quatInv = (q) => [q[0], -q[1], -q[2], -q[3]];
const invRotate = (v, q) => rotate(v, quatInv(q));
export function quatMul(u, v) {
  return [u[0] * v[0] - u[1] * v[1] - u[2] * v[2] - u[3] * v[3],
          u[0] * v[1] + u[1] * v[0] + u[2] * v[3] - u[3] * v[2],
          u[0] * v[2] - u[1] * v[3] + u[2] * v[0] + u[3] * v[1],
          u[0] * v[3] + u[1] * v[2] - u[2] * v[1] + u[3] * v[0]];
}
const quatRotAxis = (axis, angle) => { const s = Math.sin(angle / 2); return [Math.cos(angle / 2), axis[0] * s, axis[1] * s, axis[2] * s]; };
export function quatTo3x3(q) {
  const d = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
  const [w, x, y, z] = q, s = 2 / d;
  const xs = x * s, ys = y * s, zs = z * s, wx = w * xs, wy = w * ys, wz = w * zs;
  const xx = x * xs, xy = x * ys, xz = x * zs, yy = y * ys, yz = y * zs, zz = z * zs;
  return [[1 - (yy + zz), xy - wz, xz + wy], [xy + wz, 1 - (xx + zz), yz - wx], [xz - wy, yz + wx, 1 - (xx + yy)]];
}
const matVec = (m, v) => [dot(m[0], v), dot(m[1], v), dot(m[2], v)];
const matTVec = (m, v) => [m[0][0] * v[0] + m[1][0] * v[1] + m[2][0] * v[2],
                          m[0][1] * v[0] + m[1][1] * v[1] + m[2][1] * v[2],
                          m[0][2] * v[0] + m[1][2] * v[1] + m[2][2] * v[2]];
const signedAngle = (axis, p, c) => Math.atan2(dot(cross(p, c), axis), dot(p, c));
const clip = (v, lo, hi) => v < lo ? lo : v > hi ? hi : v;
const sign = (v) => v > 0 ? 1 : v < 0 ? -1 : 0;

const tDo = (a, b) => ({ pos: add(a.pos, rotate(b.pos, a.rot)), rot: quatMul(a.rot, b.rot) });
const toLocal = (a, t) => { const ti = quatInv(t.rot); return { pos: rotate(sub(a.pos, t.pos), ti), rot: quatMul(ti, a.rot) }; };
const IDQ = [1, 0, 0, 0], ZERO = [0, 0, 0];

const QW = { f: 7, 1: 1, 2: 2, 3: 3 }, QDW = { f: 6, 1: 1, 2: 2, 3: 3 };

export class System {
  constructor(J) {
    this.J = J;
    const types = J.link_types;
    this.L = types.length;
    this.dt = J.dt;
    this.nFrames = J.n_frames;
    this.gravity = J.gravity;
    this.velDamping = J.vel_damping; this.angDamping = J.ang_damping;
    this.erp = J.baumgarte_erp;
    const L = J.link;
    this.link = [];
    let qa = 0, qda = 0;
    for (let i = 0; i < this.L; i++) {
      const t = types[i];
      const imat = L.i_mat[i];
      this.link.push({
        type: t, parent: J.link_parents[i], qAdr: qa, qdAdr: qda, n: QDW[t],
        transform: { pos: L.t_pos[i], rot: L.t_rot[i] }, joint: { pos: L.j_pos[i], rot: L.j_rot[i] },
        ipos: L.i_pos[i], irot: L.i_rot[i], idiag: [imat[0], imat[4], imat[8]], imat,
        mass: L.mass[i], cs: L.cs[i], cvd: L.cvd[i], cls: L.cls[i], cad: L.cad[i],
        jf: J.joint_frames[i],
      });
      qa += QW[t]; qda += QDW[t];
    }
    this.qdSize = qda;
    this.lo = J.dof.lo; this.hi = J.dof.hi;
    for (const l of this.link) {
      const ang = J.dof.ang.slice(l.qdAdr, l.qdAdr + l.n), vel = J.dof.vel.slice(l.qdAdr, l.qdAdr + l.n);
      l.ang = ang; l.vel = vel;
      l.isTrans = vel.some(any3); l.isRot = ang.some(any3);
    }
    this.massScaled = this.link.map(l => Math.pow(l.mass, 1 - J.spring_mass_scale));
    this.inertiaScale = J.spring_inertia_scale;
    this.act = J.actuator;
    this.nu = this.act.q_id.length;
    this.geoms = J.geoms.map(g => ({ ...g }));
    this.pairs = J.pairs;
    this.box = J.box;
    this.initQ = J.init_q;
  }
}

function forward(sys, q, qd) {
  const L = sys.L, x = new Array(L), xd = new Array(L);
  for (let i = 0; i < L; i++) {
    const l = sys.link[i];
    let j, jd;
    if (l.type === 'f') {
      const a = l.qAdr, b = l.qdAdr;
      j = { pos: [q[a], q[a + 1], q[a + 2]], rot: [q[a + 3], q[a + 4], q[a + 5], q[a + 6]] };
      jd = { ang: [qd[b + 3], qd[b + 4], qd[b + 5]], vel: [qd[b], qd[b + 1], qd[b + 2]] };
    } else {
      for (let d = 0; d < l.n; d++) {
        const qq = q[l.qAdr + d], qv = qd[l.qdAdr + d];
        const jdd = { pos: scl(l.vel[d], qq), rot: normalize(quatRotAxis(l.ang[d], qq)) };
        const jddd = { ang: scl(l.ang[d], qv), vel: scl(l.vel[d], qv) };
        if (d === 0) { j = jdd; jd = jddd; }
        else {
          j = tDo(j, jdd);
          jd = { ang: add(jd.ang, rotate(jddd.ang, jdd.rot)),
                 vel: add(jd.vel, rotate(add(jddd.vel, cross(jdd.pos, jddd.ang)), jdd.rot)) };
        }
      }
    }
    const anchorPos = rotate(l.joint.pos, j.rot);
    j = { pos: add(sub(add(j.pos, l.joint.pos), anchorPos), [0, 0, 0]), rot: j.rot };
    j = tDo(l.transform, j);
    if (l.parent < 0) {
      x[i] = j; xd[i] = { ang: rotate(jd.ang, j.rot), vel: jd.vel };
    } else {
      const xp = x[l.parent], xdp = xd[l.parent];
      const xi = tDo(xp, j);
      const vel = add(add(xdp.vel, cross(xdp.ang, sub(xi.pos, xp.pos))), rotate(jd.vel, xp.rot));
      const ang = add(xdp.ang, rotate(jd.ang, xi.rot));
      x[i] = xi; xd[i] = { ang, vel };
    }
  }
  for (let i = 0; i < L; i++) x[i] = { pos: x[i].pos, rot: normalize(x[i].rot) };
  return [x, xd];
}

function worldToJoint(sys, x, xd) {
  const L = sys.L, j = new Array(L), jd = new Array(L), ap = new Array(L), ac = new Array(L);
  for (let i = 0; i < L; i++) {
    const l = sys.link[i];
    const xp = l.parent >= 0 ? x[l.parent] : { pos: ZERO, rot: IDQ };
    const xdp = l.parent >= 0 ? xd[l.parent] : { ang: ZERO, vel: ZERO };
    const a_p = tDo(tDo(xp, l.transform), l.joint);
    const a_c = tDo(x[i], l.joint);
    j[i] = toLocal(a_c, a_p);
    const off = sub(xp.pos, a_p.pos);
    const wjAng = xdp.ang, wjVel = sub(xdp.vel, cross(off, xdp.ang));
    jd[i] = { ang: invRotate(sub(xd[i].ang, wjAng), a_p.rot), vel: invRotate(sub(xd[i].vel, wjVel), a_p.rot) };
    ap[i] = a_p; ac[i] = a_c;
  }
  return [j, jd, ap, ac];
}

function axisAngleAng(j, jf, parity) {
  const cf = [rotate(jf.ang[0], j.rot), rotate(jf.ang[1], j.rot), rotate(jf.ang[2], j.rot)];
  const lon = normalize(cross(cf[2], jf.ang[0]));
  const psi = signedAngle(jf.ang[0], jf.ang[1], lon);
  let a1 = add(scl(cf[0], dot(jf.ang[0], cf[0])), scl(cf[1], dot(jf.ang[0], cf[1])));
  a1 = normalize(a1);
  const theta = Math.acos(clip(dot(a1, jf.ang[0]), -1, 1)) * sign(dot(jf.ang[0], cf[2]));
  const yc = scl(cf[2], -parity);
  const phi = signedAngle(yc, cf[1], lon);
  return [[cf[0], cf[1], scl(cf[2], parity)], [psi, theta, phi]];
}

function inverse(sys, j, jd, q, qd) {
  for (let i = 0; i < sys.L; i++) {
    const l = sys.link[i];
    if (l.type === 'f') {
      const a = l.qAdr, b = l.qdAdr, J = j[i];
      q[a] = J.pos[0]; q[a + 1] = J.pos[1]; q[a + 2] = J.pos[2];
      q[a + 3] = J.rot[0]; q[a + 4] = J.rot[1]; q[a + 5] = J.rot[2]; q[a + 6] = J.rot[3];
      const ang = invRotate(jd[i].ang, J.rot);
      qd[b] = jd[i].vel[0]; qd[b + 1] = jd[i].vel[1]; qd[b + 2] = jd[i].vel[2];
      qd[b + 3] = ang[0]; qd[b + 4] = ang[1]; qd[b + 5] = ang[2];
      continue;
    }
    const jRot = l.parent === -1 ? j[i].rot : IDQ;
    const jdAng = invRotate(jd[i].ang, jRot);
    const [axis, angles] = axisAngleAng(j[i], l.jf, l.jf.parity);
    for (let d = 0; d < l.n; d++) {
      const rotational = any3(l.ang[d]);
      q[l.qAdr + d] = rotational ? angles[d] : dot(l.vel[d], j[i].pos);
      qd[l.qdAdr + d] = rotational ? dot(axis[d], jdAng) : dot(l.vel[d], jd[i].vel);
    }
  }
}

function comFromWorld(sys, x, xd) {
  const xi = [], xdi = [];
  for (let i = 0; i < sys.L; i++) {
    const pos = add(x[i].pos, rotate(sys.link[i].ipos, x[i].rot));
    xi.push({ pos, rot: x[i].rot });
    xdi.push({ ang: xd[i].ang, vel: sub(xd[i].vel, cross(sub(pos, x[i].pos), xd[i].ang)) });
  }
  return [xi, xdi];
}
function comToWorld(sys, xi, xdi) {
  const x = [], xd = [];
  for (let i = 0; i < sys.L; i++) {
    const pos = add(xi[i].pos, rotate(neg(sys.link[i].ipos), xi[i].rot));
    x.push({ pos, rot: xi[i].rot });
    xd.push({ ang: xdi[i].ang, vel: sub(xdi[i].vel, cross(sub(pos, xi[i].pos), xdi[i].ang)) });
  }
  return [x, xd];
}
function invInertia(sys, x) {
  const out = [];
  for (let i = 0; i < sys.L; i++) {
    const l = sys.link[i];
    const ri = quatMul(x[i].rot, l.irot);
    const d = l.idiag.map(v => 1 / Math.pow(v, 1 - sys.inertiaScale));
    const rows = [rotate([d[0], 0, 0], ri), rotate([0, d[1], 0], ri), rotate([0, 0, d[2]], ri)];
    const cols = [0, 1, 2].map(k => rotate([rows[0][k], rows[1][k], rows[2][k]], ri));
    out.push(cols);
  }
  return out;
}

export function toTau(sys, act, q, qd) {
  const tau = new Float64Array(sys.qdSize), A = sys.act;
  for (let k = 0; k < sys.nu; k++) {
    const a = clip(act[k], A.ctrl[k][0], A.ctrl[k][1]);
    const bias = A.gear[k] * (q[A.q_id[k]] * A.bias_q[k] + qd[A.qd_id[k]] * A.bias_qd[k]);
    let f = A.gain[k] * a + bias;
    f = clip(f, A.force[k][0], A.force[k][1]);
    tau[A.qd_id[k]] += f * A.gear[k];
  }
  return tau;
}

function oneDof(l, j, jd, tau) {
  const jf = l.jf, T = l.isTrans ? 1 : 0, Rr = l.isRot ? 1 : 0;
  let vel = scl(j.pos, -l.cs);
  vel = sub(vel, scl(jf.vel[0], dot(jf.vel[0], vel) * T));
  vel = add(vel, scl(jf.vel[0], tau[0] * T));
  const damp = scl(jd.vel, -l.cvd);
  vel = add(vel, sub(damp, scl(jf.vel[0], dot(jf.vel[0], damp) * T)));
  const axisCX = rotate(jf.ang[0], j.rot), axisCY = rotate(jf.ang[1], j.rot);
  const psi = axisAngleAng(j, jf, jf.parity)[1][0];
  let ang = scl(cross(jf.ang[0], axisCX), -l.cs);
  ang = sub(ang, scl(cross(jf.ang[1], axisCY), l.cs * T * (1 - Rr)));
  ang = add(ang, scl(jf.ang[0], tau[0] * Rr));
  ang = sub(ang, scl(jd.ang, l.cad));
  const lo = l.lo[0], hi = l.hi[0];
  let dang = psi < lo ? psi - lo : 0; if (psi > hi) dang = psi - hi;
  ang = sub(ang, scl(jf.ang[0], l.cls * dang * (1 - T)));
  const xp = dot(j.pos, jf.vel[0]);
  let dvel = xp < lo ? xp - lo : 0; if (xp > hi) dvel = xp - hi;
  vel = sub(vel, scl(jf.vel[0], l.cls * dvel * T));
  return { ang, vel };
}

function twoDof(l, j, jd, tau) {
  const jf = l.jf, T = l.isTrans ? 1 : 0, U = l.isRot ? 1 : 0, mv = l.vel, ma = l.ang;
  let vel = scl(j.pos, -l.cs);
  vel = add(vel, scl(jd.vel, -l.cvd));
  vel = sub(vel, scl(add(scl(mv[0], dot(vel, mv[0])), scl(mv[1], dot(vel, mv[1]))), T));
  const [axisC, angles] = axisAngleAng(j, jf, jf.parity);
  const axis1 = jf.ang[0], axis2 = axisC[1];
  let axisCProj = sub(axis2, scl(axis1, dot(axis2, axis1)));
  axisCProj = scl(axisCProj, 1 / safeNorm(axisCProj));
  const axisCX = rotate(jf.ang[0], j.rot), axisCY = rotate(jf.ang[1], j.rot);
  const a0any = any3(ma[0]);
  const axisCCand = a0any ? jf.ang[0] : jf.ang[1];
  let torqueAxis2 = a0any ? axisCX : axisCY;
  if (T) axisCProj = axisCCand; else torqueAxis2 = axis2;
  let ang = scl(cross(axisCProj, torqueAxis2), -l.cls);
  const angAxis = [scl(axis1, any3(ma[0]) ? 1 : 0), scl(axis2, any3(ma[1]) ? 1 : 0)];
  ang = add(ang, add(scl(angAxis[0], tau[0]), scl(angAxis[1], tau[1])));
  vel = add(vel, add(scl(mv[0], tau[0]), scl(mv[1], tau[1])));
  const axisCZ = rotate(jf.ang[2], j.rot);
  ang = sub(ang, scl(cross(jf.ang[2], axisCZ), l.cs * T * (1 - U)));
  for (let k = 0; k < 2; k++) {
    const a = angles[k], lo = l.lo[k], hi = l.hi[k];
    let dang = a < lo ? a - lo : 0; if (a > hi) dang = a - hi;
    ang = sub(ang, scl(angAxis[k], l.cls * dang * U));
  }
  for (let k = 0; k < 2; k++) {
    const xp = dot(j.pos, mv[k]), lo = l.lo[k], hi = l.hi[k];
    let dv = xp < lo ? xp - lo : 0; if (xp > hi) dv = xp - hi;
    vel = sub(vel, scl(mv[k], l.cls * dv * T));
  }
  ang = sub(ang, scl(jd.ang, l.cad));
  return { ang, vel };
}

function threeDof(l, j, jd, tau) {
  const jf = l.jf, T = l.isTrans ? 1 : 0, Rr = l.isRot ? 1 : 0, mv = l.vel, ma = l.ang;
  let vel = scl(j.pos, -l.cs);
  vel = add(vel, scl(jd.vel, -l.cvd));
  vel = sub(vel, scl(add(add(scl(mv[0], dot(mv[0], vel)), scl(mv[1], dot(mv[1], vel))), scl(mv[2], dot(mv[2], vel))), T));
  let ang = scl(jd.ang, -l.cad);
  const [axisC, angles] = axisAngleAng(j, jf, jf.parity);
  const axes = [jf.ang[0], axisC[1], axisC[2]];
  const angAxis = axes.map((a, k) => scl(a, any3(ma[k]) ? 1 : 0));
  for (let k = 0; k < 3; k++) { ang = add(ang, scl(angAxis[k], tau[k])); vel = add(vel, scl(mv[k], tau[k])); }
  if (T && Rr) {
    const sumA = add(add(ma[0], ma[1]), ma[2]);
    const sumR = add(add(rotate(ma[0], j.rot), rotate(ma[1], j.rot)), rotate(ma[2], j.rot));
    ang = add(ang, scl(cross(sumA, sumR), -l.cs));
  }
  for (let k = 0; k < 3; k++) {
    const a = angles[k], lo = l.lo[k], hi = l.hi[k];
    let dang = a < lo ? a - lo : 0; if (a > hi) dang = a - hi;
    ang = sub(ang, scl(angAxis[k], l.cls * dang));
  }
  if (T) {
    for (let k = 0; k < 3; k++) {
      const xp = dot(mv[k], j.pos), lo = l.lo[k], hi = l.hi[k];
      let dv = xp < lo ? xp - lo : 0; if (xp > hi) dv = xp - hi;
      vel = sub(vel, scl(mv[k], l.cls * dv));
    }
  }
  return { ang, vel };
}

function jointsResolve(sys, st, tau) {
  const L = sys.L, fang = Array.from({ length: L }, () => [0, 0, 0]), fvel = Array.from({ length: L }, () => [0, 0, 0]);
  for (let i = 0; i < L; i++) {
    const l = sys.link[i];
    if (l.type === 'f') continue;
    if (!l.lo) { l.lo = sys.lo.slice(l.qdAdr, l.qdAdr + l.n); l.hi = sys.hi.slice(l.qdAdr, l.qdAdr + l.n); }
    const t = Array.from(tau.subarray(l.qdAdr, l.qdAdr + l.n));
    const jf = l.type === '1' ? oneDof(l, st.j[i], st.jd[i], t) : l.type === '2' ? twoDof(l, st.j[i], st.jd[i], t) : threeDof(l, st.j[i], st.jd[i], t);
    const xfVel = rotate(jf.vel, st.ap[i].rot), xfAng = rotate(jf.ang, st.ap[i].rot);
    const fcAng = add(xfAng, cross(sub(st.ac[i].pos, st.xi[i].pos), xfVel));
    fang[i] = add(fang[i], fcAng); fvel[i] = add(fvel[i], xfVel);
    if (l.parent >= 0) {
      const p = l.parent;
      const fpAng = add(xfAng, cross(sub(st.ap[i].pos, st.xi[p].pos), xfVel));
      fang[p] = sub(fang[p], fpAng); fvel[p] = sub(fvel[p], xfVel);
    }
  }
  return [fang, fvel];
}

function makeFrameN(n) { return normalize(n); }

function planeSphere(P, S) {
  const n = [P.mat[0][2], P.mat[1][2], P.mat[2][2]], r = S.size[0];
  const dist = dot(sub(S.pos, P.pos), n) - r;
  const pos = sub(S.pos, scl(n, r + 0.5 * dist));
  return [[dist, pos, makeFrameN(n)]];
}
function planeCapsule(P, C) {
  const n = [P.mat[0][2], P.mat[1][2], P.mat[2][2]], axis = [C.mat[0][2], C.mat[1][2], C.mat[2][2]];
  const seg = scl(axis, C.size[1]), r = C.size[0], out = [];
  const frame0 = n;
  for (const off of [seg, neg(seg)]) {
    const c = add(C.pos, off);
    const dist = dot(sub(c, P.pos), n) - r;
    out.push([dist, sub(c, scl(n, r + 0.5 * dist)), frame0]);
  }
  return out;
}
function sphereSphereCore(p1, r1, p2, r2) {
  let [n, d] = normalizeWithNorm(sub(p2, p1));
  if (d === 0) n = [1, 0, 0];
  const dist = d - (r1 + r2);
  return [dist, add(p1, scl(n, r1 + dist * 0.5)), n];
}
function sphereSphere(A, B) { const [d, p, n] = sphereSphereCore(A.pos, A.size[0], B.pos, B.size[0]); return [[d, p, makeFrameN(n)]]; }
function closestSegmentPoint(a, b, pt) {
  const ab = sub(b, a);
  const t = dot(sub(pt, a), ab) / (dot(ab, ab) + 1e-6);
  return add(a, scl(ab, clip(t, 0, 1)));
}
function sphereCapsule(S, C) {
  const axis = [C.mat[0][2], C.mat[1][2], C.mat[2][2]], seg = scl(axis, C.size[1]);
  const pt = closestSegmentPoint(sub(C.pos, seg), add(C.pos, seg), S.pos);
  const [d, p, n] = sphereSphereCore(S.pos, S.size[0], pt, C.size[0]);
  return [[d, p, makeFrameN(n)]];
}
function closestSegToSeg(a0, a1, b0, b1) {
  const [dirA, lenA] = normalizeWithNorm(sub(a1, a0));
  const [dirB, lenB] = normalizeWithNorm(sub(b1, b0));
  const hA = lenA * 0.5, hB = lenB * 0.5;
  const aMid = add(a0, scl(dirA, hA)), bMid = add(b0, scl(dirB, hB));
  const trans = sub(aMid, bMid);
  const ab = dot(dirA, dirB), at = dot(dirA, trans), bt = dot(dirB, trans);
  const denom = 1 - ab * ab;
  const origTA = (-at + ab * bt) / (denom + 1e-6);
  const origTB = bt + origTA * ab;
  const tA = clip(origTA, -hA, hA), tB = clip(origTB, -hB, hB);
  let bestA = add(aMid, scl(dirA, tA)), bestB = add(bMid, scl(dirB, tB));
  const newA = closestSegmentPoint(a0, a1, bestB), dd1 = sub(bestB, newA), d1 = dot(dd1, dd1);
  const newB = closestSegmentPoint(b0, b1, bestA), dd2 = sub(bestA, newB), d2 = dot(dd2, dd2);
  if (d1 < d2) bestA = newA; else bestB = newB;
  return [bestA, bestB];
}
const projectPtOntoPlane = (pt, pp, n) => sub(pt, scl(n, dot(sub(pt, pp), n)));
function closestSegmentPointPlane(a, b, p0, n) {
  const d = dot(p0, n), denom = dot(n, sub(b, a));
  let t = (d - dot(n, a)) / (denom + (denom === 0 ? 1e-6 : 0));
  t = clip(t, 0, 1);
  return add(a, scl(sub(b, a), t));
}
function clipEdgeToPlanes(p0, p1, planePts, planeNormals) {
  const K = planePts.length;
  const p0In = [], p1In = [], cand = [];
  for (let k = 0; k < K; k++) {
    p0In.push(dot(sub(p0, planePts[k]), planeNormals[k]) > 1e-6);
    p1In.push(dot(sub(p1, planePts[k]), planeNormals[k]) > 1e-6);
    cand.push(closestSegmentPointPlane(p0, p1, planePts[k], planeNormals[k]));
  }
  const clipPt = (a, b, inFront) => {
    let best = null, bd = -Infinity;
    for (let k = 0; k < K; k++) {
      const e = inFront[k] ? cand[k] : a;
      const dd = dot(sub(e, a), sub(b, a));
      if (dd > bd) { bd = dd; best = e; } // argmax: the first maximum wins
    }
    return best;
  };
  const n0 = clipPt(p0, p1, p0In), n1 = clipPt(p1, p0, p1In);
  let mask = true;
  for (let k = 0; k < K; k++) if (p0In[k] && p1In[k]) { mask = false; break; }
  let newPs = mask ? [n0, n1] : [p0, p1];
  if (dot(sub(p0, p1), sub(newPs[0], newPs[1])) < 0) mask = false;
  return [newPs, mask];
}

function boxInfo(sys, G) {
  const B = sys.box, s = G.size;
  return {
    vert: B.vert.map(v => [v[0] * s[0], v[1] * s[1], v[2] * s[2]]),
    face: B.face.map(f => f.map(v => [v[0] * s[0], v[1] * s[1], v[2] * s[2]])),
    normal: B.face_normal, edge: B.edge, efn: B.edge_face_normal,
  };
}

function sphereBox(sys, S, G) {
  const cv = G._box || (G._box = boxInfo(sys, G));
  const r = S.size[0];
  const sp = matTVec(G.mat, sub(S.pos, G.pos));
  let bestIdx = 0, bestSup = -Infinity, hasSep = false;
  for (let k = 0; k < 6; k++) {
    const n = cv.normal[k];
    const sup = dot(sub(sub(sp, scl(n, r)), cv.face[k][0]), n);
    if (sup >= 0) hasSep = true;
    if (sup > bestSup) { bestSup = sup; bestIdx = k; }
  }
  const face = cv.face[bestIdx], fn = cv.normal[bestIdx];
  let pt = projectPtOntoPlane(sp, face[0], fn);
  const e0 = [face[3], face[0], face[1], face[2]], e1 = face;
  const sideN = e0.map((p, k) => cross(sub(e1[k], p), fn));
  let edgeDist = e0.map((p, k) => dot(sub(pt, p), sideN[k]));
  const onFace = edgeDist.every(v => v <= 0);
  edgeDist = edgeDist.map((v, k) => (!any3(sideN[k]) || v < 0) ? 1e12 : v);
  let idx = 0; for (let k = 1; k < 4; k++) if (edgeDist[k] < edgeDist[idx]) idx = k;
  const edgePt = closestSegmentPoint(e0[idx], e1[idx], pt);
  if (!onFace) pt = edgePt;
  let [ptN, d] = normalizeWithNorm(sub(pt, sp));
  const inside = dot(pt, ptN) > 0, sg = inside ? -1 : 1;
  const n = (onFace || d < 1e-6) ? neg(fn) : scl(ptN, sg);
  d *= sg;
  const spt = add(sp, scl(n, r));
  const dist = hasSep ? 1.0 : d - r;
  const pos = scl(add(pt, spt), 0.5);
  return [[dist, add(matVec(G.mat, pos), G.pos), makeFrameN(matVec(G.mat, n))]];
}

function capsuleBox(sys, C, G) {
  const cv = G._box || (G._box = boxInfo(sys, G));
  const r = C.size[0], len = C.size[1];
  const cp = matTVec(G.mat, sub(C.pos, G.pos));
  const axis = matTVec(G.mat, [C.mat[0][2], C.mat[1][2], C.mat[2][2]]);
  const seg = scl(axis, len);
  const cap = [sub(cp, seg), add(cp, seg)];
  let bestIdx = 0, bestSup = -Infinity, hasSupport = true;
  for (let k = 0; k < 6; k++) {
    const n = cv.normal[k];
    const s0 = dot(sub(sub(cap[0], scl(n, r)), cv.face[k][0]), n);
    const s1 = dot(sub(sub(cap[1], scl(n, r)), cv.face[k][0]), n);
    const sup = Math.min(s0, s1);
    if (!(sup < 0)) hasSupport = false;
    if (sup > bestSup) { bestSup = sup; bestIdx = k; }
  }
  const face = cv.face[bestIdx], normal = cv.normal[bestIdx];
  const e0 = [face[3], face[0], face[1], face[2]], e1 = face;
  const sidePlanes = e0.map((p, k) => cross(sub(e1[k], p), normal));
  let [clipped, mask] = clipEdgeToPlanes(cap[0], cap[1], e0, sidePlanes);
  clipped = clipped.map(p => sub(p, scl(normal, r)));
  const facePts = clipped.map(p => projectPtOntoPlane(p, face[0], normal));
  let pos = clipped.map((p, k) => scl(add(p, facePts[k]), 0.5));
  let nrm = [neg(normal), neg(normal)];
  const facePen = clipped.map((p, k) => (mask && hasSupport) ? dot(sub(facePts[k], p), normal) : -1);
  let eIdx = 0, eBest = null, eAbs = Infinity;
  for (let e = 0; e < cv.edge.length; e++) {
    const [a, b] = cv.edge[e];
    const [ec, cc] = closestSegToSeg(cv.vert[a], cv.vert[b], cap[0], cap[1]);
    const dir = sub(ec, cc);
    const degenerate = dot(dir, dir) < 1e-6;
    const [ax, dist] = normalizeWithNorm(dir);
    if (Math.abs(dist) < eAbs) { eAbs = Math.abs(dist); eIdx = e; eBest = { dist, ax, degenerate, ec, cc }; }
  }
  const efn = cv.efn[eIdx];
  const front = dot(efn[0], eBest.ax) < 0 && dot(efn[1], eBest.ax) < 0;
  const shallow = !eBest.degenerate && front;
  const edgePen = shallow ? r - eBest.dist : -1;
  const edgePos = scl(add(eBest.ec, add(eBest.cc, scl(eBest.ax, r))), 0.5);
  const parallel = Math.abs(dot(eBest.ax, normal)) > 0.99 && !eBest.degenerate;
  const minFace = Math.min(facePen[0], facePen[1]);
  const hasEdge = edgePen > 0 && (minFace > 0 ? edgePen < minFace : true) && !parallel && front;
  if (hasEdge) { pos = [edgePos, pos[1]]; nrm = [eBest.ax, nrm[1]]; }
  const dist = hasEdge ? [-edgePen, 1] : [-facePen[0], -facePen[1]];
  return [0, 1].map(k => [dist[k], add(G.pos, matVec(G.mat, pos[k])), makeFrameN(matVec(G.mat, nrm[k]))]);
}

const FN = {
  plane_sphere: (s, a, b) => planeSphere(a, b), plane_capsule: (s, a, b) => planeCapsule(a, b),
  sphere_sphere: (s, a, b) => sphereSphere(a, b), sphere_capsule: (s, a, b) => sphereCapsule(a, b),
  sphere_box: sphereBox, capsule_box: capsuleBox,
};

function geomPose(sys, x) {
  const out = sys.geoms.map(g => {
    const lx = g.link >= 0 ? x[g.link] : { pos: ZERO, rot: IDQ };
    const pos = add(lx.pos, rotate(g.pos, lx.rot));
    const mat = quatTo3x3(quatMul(lx.rot, g.quat));
    return { pos, mat, size: g.size, g };
  });
  return out;
}

function contacts(sys, x) {
  const G = geomPose(sys, x);
  for (let i = 0; i < G.length; i++) if (sys.geoms[i].link < 0) G[i]._box = sys.geoms[i]._box || (sys.geoms[i]._box = boxInfo(sys, G[i]));
  const out = [];
  for (const [fn, list] of Object.entries(sys.pairs)) {
    const f = FN[fn];
    const plane = fn.startsWith('plane');
    for (const [g1, g2] of list) {
      const A = G[g1], B = G[g2];
      // Exact culling: pairs this far apart have dist >= 0, which adds neither an impulse nor a contact.
      if (!plane) {
        const dx = A.pos[0] - B.pos[0], dy = A.pos[1] - B.pos[1], dz = A.pos[2] - B.pos[2];
        const rb = sys.geoms[g1].rbound + sys.geoms[g2].rbound;
        if (dx * dx + dy * dy + dz * dz > rb * rb) continue;
      }
      for (const [dist, pos, n] of f(sys, A, B)) {
        out.push({ dist, pos, n, g1, g2, link1: sys.geoms[g1].link, link2: sys.geoms[g2].link,
          friction: Math.max(sys.geoms[g1].friction, sys.geoms[g2].friction),
          elasticity: (sys.geoms[g1].elasticity + sys.geoms[g2].elasticity) * 0.5 });
      }
    }
  }
  return out;
}

function collisionsResolve(sys, st) {
  const L = sys.L, cs = contacts(sys, st.x);
  const pv = Array.from({ length: L }, () => [0, 0, 0]), pa = Array.from({ length: L }, () => [0, 0, 0]);
  const cnt = new Float64Array(L);
  for (const c of cs) {
    if (!(c.dist < 0)) continue;
    const li = [c.link1, c.link2];
    const n = c.n, mn = neg(n);
    const relPos = [0, 1].map(k => sub(c.pos, li[k] >= 0 ? st.xi[li[k]].pos : st.xi[L - 1].pos));
    const relVel = [0, 1].map(k => li[k] >= 0 ? add(st.xdi[li[k]].vel, cross(st.xdi[li[k]].ang, relPos[k])) : [0, 0, 0]);
    // brax 0.10.1 reads i_inv.take(link), an index into the flattened (L, 3, 3) array; kept as trained.
    const flatI = (l) => { const m = st.iinv[Math.floor(l / 9)]; const w = l % 9; return m[Math.floor(w / 3)][w % 3]; };
    const iinv = [0, 1].map(k => li[k] >= 0 ? flatI(li[k]) : 0);
    const imass = [0, 1].map(k => li[k] >= 0 ? 1 / st.mass[li[k]] : 0);
    const contactVel = sub(relVel[0], relVel[1]);
    const normalVel = dot(mn, contactVel);
    let angSum = [0, 0, 0];
    for (let k = 0; k < 2; k++) angSum = add(angSum, cross(scl(cross(relPos[k], mn), iinv[k]), relPos[k]));
    const ang = dot(mn, angSum);
    const baum = sys.erp / sys.dt * c.dist;
    const impulse = (-1 * (1 + c.elasticity) * normalVel - baum) / (imass[0] + imass[1] + ang);
    const applyN = c.dist < 0 && normalVel < 0 && impulse > 0;
    if (!applyN) continue;
    const impulseVec = scl(mn, impulse);
    const velD = add(contactVel, scl(n, normalVel));
    const nd = safeNorm(velD);
    const dirD = scl(velD, 1 / (1e-6 + nd));
    let angSumD = [0, 0, 0];
    for (let k = 0; k < 2; k++) angSumD = add(angSumD, cross(scl(cross(relPos[k], dirD), iinv[k]), relPos[k]));
    const angD = dot(dirD, angSumD);
    let impulseD = nd / (imass[0] + imass[1] + angD);
    impulseD = Math.min(impulseD, c.friction * impulse);
    const applyD = nd > 1e-3;
    const f = add(impulseVec, applyD ? scl(dirD, -impulseD) : [0, 0, 0]);
    for (let k = 0; k < 2; k++) {
      const l = li[k];
      if (l < 0) continue;
      const fk = k === 0 ? f : neg(f);
      pv[l] = add(pv[l], fk);
      pa[l] = add(pa[l], cross(sub(c.pos, st.xi[l].pos), fk));
      cnt[l] += 1;
    }
  }
  const dv = [], da = [];
  for (let l = 0; l < L; l++) {
    const k = cnt[l] + 1e-8;
    const v = scl(pv[l], 1 / k), a = scl(pa[l], 1 / k);
    dv.push(scl(v, 1 / st.mass[l]));
    da.push(matVec(st.iinv[l], a));
  }
  return [da, dv];
}

export function init(sys, q, qd) {
  const [x, xd] = forward(sys, q, qd);
  const [j, jd, ap, ac] = worldToJoint(sys, x, xd);
  const [xi, xdi] = comFromWorld(sys, x, xd);
  return { q: Float64Array.from(q), qd: Float64Array.from(qd), x, xd, j, jd, ap, ac, xi, xdi,
    iinv: invInertia(sys, x), mass: sys.massScaled };
}

export function step(sys, st, act) {
  st.iinv = invInertia(sys, st.x);
  const tau = toTau(sys, act, st.q, st.qd);
  const [fang, fvel] = jointsResolve(sys, st, tau);
  const dt = sys.dt, L = sys.L;
  for (let i = 0; i < L; i++) {
    const accAng = matVec(st.iinv[i], fang[i]);
    const accVel = add(sys.gravity, scl(fvel[i], 1 / st.mass[i]));
    st.xdi[i] = { ang: add(st.xdi[i].ang, scl(accAng, dt)), vel: add(st.xdi[i].vel, scl(accVel, dt)) };
  }
  const [dAng, dVel] = collisionsResolve(sys, st);
  const eV = Math.exp(sys.velDamping * dt), eA = Math.exp(sys.angDamping * dt);
  for (let i = 0; i < L; i++) {
    const vel = add(scl(st.xdi[i].vel, eV), dVel[i]), ang = add(scl(st.xdi[i].ang, eA), dAng[i]);
    const r = st.xi[i].rot;
    const qa = quatMul([0, ang[0] * 0.5 * dt, ang[1] * 0.5 * dt, ang[2] * 0.5 * dt], r);
    const rot = [r[0] + qa[0], r[1] + qa[1], r[2] + qa[2], r[3] + qa[3]];
    const nr = Math.hypot(rot[0], rot[1], rot[2], rot[3]);
    st.xi[i] = { pos: add(st.xi[i].pos, scl(vel, dt)), rot: rot.map(v => v / nr) };
    st.xdi[i] = { ang, vel };
  }
  const [x, xd] = comToWorld(sys, st.xi, st.xdi);
  st.x = x; st.xd = xd;
  const [j, jd, ap, ac] = worldToJoint(sys, x, xd);
  st.j = j; st.jd = jd; st.ap = ap; st.ac = ac;
  inverse(sys, j, jd, st.q, st.qd);
  return st;
}
