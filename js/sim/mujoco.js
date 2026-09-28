import { fetchBuffer } from '../util.js';

let module = null;

export function loadMuJoCo(base = new URL('../../vendor/mujoco/', import.meta.url).href) {
  if (!module) {
    module = Promise.all([import(`${base}mujoco370.js`), fetchBuffer(`${base}mujoco370.wasm.gz`)])
      .then(([{ default: factory }, wasm]) => factory({ wasmBinary: wasm }));
    module.catch(() => { module = null; });
  }
  return module;
}

// Field ids of mjl_ptr in tools/mujoco_wasm/mjlite.c.
const PTR = { qpos: 0, qvel: 1, ctrl: 2, sensordata: 3, site_xpos: 4, site_xmat: 5, mocap_pos: 6, mocap_quat: 7,
  geom_xpos: 10, geom_size: 13, geom_rbound: 14, geom_aabb: 15, timestep: 16, geom_xmat: 22 };

export class MjSim {
  constructor(M, mjb) {
    this.M = M;
    const p = M._malloc(mjb.length);
    M.HEAPU8.set(mjb, p);
    const ok = M._mjl_load(p, mjb.length);
    M._free(p);
    if (!ok) throw new Error('MuJoCo could not load the model: ' + M.UTF8ToString(M._mjl_lastwarn()));
    [this.nq, this.nv, this.nu, this.nsensordata, this.nsite, this.nmocap, this.ngeom] = [0, 1, 2, 3, 4, 5, 6].map(i => M._mjl_dim(i));
    this.views();
  }

  views() {
    const M = this.M, view = (k, n) => new Float64Array(M.HEAPF64.buffer, M._mjl_ptr(PTR[k]), n);
    this.buffer = M.HEAPF64.buffer;
    this.qpos = view('qpos', this.nq);
    this.qvel = view('qvel', this.nv);
    this.ctrl = view('ctrl', this.nu);
    this.sensordata = view('sensordata', this.nsensordata);
    this.site_xpos = view('site_xpos', this.nsite * 3);
    this.site_xmat = view('site_xmat', this.nsite * 9);
    this.mocap_pos = view('mocap_pos', this.nmocap * 3);
    this.mocap_quat = view('mocap_quat', this.nmocap * 4);
    this.geom_xpos = view('geom_xpos', this.ngeom * 3);
    this.geom_xmat = view('geom_xmat', this.ngeom * 9);
    this.geom_size = view('geom_size', this.ngeom * 3);
    this.geom_rbound = view('geom_rbound', this.ngeom);
    this.geom_aabb = view('geom_aabb', this.ngeom * 6);
    this.timestep = view('timestep', 1);
  }

  // WebAssembly memory moves when it grows, so the views are rebuilt.
  check() {
    if (this.M.HEAPF64.buffer !== this.buffer) this.views();
    return this;
  }

  reset() { this.M._mjl_reset(); this.check(); }

  forward() { this.M._mjl_forward(); this.check(); }

  step(n) {
    const bad = this.M._mjl_step(n);
    this.check();
    return bad;
  }
}
