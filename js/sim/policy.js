function halfToFloat(h) {
  const out = new Float32Array(h.length);
  for (let i = 0; i < h.length; i++) {
    const x = h[i], sgn = x & 0x8000 ? -1 : 1, e = (x >> 10) & 0x1f, f = x & 0x3ff;
    out[i] = e === 0 ? sgn * f * 2 ** -24 : e === 31 ? (f ? NaN : sgn * Infinity) : sgn * (1 + f / 1024) * 2 ** (e - 15);
  }
  return out;
}

function parse(buffer) {
  const u8 = new Uint8Array(buffer);
  if (String.fromCharCode(...u8.subarray(0, 4)) !== 'CSRL') throw new Error('not a ChronoSRL policy file');
  const n = new DataView(buffer).getUint32(8, true);
  const header = JSON.parse(new TextDecoder().decode(u8.subarray(12, 12 + n)));
  const base = 12 + n, nets = {};
  let width = 0;
  for (const [name, prog] of Object.entries(header.nets)) {
    nets[name] = prog.map(op => {
      if (op.op === 'dense') {
        width = Math.max(width, op.in, op.out);
        const w = op.fmt === 'f16' ? halfToFloat(new Uint16Array(buffer, base + op.w, op.in * op.out)) : new Int8Array(buffer, base + op.w, op.in * op.out);
        const s = op.fmt === 'f16' ? new Float32Array(op.out).fill(1) : new Float32Array(buffer, base + op.s, op.out);
        return { op: 'dense', n: op.in, m: op.out, w, s, b: new Float32Array(buffer, base + op.b, op.out) };
      }
      if (op.op === 'ln') return { op: 'ln', n: op.n, g: new Float32Array(buffer, base + op.g, op.n), b: new Float32Array(buffer, base + op.b, op.n) };
      return { op: op.op };
    });
  }
  delete header.nets;
  return { header, nets, width };
}

export class Policy {
  constructor(actorBuffer) {
    const f = parse(actorBuffer);
    this.header = f.header;
    this.nets = f.nets;
    this.alloc(f.width);
    this.critic = null;
  }

  alloc(width) {
    if (this.bufA && this.bufA.length >= width) return;
    this.bufA = new Float32Array(width);
    this.bufB = new Float32Array(width);
    this.saved = new Float32Array(width);
  }

  attachCritic(buffer) {
    const f = parse(buffer);
    Object.assign(this.nets, f.nets);
    this.alloc(f.width);
    this.critic = f.header;
  }

  run(name, x) {
    let cur = this.bufA, nxt = this.bufB, n = x.length;
    for (let i = 0; i < n; i++) cur[i] = x[i];
    for (const op of this.nets[name]) {
      if (op.op === 'dense') {
        const W = op.w, S = op.s, B = op.b, N = op.n, M = op.m;
        for (let o = 0; o < M; o++) {
          let a0 = 0, a1 = 0, a2 = 0, a3 = 0, i = 0;
          const r = o * N;
          for (; i + 3 < N; i += 4) {
            a0 += W[r + i] * cur[i]; a1 += W[r + i + 1] * cur[i + 1];
            a2 += W[r + i + 2] * cur[i + 2]; a3 += W[r + i + 3] * cur[i + 3];
          }
          for (; i < N; i++) a0 += W[r + i] * cur[i];
          nxt[o] = (a0 + a1 + a2 + a3) * S[o] + B[o];
        }
        const t = cur; // a destructuring swap here makes WebKit run this function about 20 times slower
        cur = nxt;
        nxt = t;
        n = M;
      } else if (op.op === 'ln') {
        let mean = 0, sq = 0;
        for (let i = 0; i < n; i++) { const v = cur[i]; mean += v; sq += v * v; }
        mean /= n;
        const inv = 1 / Math.sqrt(Math.max(0, sq / n - mean * mean) + 1e-6), G = op.g, B = op.b;
        for (let i = 0; i < n; i++) cur[i] = (cur[i] - mean) * inv * G[i] + B[i];
      } else if (op.op === 'swish') {
        for (let i = 0; i < n; i++) { const v = cur[i]; cur[i] = v / (1 + Math.exp(-v)); }
      } else if (op.op === 'save') {
        this.saved.set(cur.subarray(0, n));
      } else if (op.op === 'add') {
        for (let i = 0; i < n; i++) cur[i] += this.saved[i];
      }
    }
    if (cur !== this.bufA) this.bufA.set(cur.subarray(0, n));
    return this.bufA.subarray(0, n);
  }

  act(obs) {
    const mean = this.run('actor', obs), out = new Float64Array(mean.length);
    for (let i = 0; i < mean.length; i++) out[i] = Math.tanh(mean[i]);
    return out;
  }

  distance(state, chunk, goalSequence) {
    const sa = new Float32Array(state.length + chunk.length);
    sa.set(state);
    sa.set(chunk, state.length);
    const zsa = Float32Array.from(this.run('enc_sa', sa)), zg = this.run('enc_g', goalSequence);
    let s = 0;
    for (let i = 0; i < zsa.length; i++) { const d = zsa[i] - zg[i]; s += d * d; }
    return Math.sqrt(s);
  }

  goalSequence(goal) {
    const { m, goal_const: c } = this.critic, g = new Float32Array(goal.length * m + (c ? 1 : 0));
    for (let r = 0; r < m; r++) g.set(goal, r * goal.length);
    if (c) g[g.length - 1] = 1;
    return g;
  }

  observationDistance(obs, chunk, goalDim) {
    const state = obs.subarray(0, this.critic.state_dim);
    return this.distance(state, chunk, this.goalSequence(obs.slice(obs.length - goalDim)));
  }
}
