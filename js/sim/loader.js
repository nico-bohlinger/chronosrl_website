import { Policy } from './policy.js';
import { fetchBuffer, fetchYielding, readStream, retryDelay, sleep } from '../util.js';

export const MANIFEST = 'assets/policies/manifest.json';
export const envFile = (task) => `assets/env/${task}.json`;
export const policyFile = (task, depth, seed) => `assets/policies/${task}/d${depth}_s${seed}.bin`;

export function prefetch(file) {
  const listeners = [];
  let fraction = 0;
  const onProgress = (f) => { fraction = f; for (const cb of listeners) cb(f); };
  const buffer = fetch(file).then(r => r.ok ? readStream(r, onProgress) : null).catch(() => null);
  return { file, buffer, progress: (cb) => { listeners.push(cb); cb(fraction); } };
}

export class PolicySlot {
  constructor({ status, onChange, gate = () => null, prefetched = null, cacheSize = 3 }) {
    this.status = status;
    this.onChange = onChange;
    this.gate = gate;
    this.prefetched = prefetched;
    this.cacheSize = cacheSize;
    this.cache = new Map();
    this.token = 0;
    this.criticToken = 0;
    this.criticTimer = 0;
    this.policy = null;
    this.key = '';
    this.selected = '';
    this.critic = { state: 'none', key: '' };
  }

  get criticReady() { return this.critic.state === 'ready' && this.critic.key === this.key && !!this.policy?.critic; }

  criticMessage() {
    const c = this.critic;
    if (c.state === 'loading') return 'loading the critic…';
    if (c.state === 'error') return `the critic could not be loaded, retrying in ${Math.max(1, Math.ceil((c.retryAt - performance.now()) / 1000))} s`;
    return '';
  }

  clear() {
    this.token++;
    this.policy = null;
    this.key = '';
    this.resetCritic();
  }

  actor(file, onProgress) {
    let p = this.cache.get(file);
    if (p) { this.cache.delete(file); this.cache.set(file, p); return p; }
    const pre = this.prefetched;
    let buffer;
    if (pre?.file === file) {
      this.prefetched = null;
      pre.progress(onProgress);
      buffer = pre.buffer.then(b => b || fetchBuffer(file, onProgress));
    } else {
      buffer = fetchBuffer(file, onProgress);
    }
    p = buffer.then(b => new Policy(b));
    p.catch(() => { if (this.cache.get(file) === p) this.cache.delete(file); });
    this.cache.set(file, p);
    while (this.cache.size > this.cacheSize) this.cache.delete(this.cache.keys().next().value);
    return p;
  }

  async select(task, depth, seed) {
    const token = ++this.token, file = policyFile(task, depth, seed), current = () => token === this.token;
    this.selected = file;
    this.resetCritic();
    for (let attempt = 0; ; attempt++) {
      this.status(`Loading the ${depth}-layer policy…`);
      try {
        const policy = await this.actor(file, (f) => { if (current()) this.status(`Loading the ${depth}-layer policy… ${Math.round(100 * f)}%`); });
        if (!current()) return null;
        const h = policy.header;
        if (h.task !== task || h.depth !== depth || h.seed !== seed) {
          this.status('This policy does not match the task.');
          return null;
        }
        this.policy = policy;
        this.key = file;
        this.status('');
        this.loadCritic(policy, file);
        return policy;
      } catch (err) {
        console.warn(err);
        if (!current()) return null;
        const wait = retryDelay(attempt);
        this.status(`The policy could not be loaded, retrying in ${wait} s`);
        await sleep(wait);
        if (!current()) return null;
      }
    }
  }

  resetCritic() {
    this.criticToken++;
    clearTimeout(this.criticTimer);
    this.critic = { state: 'none', key: '' };
    this.onChange();
  }

  loadCritic(policy, file, attempt = 0) {
    const token = this.criticToken;
    if (policy.critic) {
      this.critic = { state: 'ready', key: file };
      return;
    }
    this.critic = { state: 'loading', key: file };
    fetchYielding(file.replace(/\.bin$/, '.critic.bin'), this.gate).then(b => {
      if (!policy.critic) policy.attachCritic(b);
      if (token !== this.criticToken) return;
      this.critic = { state: 'ready', key: file };
      this.onChange();
    }).catch(err => {
      if (token !== this.criticToken) return;
      console.warn(err);
      const wait = retryDelay(attempt);
      this.critic = { state: 'error', key: file, retryAt: performance.now() + wait * 1000 };
      this.criticTimer = setTimeout(() => { if (token === this.criticToken) this.loadCritic(policy, file, attempt + 1); }, wait * 1000);
    });
  }

  state() {
    return { selected: this.selected, key: this.key, critic: this.critic.state, criticReady: this.criticReady,
      policy: this.policy && { task: this.policy.header.task, depth: this.policy.header.depth, seed: this.policy.header.seed } };
  }
}
