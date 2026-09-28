import { el, canvas2d } from './util.js';

const SPACE = '#8a8983', TIME = '#11917e';

export class Hud {
  constructor(stage) {
    this.step = el('b');
    this.atGoal = el('b');
    this.dist = el('b');
    this.state = el('div', { class: 'hud-state' });
    this.meter = el('i');
    this.time = el('b', { text: '–' });
    this.message = el('div', { class: 'hud-message', hidden: true });
    this.spark = el('canvas', { class: 'hud-spark' });
    this.spaceLabel = el('span', { text: 'distance in space' });
    const swatch = (color) => { const i = el('i'); i.style.background = color; return i; };
    this.host = el('div', { class: 'hud', hidden: true },
      el('div', { class: 'hud-box' }, el('div', {}, 'Step ', this.step, ' · at goal ', this.atGoal, ' · ', this.dist),
        el('div', { class: 'hud-meter' }, this.meter), this.state),
      el('div', { class: 'hud-box' }, el('div', {}, 'Temporal distance ', this.time, el('span', { class: 'hud-unit', text: ' horizons' })),
        this.message, this.spark,
        el('div', { class: 'hud-legend' }, el('span', {}, swatch(SPACE), this.spaceLabel), el('span', {}, swatch(TIME), 'in time (critic)'))));
    stage.append(this.host);
  }

  update({ step, atGoal, dist, meter, state, ready, hist, message, spaceLabel = 'distance in space' }) {
    const set = (node, text) => { if (node.textContent !== text) node.textContent = text; };
    this.host.hidden = false;
    set(this.step, String(step));
    set(this.atGoal, String(atGoal));
    set(this.dist, dist);
    this.meter.style.width = `${Math.min(100, 100 * meter)}%`;
    set(this.state, state);
    set(this.time, ready && hist.length ? hist[hist.length - 1].d.toFixed(2) : '–');
    set(this.message, ready ? '' : message);
    this.message.hidden = ready || !message;
    set(this.spaceLabel, spaceLabel);
    this.drawSpark(ready ? hist : []);
  }

  drawSpark(hist) {
    const { x, W, H } = canvas2d(this.spark, 190, 46);
    x.clearRect(0, 0, W, H);
    if (hist.length < 2) return;
    const t0 = hist[0].t, t1 = Math.max(hist[hist.length - 1].t, t0 + 1);
    for (const [k, color] of [['e', SPACE], ['d', TIME]]) {
      const v = hist.map(p => p[k]).sort((a, b) => a - b), top = Math.max(1e-9, v[Math.floor(.95 * (v.length - 1))]);
      x.beginPath();
      hist.forEach((p, i) => {
        const px = (p.t - t0) / (t1 - t0) * (W - 4) + 2, py = Math.max(1, H - 3 - p[k] / top * (H - 8));
        if (i) x.lineTo(px, py); else x.moveTo(px, py);
      });
      x.strokeStyle = color;
      x.lineWidth = k === 'd' ? 2 : 1.5;
      x.stroke();
    }
  }
}

const ICONS = {
  click: '<svg viewBox="0 0 24 24" aria-hidden="true"><path d="M9 3.5v2M4.6 5.6l1.4 1.4M3 10h2M13.4 5.6 12 7" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round"/><path d="M9.2 9.2l10.3 4.1-4.4 1.6-1.6 4.4z" fill="currentColor" stroke="currentColor" stroke-width="1.4" stroke-linejoin="round"/></svg>',
  drive: '<svg viewBox="0 0 24 24" aria-hidden="true"><rect x="9" y="3" width="6" height="6" rx="1.5" fill="none" stroke="currentColor" stroke-width="1.7"/><rect x="2.5" y="12" width="6" height="6" rx="1.5" fill="none" stroke="currentColor" stroke-width="1.7"/><rect x="9" y="12" width="6" height="6" rx="1.5" fill="none" stroke="currentColor" stroke-width="1.7"/><rect x="15.5" y="12" width="6" height="6" rx="1.5" fill="none" stroke="currentColor" stroke-width="1.7"/><path d="M12 7.6V4.8M10.8 6l1.2-1.2L13.2 6" fill="none" stroke="currentColor" stroke-width="1.3" stroke-linecap="round" stroke-linejoin="round"/></svg>',
};

export class HintPill {
  constructor(stage, seconds = 7) {
    this.seconds = seconds;
    this.icon = el('span', { class: 'hint-icon' });
    this.text = el('span');
    this.el = el('div', { class: 'hint-pill gone', role: 'status', 'aria-live': 'polite' }, this.icon, this.text);
    stage.append(this.el);
    this.inView = false;
    this.timer = 0;
    new IntersectionObserver((entries) => {
      this.inView = entries[entries.length - 1].isIntersecting;
      if (this.inView) this.arm();
      else { clearTimeout(this.timer); this.timer = 0; }
    }, { threshold: .5 }).observe(stage);
  }

  show(text, icon) {
    this.text.textContent = text;
    this.icon.innerHTML = ICONS[icon];
    this.el.classList.remove('gone');
    clearTimeout(this.timer);
    this.timer = 0;
    this.arm();
  }

  arm() {
    if (this.el.classList.contains('gone') || !this.inView || this.timer) return;
    this.timer = setTimeout(() => this.done(), this.seconds * 1000);
  }

  done() {
    clearTimeout(this.timer);
    this.timer = 0;
    this.el.classList.add('gone');
  }
}

export const pointerVerb = () => matchMedia('(pointer: coarse)').matches ? 'Tap' : 'Click';
