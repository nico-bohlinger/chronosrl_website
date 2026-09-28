const EVENTS = ['wheel', 'touchstart', 'keydown', 'mousedown'];
let pin = null, observer = null;

export function pinSection(node, smooth) {
  pin?.stop();
  const t0 = performance.now();
  const offset = () => node.getBoundingClientRect().top - (parseFloat(getComputedStyle(node).scrollMarginTop) || 0);
  let lastY = scrollY, still = 0, settled = false, own = 0;
  const me = {
    resized() {
      const d = offset();
      if (settled && Math.abs(d) > 1) { own++; scrollBy({ top: d, behavior: 'instant' }); }
    },
    stop() {
      clearInterval(timer);
      observer.disconnect();
      removeEventListener('scroll', onScroll);
      for (const ev of EVENTS) removeEventListener(ev, me.stop, { capture: true });
      if (pin === me) pin = null;
    },
  };
  const onScroll = () => {
    if (own) { own--; return; }
    if (settled && Math.abs(offset()) > 3) me.stop();
  };
  pin = me;
  observer = observer || new ResizeObserver(() => pin?.resized());
  for (const ev of EVENTS) addEventListener(ev, me.stop, { once: true, passive: true, capture: true });
  addEventListener('scroll', onScroll, { passive: true });
  for (let n = node.previousElementSibling; n; n = n.previousElementSibling) observer.observe(n);
  const timer = setInterval(() => {
    if (performance.now() - t0 > 60000) { me.stop(); return; }
    if (settled) return;
    still = Math.abs(scrollY - lastY) < .5 ? still + 1 : 0;
    lastY = scrollY;
    const d = offset(), atEnd = innerHeight + scrollY >= document.documentElement.scrollHeight - 1;
    if (still > 2) {
      if (Math.abs(d) > 3 && !(atEnd && d > 0)) { own++; scrollBy({ top: d, behavior: smooth ? 'smooth' : 'instant' }); still = 0; }
      else settled = true;
    }
    if (performance.now() - t0 > 6000) settled = true;
  }, 50);
}
