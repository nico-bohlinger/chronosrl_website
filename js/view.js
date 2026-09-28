import * as THREE from 'three';
import { OrbitControls } from '../vendor/three/OrbitControls.js';
import { el, isDark, onThemeChange } from './util.js';

export const TEAL = 0x11917e;

function floorTexture(dark, { px, step, major, minor }) {
  const c = document.createElement('canvas');
  c.width = c.height = px;
  const x = c.getContext('2d');
  const line = (k, color, width) => {
    x.strokeStyle = color;
    x.lineWidth = width;
    x.beginPath(); x.moveTo(k, 0); x.lineTo(k, px); x.stroke();
    x.beginPath(); x.moveTo(0, k); x.lineTo(px, k); x.stroke();
  };
  x.fillStyle = dark ? '#2a2e2d' : '#eef1ef';
  x.fillRect(0, 0, px, px);
  for (let k = 0; k <= px; k += step) line(k, major[+dark], 2);
  if (minor) for (let k = step / 2; k < px; k += step) line(k, minor[+dark], 1);
  const t = new THREE.CanvasTexture(c);
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.colorSpace = THREE.SRGBColorSpace;
  return t;
}

function cooperativeGestures(controls, canvas, stage) {
  controls.touches = { ONE: null, TWO: THREE.TOUCH.DOLLY_ROTATE };
  canvas.style.touchAction = 'pan-y';
  const mac = /Mac|iPhone|iPad|iPod/.test(navigator.platform || navigator.userAgent || '');
  const hint = el('div', { class: 'gesture-hint', 'aria-hidden': 'true' });
  stage.append(hint);
  let timer = 0;
  const show = (text) => {
    hint.textContent = text;
    hint.classList.add('on');
    clearTimeout(timer);
    timer = setTimeout(() => hint.classList.remove('on'), 1300);
  };
  stage.addEventListener('wheel', (e) => {
    if (e.ctrlKey || e.metaKey) return;
    e.stopPropagation();
    if (e.target === canvas) show(`Use ${mac ? '⌘' : 'Ctrl'} + scroll to zoom`);
  }, { capture: true, passive: true });
  let start = null;
  canvas.addEventListener('touchstart', (e) => { start = e.touches.length === 1 ? [e.touches[0].clientX, e.touches[0].clientY] : null; }, { passive: true });
  canvas.addEventListener('touchmove', (e) => {
    if (e.touches.length >= 2) { if (e.cancelable) e.preventDefault(); return; }
    const t = e.touches[0];
    if (start && Math.hypot(t.clientX - start[0], t.clientY - start[1]) > 12) { show('Use two fingers to move the view'); start = null; }
  }, { passive: false });
}

export class View {
  constructor(stage, o) {
    this.stage = stage;
    this.o = o;
    const r = this.renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: 'high-performance' });
    r.setPixelRatio(Math.min(2, devicePixelRatio || 1));
    r.shadowMap.enabled = true;
    r.shadowMap.type = THREE.PCFShadowMap;
    r.outputColorSpace = THREE.SRGBColorSpace;
    stage.prepend(r.domElement);
    const scene = this.scene = new THREE.Scene();
    scene.background = new THREE.Color();
    scene.fog = new THREE.Fog(0, ...o.fog);
    const cam = this.camera = new THREE.PerspectiveCamera(o.fov, 16 / 10, o.near, o.far);
    cam.up.set(0, 0, 1);
    const c = this.controls = new OrbitControls(cam, r.domElement);
    c.enableDamping = true;
    c.dampingFactor = .09;
    c.maxPolarAngle = Math.PI * .47;
    c.minDistance = o.minDistance;
    c.maxDistance = o.maxDistance;
    cooperativeGestures(c, r.domElement, stage);
    scene.add(new THREE.HemisphereLight(0xffffff, o.hemi[0], o.hemi[1]));
    const sun = this.sun = new THREE.DirectionalLight(0xffffff, o.sun);
    sun.castShadow = true;
    sun.shadow.mapSize.set(o.shadow.size, o.shadow.size);
    sun.shadow.bias = o.shadow.bias ?? 0;
    const e = o.shadow.extent;
    Object.assign(sun.shadow.camera, { left: -e, right: e, top: e, bottom: -e, near: .5, far: o.shadow.far });
    scene.add(sun, sun.target);
    this.floor = new THREE.Mesh(new THREE.PlaneGeometry(o.floor.size, o.floor.size), new THREE.MeshStandardMaterial({ roughness: .95 }));
    this.floor.receiveShadow = true;
    scene.add(this.floor);
    this.trail = { n: 400, count: 0, pos: new Float32Array(1200) };
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(this.trail.pos, 3));
    geo.setDrawRange(0, 0);
    this.trailLine = new THREE.Line(geo, new THREE.LineBasicMaterial({ color: TEAL, transparent: true, opacity: .55 }));
    this.trailLine.frustumCulled = false;
    scene.add(this.trailLine);
    this.dark = null;
    this.setDark(isDark());
    onThemeChange(() => this.setDark(isDark()));
    new ResizeObserver(() => this.resize()).observe(stage);
    this.resize();
    this.visible = true;
    new IntersectionObserver(([entry]) => { this.visible = entry.isIntersecting; }).observe(stage);
    this.onPick = null;
    this.picking();
  }

  setDark(dark) {
    if (dark === this.dark) return;
    this.dark = dark;
    this.scene.background.set(this.o.background[+dark]);
    this.scene.fog.color.set(this.o.background[+dark]);
    const f = this.floor.material;
    f.map?.dispose();
    f.map = floorTexture(dark, this.o.floor);
    f.map.anisotropy = this.o.floor.anisotropy;
    f.map.repeat.set(this.o.floor.repeat, this.o.floor.repeat);
    f.needsUpdate = true;
  }

  resize() {
    const w = this.stage.clientWidth, h = this.stage.clientHeight;
    if (!w || !h) return;
    this.renderer.setSize(w, h, false);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
  }

  clearTrail() {
    this.trail.count = 0;
    this.trailLine.geometry.setDrawRange(0, 0);
  }

  pushTrail(x, y) {
    const t = this.trail;
    if (t.count === t.n) { t.pos.copyWithin(0, 3); t.count--; }
    t.pos.set([x, y, this.o.trailZ], 3 * t.count++);
    this.trailLine.geometry.attributes.position.needsUpdate = true;
    this.trailLine.geometry.setDrawRange(0, t.count);
  }

  moveCamera(dx, dy, dz = 0) {
    const t = this.controls.target, p = this.camera.position;
    t.x += dx; t.y += dy; t.z += dz;
    p.x += dx; p.y += dy; p.z += dz;
  }

  picking() {
    const canvas = this.renderer.domElement, ray = new THREE.Raycaster(), ndc = new THREE.Vector2();
    const floor = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0), hit = new THREE.Vector3();
    let down = null;
    canvas.addEventListener('pointerdown', (e) => { down = { x: e.clientX, y: e.clientY, id: e.pointerId, t: performance.now() }; });
    canvas.addEventListener('pointercancel', () => { down = null; });
    canvas.addEventListener('pointerup', (e) => {
      const d = down;
      down = null;
      if (!d || d.id !== e.pointerId || Math.hypot(e.clientX - d.x, e.clientY - d.y) > 6 || performance.now() - d.t > 500 || !this.onPick) return;
      const r = canvas.getBoundingClientRect();
      ndc.set((e.clientX - r.left) / r.width * 2 - 1, -(e.clientY - r.top) / r.height * 2 + 1);
      ray.setFromCamera(ndc, this.camera);
      if (ray.ray.intersectPlane(floor, hit)) this.onPick(hit.x, hit.y);
    });
  }

  render() {
    if (!this.visible) return;
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }

  gpu() {
    const i = this.renderer.info;
    return { geometries: i.memory.geometries, textures: i.memory.textures, programs: i.programs?.length ?? 0 };
  }
}
