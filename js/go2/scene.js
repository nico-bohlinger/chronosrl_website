import * as THREE from 'three';
import { View, TEAL } from '../view.js';
import { fetchJSON, fetchBuffer } from '../util.js';

const OPTIONS = {
  background: [0xe8ecea, 0x1f2423], fog: [9, 28], fov: 45, near: .05, far: 100, minDistance: 1, maxDistance: 20,
  hemi: [0x9aa3a0, 1.5], sun: 2.2, shadow: { size: 1024, extent: 4, far: 20 }, trailZ: .01,
  floor: { size: 120, px: 256, step: 64, major: ['#d5dbd8', '#3a403e'], minor: null, anisotropy: 4, repeat: 60 },
};
const TURN_SEGMENTS = 48;
const yawOf = (q) => Math.atan2(2 * (q[3] * q[6] + q[4] * q[5]), 1 - 2 * (q[5] * q[5] + q[6] * q[6]));
const flat = (opacity) => new THREE.MeshBasicMaterial({ color: TEAL, transparent: true, opacity, side: THREE.DoubleSide, depthWrite: false });

let meshes = null;

function loadMeshes() {
  if (!meshes) {
    meshes = Promise.all([import('../../vendor/meshopt/meshopt_decoder.js'), fetchJSON('assets/go2/model.json'), fetchBuffer('assets/go2/meshes.bin.gz')])
      .then(async ([{ MeshoptDecoder }, model, buf]) => {
        await MeshoptDecoder.ready;
        const src = new Uint8Array(buf), part = (p) => src.subarray(p.offset, p.offset + p.bytes);
        return { model, meshes: model.meshes.map(m => {
          const pos = new Uint8Array(m.nv * 8), nrm = new Uint8Array(m.nv * 4), idx = new Uint8Array(m.nf * 12);
          MeshoptDecoder.decodeVertexBuffer(pos, m.nv, 8, part(m.pos));
          MeshoptDecoder.decodeVertexBuffer(nrm, m.nv, 4, part(m.nrm), 'OCTAHEDRAL');
          MeshoptDecoder.decodeIndexBuffer(idx, 3 * m.nf, 4, part(m.idx));
          const g = new THREE.BufferGeometry();
          g.setAttribute('position', new THREE.InterleavedBufferAttribute(new THREE.InterleavedBuffer(new Uint16Array(pos.buffer), 4), 3, 0, false));
          g.setAttribute('normal', new THREE.InterleavedBufferAttribute(new THREE.InterleavedBuffer(new Int8Array(nrm.buffer), 4), 3, 0, true));
          g.setIndex(new THREE.BufferAttribute(new Uint32Array(idx.buffer), 1));
          g.computeBoundingSphere();
          return { geometry: g, lo: m.lo, scale: m.scale };
        }) };
      });
    meshes.catch(() => { meshes = null; });
  }
  return meshes;
}

class Robot {
  constructor(scene, { model, meshes }) {
    this.model = model;
    const materials = new Map(), material = (rgba) => {
      const key = rgba.join(',');
      if (!materials.has(key)) {
        const c = new THREE.Color(rgba[0], rgba[1], rgba[2]), dark = c.r + c.g + c.b < .2;
        materials.set(key, new THREE.MeshStandardMaterial({ color: dark ? 0x1d1f22 : c, roughness: dark ? .55 : .42, metalness: dark ? .1 : .25 }));
      }
      return materials.get(key);
    };
    this.root = new THREE.Group();
    scene.add(this.root);
    this.bodies = model.bodies.map(() => new THREE.Group());
    model.bodies.forEach((b, i) => (b.parent < 0 ? this.root : this.bodies[b.parent]).add(this.bodies[i]));
    for (const g of model.geoms) {
      const m = meshes[g.mesh], frame = new THREE.Group(), mesh = new THREE.Mesh(m.geometry, material(g.rgba));
      frame.position.set(...g.pos);
      frame.quaternion.set(g.quat[1], g.quat[2], g.quat[3], g.quat[0]);
      mesh.position.set(...m.lo);
      mesh.scale.set(...m.scale);
      mesh.castShadow = true;
      frame.add(mesh);
      this.bodies[g.body].add(frame);
    }
    this.axis = model.bodies.map(b => b.joint ? new THREE.Vector3(...b.joint.axis) : null);
    this.base = model.bodies.map(b => new THREE.Quaternion(b.quat[1], b.quat[2], b.quat[3], b.quat[0]));
    this.turn = new THREE.Quaternion();
  }

  set(q) {
    this.model.bodies.forEach((b, i) => {
      const node = this.bodies[i];
      if (b.parent < 0) {
        node.position.set(q[0], q[1], q[2]);
        node.quaternion.set(q[4], q[5], q[6], q[3]).normalize(); // MuJoCo (w, x, y, z) to three.js (x, y, z, w)
      } else {
        node.position.set(...b.pos);
        node.quaternion.copy(this.base[i]);
        if (b.joint?.type === 3) node.quaternion.multiply(this.turn.setFromAxisAngle(this.axis[i], q[b.joint.qadr]));
      }
    });
  }
}

export class Go2View extends View {
  static async create(stage) { return new Go2View(stage, await loadMeshes()); }

  constructor(stage, assets) {
    super(stage, OPTIONS);
    this.goalXY = null;
    this.chase([0, 0, .3, 1, 0, 0, 0]);
    this.robot = new Robot(this.scene, assets);
    this.markers();
  }

  markers() {
    const goal = this.goal = new THREE.Group();
    const tick = new THREE.Mesh(new THREE.PlaneGeometry(.34, .05), flat(.85));
    tick.position.x = .66;
    const pin = new THREE.Mesh(new THREE.CylinderGeometry(.012, .012, .5, 8), new THREE.MeshBasicMaterial({ color: TEAL }));
    pin.rotation.x = Math.PI / 2;
    pin.position.z = .25;
    const head = new THREE.Mesh(new THREE.SphereGeometry(.045, 16, 12), new THREE.MeshBasicMaterial({ color: TEAL }));
    head.position.z = .5;
    goal.add(new THREE.Mesh(new THREE.RingGeometry(.41, .5, 56), flat(.8)), new THREE.Mesh(new THREE.CircleGeometry(.41, 56), flat(.14)), tick, pin, head);
    const box = this.box = new THREE.Mesh(new THREE.BoxGeometry(1, 1, 1), new THREE.MeshStandardMaterial({ color: 0x9c8466, roughness: .8 }));
    box.castShadow = box.receiveShadow = true;
    box.add(new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.BoxGeometry(1, 1, 1)), new THREE.LineBasicMaterial({ color: 0x5e4c38 })));
    this.shaft = new THREE.Mesh(new THREE.PlaneGeometry(1, .06), flat(.85));
    this.tip = new THREE.Mesh(new THREE.CircleGeometry(.12, 3), flat(.85));
    this.turn = new THREE.Mesh(new THREE.RingGeometry(.52, .56, TURN_SEGMENTS, 1, 0, 2 * Math.PI), flat(.7));
    this.arrow = new THREE.Group();
    this.arrow.add(this.shaft, this.tip, this.turn);
    goal.visible = box.visible = this.arrow.visible = false;
    this.scene.add(goal, box, this.arrow);
  }

  chase(q, goal = this.goalXY) {
    this.follow = true;
    this.goalXY = goal ? { x: goal.x, y: goal.y } : null;
    const yaw = yawOf(q);
    let ux = Math.cos(yaw), uy = Math.sin(yaw), span = .9;
    if (goal) {
      const dx = goal.x - q[0], dy = goal.y - q[1], d = Math.hypot(dx, dy);
      if (d > .3) { ux = dx / d; uy = dy / d; }
      span = d / 2 + 1.2;
    }
    const t = this.lookAt(q), half = Math.atan(Math.tan(this.camera.fov * Math.PI / 360) * Math.max(this.camera.aspect, 1));
    const dist = Math.min(7, Math.max(3.2, span / Math.tan(half))), el = 22 * Math.PI / 180;
    this.controls.target.set(t.x, t.y, .28);
    this.camera.position.set(t.x + dist * Math.cos(el) * uy, t.y - dist * Math.cos(el) * ux, .28 + dist * Math.sin(el));
    this.controls.update();
  }

  lookAt(q) {
    const g = this.goalXY;
    return g ? { x: q[0] + .5 * (g.x - q[0]), y: q[1] + .5 * (g.y - q[1]) } : { x: q[0], y: q[1] };
  }

  overview(q, goal) {
    this.follow = false;
    const gx = goal ? goal.x : q[0], gy = goal ? goal.y : q[1];
    const cx = (q[0] + gx) / 2, cy = (q[1] + gy) / 2, d = Math.hypot(gx - q[0], gy - q[1]) * 1.1 + 4;
    this.controls.target.set(cx, cy, 0);
    this.camera.position.set(cx - d * .18, cy - d * .78, d * .72);
    this.controls.update();
  }

  setRobot(q) {
    this.robot.set(q);
    if (this.follow) {
      const want = this.lookAt(q), t = this.controls.target;
      this.moveCamera((want.x - t.x) * .08, (want.y - t.y) * .08);
    }
    this.sun.position.set(q[0] - 3, q[1] - 4, 6);
    this.sun.target.position.set(q[0], q[1], 0);
    this.floor.position.x = Math.round(q[0] / 2) * 2;
    this.floor.position.y = Math.round(q[1] / 2) * 2;
  }

  setGoal(show, x = 0, y = 0, z = 0, heading = 0) {
    this.goal.visible = show;
    this.goalXY = show ? { x, y } : null;
    if (show) { this.goal.position.set(x, y, z + .006); this.goal.rotation.z = heading; }
  }

  setBox(show, cx = 0, cy = 0, yaw = 0, hx = .75, hy = .5, h = .1) {
    const b = this.box;
    b.visible = show && h > .002;
    if (b.visible) { b.scale.set(2 * hx, 2 * hy, h); b.position.set(cx, cy, h / 2); b.rotation.z = yaw; }
  }

  setCommand(q, vx = 0, vy = 0, wz = 0) {
    this.arrow.visible = !!q;
    if (!q) return;
    this.arrow.position.set(q[0], q[1], .008);
    this.arrow.rotation.z = yawOf(q);
    const len = Math.hypot(vx, vy), ang = Math.atan2(vy, vx), r = .35 + .6 * len;
    this.shaft.visible = this.tip.visible = len > .05;
    this.shaft.scale.x = Math.max(r - .35, 1e-3);
    this.shaft.position.set(Math.cos(ang) * (.35 + (r - .35) / 2), Math.sin(ang) * (.35 + (r - .35) / 2), 0);
    this.shaft.rotation.z = ang;
    this.tip.position.set(Math.cos(ang) * (r + .06), Math.sin(ang) * (r + .06), 0);
    this.tip.rotation.z = ang;
    const k = Math.min(TURN_SEGMENTS, Math.round(Math.abs(wz) * 2.2 / (2 * Math.PI) * TURN_SEGMENTS));
    this.turn.visible = k > 0;
    this.turn.geometry.setDrawRange(0, 6 * k);
    this.turn.scale.y = wz >= 0 ? 1 : -1;
  }
}
