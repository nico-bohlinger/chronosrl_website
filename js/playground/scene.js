import * as THREE from 'three';
import { View, TEAL } from '../view.js';

const qa = new THREE.Quaternion(), qb = new THREE.Quaternion(), va = new THREE.Vector3(), vb = new THREE.Vector3();
const Y_TO_Z = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), Math.PI / 2);

const OPTIONS = {
  background: [0xdfe6e4, 0x1f2423], fog: [30, 90], fov: 42, near: .1, far: 300, minDistance: 2, maxDistance: 90,
  hemi: [0x8d9794, 1.35], sun: 2.3, shadow: { size: 2048, extent: 7, far: 40, bias: -0.0005 }, trailZ: .02,
  floor: { size: 400, px: 512, step: 128, major: ['#dfe5e2', '#343938'], minor: ['#e8ecea', '#303433'], anisotropy: 8, repeat: 100 },
};

export class PlaygroundView extends View {
  constructor(stage) {
    super(stage, OPTIONS);
    this.world = new THREE.Group();
    this.agent = new THREE.Group();
    this.scene.add(this.world, this.agent);
    this.meshes = [];
    this.goalMarker();
    this.follow = false;
  }

  setDark(dark) {
    super.setDark(dark);
    if (this.json) this.setEnv(this.json, true);
  }

  goalMarker() {
    const g = this.goal = new THREE.Group(), teal = new THREE.Color(TEAL);
    this.goalDisc = new THREE.Mesh(new THREE.CircleGeometry(.5, 64), new THREE.MeshBasicMaterial({ color: teal, transparent: true, opacity: .18, depthWrite: false }));
    this.goalRing = new THREE.Mesh(new THREE.RingGeometry(.46, .5, 64), new THREE.MeshBasicMaterial({ color: teal, transparent: true, opacity: .9, depthWrite: false }));
    this.goalDisc.position.z = this.goalRing.position.z = .012;
    const pin = new THREE.Mesh(new THREE.CylinderGeometry(.018, .018, 1.1, 12), new THREE.MeshStandardMaterial({ color: 0x0b6e60 }));
    pin.quaternion.copy(Y_TO_Z);
    pin.position.z = .55;
    pin.castShadow = true;
    const head = this.goalHead = new THREE.Mesh(new THREE.SphereGeometry(.11, 24, 16), new THREE.MeshStandardMaterial({ color: teal, emissive: teal, emissiveIntensity: .35 }));
    head.position.z = 1.15;
    head.castShadow = true;
    g.add(this.goalDisc, this.goalRing, pin, head);
    this.scene.add(g);
  }

  setEnv(json, keepTrail = false) {
    this.json = json;
    for (const g of [this.world, this.agent]) {
      const materials = new Set();
      for (const c of [...g.children]) {
        g.remove(c);
        c.geometry?.dispose();
        if (c.material) materials.add(c.material);
        if (c.isInstancedMesh) c.dispose();
      }
      for (const m of materials) m.dispose();
    }
    if (!keepTrail) this.clearTrail();
    const walls = json.geoms.filter(g => g.link < 0 && g.type === 6);
    if (walls.length) {
      const inst = new THREE.InstancedMesh(new THREE.BoxGeometry(1, 1, 1),
        new THREE.MeshStandardMaterial({ color: this.dark ? 0x5d5953 : 0xcfc8bb, roughness: .9 }), walls.length);
      const m4 = new THREE.Matrix4();
      walls.forEach((w, i) => {
        m4.compose(new THREE.Vector3(...w.pos), new THREE.Quaternion(w.quat[1], w.quat[2], w.quat[3], w.quat[0]),
          new THREE.Vector3(2 * w.size[0], 2 * w.size[1], 2 * w.size[2]));
        inst.setMatrixAt(i, m4);
      });
      inst.castShadow = inst.receiveShadow = true;
      this.world.add(inst);
    }
    const body = new THREE.MeshStandardMaterial({ color: 0xfbfaf6, roughness: .45, metalness: .05 });
    const feet = new THREE.MeshStandardMaterial({ color: 0x2c3a3f, roughness: .6 });
    const torso = new THREE.MeshStandardMaterial({ color: TEAL, roughness: .4, metalness: .05 });
    const target = json.link_types.length - 1;
    this.meshes = [];
    for (const g of json.geoms) {
      if (g.link < 0 || g.link === target) continue;
      let geo;
      if (g.type === 2) geo = new THREE.SphereGeometry(g.size[0], 20, 14);
      else if (g.type === 3) geo = new THREE.CapsuleGeometry(g.size[0], 2 * g.size[1], 6, 14);
      else if (g.type === 6) geo = new THREE.BoxGeometry(2 * g.size[0], 2 * g.size[1], 2 * g.size[2]);
      else continue;
      const material = /foot|hand/.test(g.name) ? feet : g.link === 0 && (g.type === 2 || /torso|head/.test(g.name)) ? torso : body;
      const mesh = new THREE.Mesh(geo, material);
      mesh.castShadow = true;
      this.agent.add(mesh);
      this.meshes.push({ mesh, g, capsule: g.type === 3 });
    }
    this.goalHead.position.z = json.task.kind === 'ant_maze' ? 1.15 : 1.65;
  }

  setPose(x, prev, alpha) {
    for (const { mesh, g, capsule } of this.meshes) {
      const a = x[g.link], b = prev?.[g.link];
      if (b) {
        va.set(b.pos[0], b.pos[1], b.pos[2]).lerp(vb.set(a.pos[0], a.pos[1], a.pos[2]), alpha);
        qa.set(b.rot[1], b.rot[2], b.rot[3], b.rot[0]).slerp(qb.set(a.rot[1], a.rot[2], a.rot[3], a.rot[0]), alpha);
      } else {
        va.set(a.pos[0], a.pos[1], a.pos[2]);
        qa.set(a.rot[1], a.rot[2], a.rot[3], a.rot[0]);
      }
      mesh.position.set(g.pos[0], g.pos[1], g.pos[2]).applyQuaternion(qa).add(va);
      mesh.quaternion.copy(qa).multiply(qb.set(g.quat[1], g.quat[2], g.quat[3], g.quat[0]));
      if (capsule) mesh.quaternion.multiply(Y_TO_Z);
    }
  }

  setGoal(xy, reached = false, t = 0) {
    this.goal.position.set(xy[0], xy[1], 0);
    this.goalRing.material.opacity = reached ? .95 : .7;
    this.goalDisc.material.opacity = reached ? .32 + .08 * Math.sin(t * 6) : .16;
  }

  overview([x0, x1, y0, y1]) {
    this.follow = false;
    const cx = (x0 + x1) / 2, cy = (y0 + y1) / 2, d = Math.max(x1 - x0, y1 - y0) * .95 + 4;
    this.controls.target.set(cx, cy, 0);
    this.camera.position.set(cx - d * .18, cy - d * .78, d * .72);
    this.controls.update();
  }

  chase(p, dist = 9) {
    this.follow = true;
    this.controls.target.set(p[0], p[1], p[2] * .5);
    this.camera.position.set(p[0] - dist * .16, p[1] - dist * .55, p[2] * .5 + dist * .82);
    this.controls.update();
  }

  track(p) {
    const t = this.controls.target;
    if (this.follow) this.moveCamera((p[0] - t.x) * .08, (p[1] - t.y) * .08, (p[2] * .5 - t.z) * .08);
    this.sun.position.set(p[0] - 4, p[1] - 6, 10);
    this.sun.target.position.set(p[0], p[1], 0);
  }
}
