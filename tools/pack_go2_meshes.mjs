// Usage: node tools/pack_go2_meshes.mjs <model.json> <raw.bin> <meshoptimizer npm package> <out dir> (run by export_go2.py)
import fs from 'fs';
import path from 'path';
import zlib from 'zlib';

const [specPath, rawPath, meshopt, out] = process.argv.slice(2);
const { MeshoptEncoder } = await import(path.join(meshopt, 'meshopt_encoder.js'));
await MeshoptEncoder.ready;
const model = JSON.parse(fs.readFileSync(specPath, 'utf8'));
const raw = new Uint8Array(fs.readFileSync(rawPath));
const copy = (Type, byteOffset, n) => new Type(raw.slice(byteOffset, byteOffset + n * Type.BYTES_PER_ELEMENT).buffer);

const parts = [];
let offset = 0;
const add = (u8) => {
  const at = offset;
  parts.push(Buffer.from(u8.buffer, u8.byteOffset, u8.length));
  offset += u8.length;
  const pad = (4 - (u8.length % 4)) % 4;
  if (pad) { parts.push(Buffer.alloc(pad)); offset += pad; }
  return { offset: at, bytes: u8.length };
};

const meshes = model.meshes.map(({ nv, nf, raw: at }) => {
  const pos = copy(Float32Array, at, 3 * nv), nrm = copy(Float32Array, at + 12 * nv, 3 * nv);
  const idx = copy(Uint32Array, at + 24 * nv, 3 * nf);
  const [remap, unique] = MeshoptEncoder.reorderMesh(idx, true, true);
  const P = new Float32Array(3 * unique), N = new Float32Array(4 * unique);
  for (let i = 0; i < nv; i++) {
    const r = remap[i];
    if (r === 0xffffffff) continue;
    P.set(pos.subarray(3 * i, 3 * i + 3), 3 * r);
    N.set(nrm.subarray(3 * i, 3 * i + 3), 4 * r);
  }
  const lo = [0, 1, 2].map(k => { let v = Infinity; for (let i = k; i < P.length; i += 3) v = Math.min(v, P[i]); return v; });
  const hi = [0, 1, 2].map(k => { let v = -Infinity; for (let i = k; i < P.length; i += 3) v = Math.max(v, P[i]); return v; });
  const scale = lo.map((l, k) => Math.max(hi[k] - l, 1e-9) / 16383);
  const Q = new Uint16Array(4 * unique);
  for (let i = 0; i < unique; i++) for (let k = 0; k < 3; k++) Q[4 * i + k] = Math.round((P[3 * i + k] - lo[k]) / scale[k]);
  return {
    nv: unique, nf, lo: lo.map(v => +v.toFixed(7)), scale,
    pos: add(MeshoptEncoder.encodeVertexBuffer(new Uint8Array(Q.buffer), unique, 8)),
    nrm: add(MeshoptEncoder.encodeVertexBuffer(MeshoptEncoder.encodeFilterOct(N, unique, 4, 8), unique, 4)),
    idx: add(MeshoptEncoder.encodeIndexBuffer(new Uint8Array(idx.buffer), 3 * nf, 4)),
  };
});
fs.writeFileSync(path.join(out, 'meshes.bin.gz'), zlib.gzipSync(Buffer.concat(parts), { level: 9 }));
fs.writeFileSync(path.join(out, 'model.json'), JSON.stringify({ ...model, meshes }));
