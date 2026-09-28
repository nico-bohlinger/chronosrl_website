"""Usage: python tools/export_go2.py --xml <unitree_go2/data/plane.xml of RL-X> --meshopt <meshoptimizer npm package>"""
import argparse
import json
import os
import subprocess
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))


def export(xml, meshopt, out):
    import mujoco
    m = mujoco.MjModel.from_xml_path(xml)
    bodies = []
    for b in range(1, m.nbody):
        joint = int(m.body_jntadr[b])
        bodies.append({
            "parent": int(m.body_parentid[b]) - 1, "pos": np.round(m.body_pos[b], 6).tolist(), "quat": np.round(m.body_quat[b], 6).tolist(),
            "joint": None if joint < 0 else {"type": int(m.jnt_type[joint]), "axis": m.jnt_axis[joint].tolist(), "qadr": int(m.jnt_qposadr[joint])},
        })
    tmp = tempfile.mkdtemp()
    meshes, offset = [], 0
    vert, norm = np.asarray(m.mesh_vert, np.float32), np.asarray(m.mesh_normal, np.float32)
    with open(os.path.join(tmp, "raw.bin"), "wb") as raw:
        for i in range(m.nmesh):
            v0, f0, n0, nf = int(m.mesh_vertadr[i]), int(m.mesh_faceadr[i]), int(m.mesh_normaladr[i]), int(m.mesh_facenum[i])
            face = np.asarray(m.mesh_face[f0:f0 + nf], np.int64) + v0
            face_normal = np.asarray(m.mesh_facenormal[f0:f0 + nf], np.int64) + n0
            corners = np.concatenate([vert[face.reshape(-1)], norm[face_normal.reshape(-1)]], axis=1)
            unique, inverse = np.unique(corners, axis=0, return_inverse=True)
            idx = inverse.reshape(-1).astype(np.uint32)
            pos, nrm = np.ascontiguousarray(unique[:, :3], np.float32), np.ascontiguousarray(unique[:, 3:], np.float32)
            nrm /= np.maximum(np.linalg.norm(nrm, axis=1, keepdims=True), 1e-12)
            for a in (pos, nrm, idx):
                raw.write(a.tobytes())
            meshes.append({"nv": int(len(pos)), "nf": nf, "raw": offset})
            offset += pos.nbytes + nrm.nbytes + idx.nbytes
    geoms = []
    for g in range(m.ngeom):
        if m.geom_group[g] != 2 or m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        material = int(m.geom_matid[g])
        rgba = (m.mat_rgba[material] if material >= 0 else m.geom_rgba[g]).tolist()
        geoms.append({"body": int(m.geom_bodyid[g]) - 1, "mesh": int(m.geom_dataid[g]), "pos": np.round(m.geom_pos[g], 7).tolist(),
                      "quat": np.round(m.geom_quat[g], 7).tolist(), "rgba": [round(c, 4) for c in rgba]})
    spec = os.path.join(tmp, "model.json")
    with open(spec, "w") as f:
        json.dump({"bodies": bodies, "meshes": meshes, "geoms": geoms}, f)
    os.makedirs(out, exist_ok=True)
    subprocess.run(["node", os.path.join(HERE, "pack_go2_meshes.mjs"), spec, os.path.join(tmp, "raw.bin"), meshopt, out], check=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--xml", required=True)
    ap.add_argument("--meshopt", required=True)
    ap.add_argument("--out", default=os.path.join(HERE, "..", "assets", "go2"))
    a = ap.parse_args()
    export(a.xml, os.path.abspath(a.meshopt), a.out)


if __name__ == "__main__":
    main()
