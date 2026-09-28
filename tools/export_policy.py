"""Usage: python tools/export_policy.py <checkpoint.pkl> <args.pkl> <task> <depth> <seed> [--out assets/policies]"""
import argparse
import json
import os
import pickle
import struct

import numpy as np

TASKS = ["u4", "bigmaze", "hardest", "u5", "humanoid", "humanoidu", "humanoidbig", "velocity", "position", "box"]
GO2_INPUTS = {"velocity": 47, "position": 48, "box": 48}


class Writer:
    def __init__(self, half=False):
        self.parts, self.offset, self.half = [], 0, half

    def add(self, array):
        data = array.tobytes()
        data += b"\0" * (-len(data) % 8)
        self.parts.append(data)
        self.offset += len(data)
        return self.offset - len(data)

    def dense(self, p):
        kernel, bias = np.asarray(p["kernel"], np.float32), np.asarray(p["bias"], np.float32)
        op = {"op": "dense", "in": int(kernel.shape[0]), "out": int(kernel.shape[1])}
        if self.half:
            return {**op, "fmt": "f16", "w": self.add(kernel.T.astype(np.float16).copy()), "b": self.add(bias)}
        scale = np.abs(kernel).max(0) / 127.0
        scale = np.where(scale == 0, 1e-12, scale).astype(np.float32)
        weights = np.round(kernel / scale).clip(-127, 127).astype(np.int8).T.copy()
        return {**op, "w": self.add(weights), "s": self.add(scale), "b": self.add(bias)}

    def ln(self, p):
        scale = np.asarray(p["scale"], np.float32)
        return {"op": "ln", "n": int(scale.shape[0]), "g": self.add(scale), "b": self.add(np.asarray(p["bias"], np.float32))}

    def mlp(self, p):
        program = []
        if "block_0" not in p:
            for k in range(len([n for n in p if n.startswith("Dense_")])):
                program += [self.dense(p[f"Dense_{k}"]), self.ln(p[f"LayerNorm_{k}"]), {"op": "swish"}]
            return program
        program += [self.dense(p["Dense_0"]), self.ln(p["LayerNorm_0"]), {"op": "swish"}]
        for i in range(len([n for n in p if n.startswith("block_")])):
            block = p[f"block_{i}"]
            program.append({"op": "save"})
            for k in range(4):
                program += [self.dense(block[f"dense_{k}"]), self.ln(block[f"norm_{k}"]), {"op": "swish"}]
            program.append({"op": "add"})
        return program

    def write(self, path, header):
        head = json.dumps(header, separators=(",", ":")).encode()
        head += b" " * (-(12 + len(head)) % 8)
        with open(path, "wb") as f:
            f.write(b"CSRL" + struct.pack("<II", 1, len(head)) + head + b"".join(self.parts))


def export(checkpoint, args_file, task, depth, seed, out):
    _, actor, critic = pickle.load(open(checkpoint, "rb"))
    args = pickle.load(open(args_file, "rb"))
    base = os.path.join(out, task, f"d{depth}_s{seed}")
    os.makedirs(os.path.dirname(base), exist_ok=True)
    a = actor["params"]
    w = Writer(half=task in GO2_INPUTS)
    program = w.mlp(a["ResidualMLP_0"] if "ResidualMLP_0" in a else a["MLP_0"]) + [w.dense(a["Dense_0"])]
    if task in GO2_INPUTS:
        assert program[0]["in"] == GO2_INPUTS[task] and program[-1]["out"] == 24
    w.write(base + ".bin", {"task": task, "depth": depth, "seed": seed, "chunk": int(args.agent.action_chunk_length), "nets": {"actor": program}})
    w, nets = Writer(), {}
    for name in ("enc_sa", "enc_g"):
        e = critic["params"][name]
        nets[name] = w.mlp(e["backbone"]) + [w.dense(e["Dense_0"])] + ([w.ln(e["LayerNorm_0"])] if "LayerNorm_0" in e else [])
    w.write(base + ".critic.bin", {"m": int(args.agent.m), "state_dim": int(args.env.state_dim),
                                   "goal_const": bool(args.agent.get("goal_const_input", False)), "nets": nets})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("checkpoint")
    ap.add_argument("args")
    ap.add_argument("task", choices=TASKS)
    ap.add_argument("depth", type=int)
    ap.add_argument("seed", type=int)
    ap.add_argument("--out", default=os.path.join(os.path.dirname(__file__), "..", "assets", "policies"))
    a = ap.parse_args()
    export(a.checkpoint, a.args, a.task, a.depth, a.seed, a.out)
    path = os.path.join(a.out, "manifest.json")
    manifest = json.load(open(path)) if os.path.exists(path) else {}
    previous = manifest.get(a.task, {}).get(str(a.depth))
    if previous not in (None, a.seed):
        for suffix in (".bin", ".critic.bin"):
            os.remove(os.path.join(a.out, a.task, f"d{a.depth}_s{previous}{suffix}"))
    manifest.setdefault(a.task, {})[str(a.depth)] = a.seed
    manifest = {t: dict(sorted(manifest[t].items(), key=lambda kv: int(kv[0]))) for t in TASKS if t in manifest}
    with open(path, "w") as f:
        f.write("{\n" + ",\n".join(f' "{t}": {json.dumps(d)}' for t, d in manifest.items()) + "\n}\n")


if __name__ == "__main__":
    main()
