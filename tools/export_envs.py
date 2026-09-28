"""Usage: python tools/export_envs.py --chronosrl <chronosrl checkout> [--out assets/env]"""
import argparse
import json
import os
import sys

import numpy as np

TASKS = {
    "u4": ("ant_u4_maze", "u4_maze"),
    "bigmaze": ("ant_big_maze", "big_maze"),
    "hardest": ("ant_hardest_maze", "hardest_maze"),
    "u5": ("ant_u5_maze", "u5_maze"),
    "humanoid": ("humanoid", None),
    "humanoidu": ("humanoid_u_maze", "u_maze"),
    "humanoidbig": ("humanoid_big_maze", "big_maze"),
}
PAIR_FUNCTIONS = {("PLANE", "SPHERE"): "plane_sphere", ("PLANE", "CAPSULE"): "plane_capsule", ("SPHERE", "SPHERE"): "sphere_sphere",
                  ("SPHERE", "CAPSULE"): "sphere_capsule", ("SPHERE", "BOX"): "sphere_box", ("CAPSULE", "BOX"): "capsule_box"}


def r(x):
    a = np.asarray(x, dtype=np.float64)
    if a.ndim == 0:
        v = float(a)
        return v if np.isfinite(v) else (1e30 if v > 0 else -1e30)
    return [r(v) for v in a]


def export(key):
    import jax.numpy as jnp
    import jax.tree_util as jtu
    import mujoco
    from brax import kinematics
    from mujoco.mjx._src import collision_driver, mesh
    from mujoco.mjx._src.collision_types import GeomInfo
    from mujoco.mjx._src.types import GeomType
    from chronosrl.environments.jaxgcrl import TASKS as ENVIRONMENTS
    from chronosrl.environments.jaxgcrl import ant_maze, humanoid

    name, layout = TASKS[key]
    env = ENVIRONMENTS[name]()
    s, m = env.sys, env.sys.mj_model
    link = s.link
    out = {
        "dt": float(s.dt), "n_frames": int(env._n_frames), "gravity": r(s.gravity), "vel_damping": float(s.vel_damping),
        "ang_damping": float(s.ang_damping), "baumgarte_erp": float(s.baumgarte_erp), "spring_mass_scale": float(s.spring_mass_scale),
        "spring_inertia_scale": float(s.spring_inertia_scale), "link_types": s.link_types, "link_parents": [int(v) for v in s.link_parents],
        "link": {
            "t_pos": r(link.transform.pos), "t_rot": r(link.transform.rot), "j_pos": r(link.joint.pos), "j_rot": r(link.joint.rot),
            "i_pos": r(link.inertia.transform.pos), "i_rot": r(link.inertia.transform.rot),
            "i_mat": r(np.asarray(link.inertia.i).reshape(s.num_links(), 9)), "mass": r(link.inertia.mass),
            "cs": r(link.constraint_stiffness), "cvd": r(link.constraint_vel_damping),
            "cls": r(link.constraint_limit_stiffness), "cad": r(link.constraint_ang_damping),
        },
        "dof": {
            "ang": r(s.dof.motion.ang), "vel": r(s.dof.motion.vel),
            "lo": r(s.dof.limit[0]) if s.dof.limit is not None else None, "hi": r(s.dof.limit[1]) if s.dof.limit is not None else None,
        },
        "actuator": {
            "q_id": [int(v) for v in s.actuator.q_id], "qd_id": [int(v) for v in s.actuator.qd_id], "ctrl": r(s.actuator.ctrl_range),
            "force": r(s.actuator.force_range), "gain": r(s.actuator.gain), "gear": r(s.actuator.gear),
            "bias_q": r(s.actuator.bias_q), "bias_qd": r(s.actuator.bias_qd),
        },
        "init_q": r(s.init_q), "state_dim": int(env.state_dim), "goal_idx": [int(i) for i in env.goal_indices],
    }
    frames, q0 = [], 0
    for typ in s.link_types:
        width = {"f": 6, "1": 1, "2": 2, "3": 3}[typ]
        if typ == "f":
            frames.append(None)
        else:
            frame, parity = kinematics.link_to_joint_frame(jtu.tree_map(lambda a: a[q0:q0 + width], s.dof.motion))
            frames.append({"ang": r(frame.ang), "vel": r(frame.vel), "parity": float(parity)})
        q0 += width
    out["joint_frames"] = frames
    out["geoms"] = [{
        "name": mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or "", "type": int(m.geom_type[g]), "size": r(m.geom_size[g]),
        "link": int(s.geom_bodyid[g]) - 1, "pos": r(s.geom_pos[g]), "quat": r(s.geom_quat[g]), "rbound": float(m.geom_rbound[g]),
        "friction": float(s.geom_friction[g][0]), "elasticity": float(s.elasticity[g]),
    } for g in range(m.ngeom)]
    pairs = {}
    for group, geoms in collision_driver._geom_groups(s).items():
        function = PAIR_FUNCTIONS[tuple(GeomType(int(t)).name for t in group.types)]
        pairs.setdefault(function, []).extend([[int(a), int(b)] for a, b, _ in geoms])
    out["pairs"] = pairs
    box = mesh.box(GeomInfo(pos=jnp.zeros(3), mat=jnp.eye(3), size=jnp.ones(3)))
    out["box"] = {"vert": r(np.asarray(box.vert)[0]), "face": r(np.asarray(box.face)[0]), "face_normal": r(box.face_normal),
                  "edge": np.asarray(box.edge).tolist(), "edge_face_normal": r(box.edge_face_normal)}
    task = {"healthy_z": [float(v) for v in env.healthy_z_range], "success_radius": float(env.goal_radius)}
    if name.startswith("ant"):
        task.update(kind="ant_maze", reset_noise=float(env.reset_noise_scale), goals=r(env.possible_goals))
        task.update(layout=ant_maze.LAYOUTS[layout], scale=4.0)
    elif layout:
        task.update(kind="humanoid_maze", reset_noise=0.0, goals=r(env.possible_goals), starts=r(env.possible_starts),
                    target_z=humanoid.TARGET_Z_COORD, layout=humanoid.LAYOUTS[layout], scale=2.0)
    else:
        task.update(kind="humanoid", reset_noise=0.0, target_z=humanoid.TARGET_Z_COORD, min_goal_dist=1.0, max_goal_dist=5.0)
    task["key"] = key
    out["task"] = task
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--chronosrl", required=True)
    ap.add_argument("--out", default=os.path.join(os.path.dirname(__file__), "..", "assets", "env"))
    args = ap.parse_args()
    sys.path.insert(0, os.path.abspath(args.chronosrl))
    os.makedirs(args.out, exist_ok=True)
    for key in TASKS:
        with open(os.path.join(args.out, f"{key}.json"), "w") as f:
            json.dump(export(key), f, separators=(",", ":"))


if __name__ == "__main__":
    main()
