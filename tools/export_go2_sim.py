"""Usage: python tools/export_go2_sim.py --chronosrl <chronosrl checkout> [--out assets/go2]"""
import argparse
import gzip
import json
import math
import os
import pathlib
import sys


def capture(box):
    import mujoco
    from chronosrl.environments.go2.locomotion.default_config import get_config
    from chronosrl.environments.go2.locomotion.environment import LocomotionEnv
    from chronosrl.environments.go2.unitree_go2.robot_config import robot_config
    import chronosrl.environments.go2.goal_environment as goal_environment

    class Captured(Exception):
        pass

    def intercept(xml, assets=None):
        captured.update(xml=xml, assets=assets)
        raise Captured()

    config, captured = get_config(), {}
    if box:
        config.terrain.type = "plane_box"
        config.njmax = 256
        config.naconmax_per_env = 24
    original = mujoco.MjModel.from_xml_string
    mujoco.MjModel.from_xml_string = staticmethod(intercept)
    try:
        LocomotionEnv({**robot_config, "directory_path": pathlib.Path(goal_environment.__file__).parent / "unitree_go2"}, config, 1)
    except Captured:
        pass
    finally:
        mujoco.MjModel.from_xml_string = original
    model = mujoco.MjModel.from_xml_string(captured["xml"], captured["assets"])
    model.opt.timestep = config["timestep"]
    return model, config, robot_config


def export(out):
    import mujoco
    assert mujoco.__version__ == "3.7.0", "MJB files are bound to the MuJoCo version of vendor/mujoco"
    geom = lambda m, name: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, name)
    meta, ids = {"models": {}}, {}
    for name, box in (("plane", False), ("plane_box", True)):
        m, config, robot_config = capture(box)
        raw = os.path.join(out, f"{name}.mjb")
        mujoco.mj_saveModel(m, raw)
        with open(raw, "rb") as f, open(raw + ".gz", "wb") as g:
            g.write(gzip.compress(f.read(), compresslevel=9, mtime=0))
        os.remove(raw)
        feet = [geom(m, f"{leg}_foot") for leg in ("FR", "FL", "RR", "RL")]
        ids[name] = {"imu_site": mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "imu"), "gyro_adr": int(m.sensor_adr[m.sensor("imu_angular_velocity").id]),
                     "velo_adr": int(m.sensor_adr[m.sensor("imu_linear_velocity").id]), "floor_geom": geom(m, "floor")}
        assert [int(m.jnt_qposadr[m.actuator_trnid[i, 0]]) for i in range(m.nu)] == list(range(7, 19))
        meta["models"][name] = {"file": f"{name}.mjb.gz", "foot_geoms": feet}
        if box:
            meta["models"][name].update(
                calf_geoms=[[g, float(m.geom_size[g, 0]), float(m.geom_size[g, 1])] for g in range(m.ngeom)
                            if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_CAPSULE and "calf" in (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or "")],
                box_geom=geom(m, "climb_box"), box_mocap=int(m.body_mocapid[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "climb_box_body")]))
            meta["box"] = {"length": float(config.terrain.box_length_in_meters), "width": float(config.terrain.box_width_in_meters)}
        else:
            joints = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, m.actuator_trnid[i, 0]) for i in range(m.nu)]
            home = m.key("home").qpos.copy()
            phase = {"FL": 0.0, "RR": 0.0, "FR": math.pi, "RL": math.pi}
            meta.update({
                "timestep": float(config.timestep), "substeps": int(round(0.02 / config.timestep)), "control_dt": 0.02, "home": home.tolist(),
                "nominal": home[7:].tolist(), "joint_range": m.jnt_range[m.actuator_trnid[:, 0]].tolist(), "foot_radius": float(m.geom_size[feet[0], 0]),
                "action_scale": float(robot_config["scaling_factor"]), "gait_amplitude": 0.3, "clock_rate": 2 * math.pi * 3.0 * 0.02,
                "gait_gain": [2.0 if "thigh" in j else (-2.0 if "calf" in j else 0.0) for j in joints],
                "gait_phase": [phase[j.upper()[:2]] for j in joints], "heading_scale": 0.3, "goal_radius": {"velocity": 0.25, "position": 0.5, "box": 0.5},
            })
    assert ids["plane"] == ids["plane_box"]
    meta.update(ids["plane"])
    with open(os.path.join(out, "sim.json"), "w") as f:
        json.dump(meta, f, indent=1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--chronosrl", required=True)
    ap.add_argument("--out", default=os.path.join(os.path.dirname(__file__), "..", "assets", "go2"))
    args = ap.parse_args()
    sys.path.insert(0, os.path.abspath(args.chronosrl))
    os.makedirs(args.out, exist_ok=True)
    export(args.out)


if __name__ == "__main__":
    main()
