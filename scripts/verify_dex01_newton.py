# Copyright (c) 2026 Analog Devices, Inc. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Verify real Newton contact, penetration depths and environment isolation."""

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "source/dex01_sim_asset"))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument(
        "--sweep-all", action="store_true", help="Check every taxel instead of five representative locations."
    )
    parser.add_argument("--num-envs", type=int, choices=(1, 2), default=2)
    parser.add_argument("--device", default="cuda:0")
    args = parser.parse_args()
    from dex01_newton_lifecycle import run_with_report

    run_with_report(lambda: verify(args), args.report, "contact-verification")


def verify(args):
    import isaaclab.sim as sim
    import torch
    import trimesh
    from dex01_sim_asset.newton import TactileSensor, TactileSensorCfg
    from dex01_sim_asset.newton.assets import prepare_sensor_asset, write_indenter
    from dex01_sim_asset.taxel_patterns import Dex01PatternCfg
    from isaaclab.app import launch_simulation
    from isaaclab.assets import RigidObjectCfg
    from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
    from isaaclab.sensors import ContactSensorCfg
    from isaaclab.sim.schemas import UsdPhysicsRigidBodyCfg
    from isaaclab.utils import configclass
    from isaaclab_newton.physics import MJWarpSolverCfg, NewtonCfg

    reference = json.loads((ROOT / "assets/sensors/dex01/reference.json").read_text())
    mesh = trimesh.creation.icosphere(subdivisions=3, radius=0.003)

    @configclass
    class SceneCfg(InteractiveSceneCfg):
        pad = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Pad",
            spawn=sim.UsdFileCfg(
                usd_path=prepare_sensor_asset(ROOT / "assets/robots/tesollo/dex01.usda"),
                rigid_props=UsdPhysicsRigidBodyCfg(kinematic_enabled=True),
            ),
        )
        probe = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Probe",
            spawn=sim.UsdFileCfg(usd_path=write_indenter(mesh), rigid_props=UsdPhysicsRigidBodyCfg()),
            init_state=RigidObjectCfg.InitialStateCfg(pos=(0, 0, 1)),
        )
        contact = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Pad", filter_prim_paths_expr=["{ENV_REGEX_NS}/Probe"])

    physics = NewtonCfg(solver_cfg=MJWarpSolverCfg(use_mujoco_contacts=False, nconmax=160, njmax=960))
    with launch_simulation(physics) as resolved:
        context = sim.SimulationContext(
            sim.SimulationCfg(dt=1 / 240, device=args.device, physics=resolved, gravity=(0, 0, 0))
        )
        scene = InteractiveScene(SceneCfg(num_envs=args.num_envs, env_spacing=0.1))
        sensor = TactileSensor(
            TactileSensorCfg(
                prim_path="/World/envs/env_.*/Pad",
                mesh_prim_path="/World/envs/env_.*/Probe",
                pattern_cfg=Dex01PatternCfg(),
                offset_pos=tuple(reference["mount_translation_m"]),
                offset_rot_wxyz=tuple(reference["mount_rotation_wxyz"]),
            )
        )
        context.reset()

        def state(asset):
            return lambda: (
                asset.data.root_link_pose_w.torch,
                asset.data.root_com_pos_w.torch,
                asset.data.root_com_vel_w.torch,
            )

        probe = scene["probe"]
        sensor.bind(state(scene["pad"]), state(probe), scene["contact"], mesh.vertices, mesh.faces)
        sensor.update(1 / 240)
        pixels = sensor.taxel2pixel
        points = sensor.data.taxel_points_w[0].clone()
        normals = sensor.data.taxel_normals_w[0].clone()
        selected = (
            range(738)
            if args.sweep_all
            else [
                int(torch.where((pixels == pixels.new_tensor(pixel)).all(-1))[0][0])
                for pixel in ((25, 15), (25, 2), (25, 29), (3, 15), (12, 15))
            ]
        )
        report = {
            "backend": "newton",
            "mode": "contact-verification",
            "failure": None,
            "num_envs": args.num_envs,
            "sweep_all": args.sweep_all,
            "cases": [],
            "subset": None,
        }
        # Save measurements before any assertion can fail, so failures retain evidence.
        for index in selected:
            loads = []
            for penetration in (-0.001, 0.0, 0.0005, 0.001):
                pose = probe.data.root_link_pose_w.torch[:1].clone()
                pose[0, :3] = points[index] + (0.003 - penetration) * normals[index]
                pose[0, 3:] = pose.new_tensor([0, 0, 0, 1])
                probe.write_root_pose_to_sim_index(root_pose=pose, env_ids=[0])
                probe.write_root_velocity_to_sim_index(root_velocity=torch.zeros(1, 6, device=args.device), env_ids=[0])
                scene.write_data_to_sim()
                context.step(render=False)
                scene.update(1 / 240)
                sensor.update(1 / 240)
                case = measure(sensor, probe, index, points[index], penetration)
                report["cases"].append(case)
                args.report.write_text(json.dumps(report, indent=2) + "\n")
                check_case(case, penetration)
                if args.num_envs == 2 and sensor.data.normal_forces[1].sum() != 0:
                    raise RuntimeError("Contact leaked into the unloaded environment")
                if penetration > 0:
                    loads.append(case["load_n"])
            if loads[1] <= loads[0]:
                raise RuntimeError("Greater indentation did not produce greater normal load")
        if args.num_envs == 2:
            report["subset"] = verify_subset(scene, context, sensor)
        args.report.write_text(json.dumps(report, indent=2) + "\n")
        print(
            f"Passed {len(report['cases'])} contact cases, analytic depths, force conservation and environment"
            " isolation"
        )


def measure(sensor, probe, index, target, penetration):
    import torch

    data = sensor.data
    force = data.normal_forces[0]
    depths = data.depths[0]
    image = sensor.get_tactile_image()[0]
    pixels = sensor.taxel2pixel
    if not torch.equal(image[pixels[:, 0], pixels[:, 1]], force):
        raise RuntimeError("Firmware mapping mismatch")
    occupied = torch.zeros_like(image, dtype=torch.bool)
    occupied[pixels[:, 0], pixels[:, 1]] = True
    if image[~occupied].any():
        raise RuntimeError("Absent firmware cells contain force")
    centroid = (data.taxel_points_w[0] * force[:, None]).sum(0) / force.sum().clamp(min=1e-8)
    analytic = (data.taxel_points_w[0] - probe.data.root_link_pose_w.torch[0, :3]).norm(dim=-1) - 0.003
    contacting = depths < 0
    depth_error = (depths[contacting] - analytic[contacting]).abs().max().item() if contacting.any() else None
    projected = max(0.0, -float(torch.dot(sensor.get_filtered_normal_force_w()[0], sensor.penetration_normal_w[0])))
    return {
        "taxel": index,
        "penetration_m": penetration,
        "load_n": force.sum().item(),
        "centroid_error_m": (centroid - target).norm().item(),
        "depth_m": depths[index].item(),
        "analytic_depth_error_m": depth_error,
        "projected_normal_load_n": projected,
        "finite": bool(torch.isfinite(force).all() and torch.isfinite(depths).all()),
        "nonnegative": bool((force >= 0).all()),
        "max_abs_depth_m": depths.abs().max().item(),
    }


def check_case(case, penetration):
    import numpy as np

    if not case["finite"] or not case["nonnegative"]:
        raise RuntimeError("Invalid tactile values")
    if not np.isclose(case["load_n"], case["projected_normal_load_n"], rtol=1e-5, atol=1e-6):
        raise RuntimeError("Force conservation failed")
    if penetration <= 0:
        if case["load_n"] != 0 or case["max_abs_depth_m"] != 0:
            raise RuntimeError("Separated/touching fixture reports penetration or force")
    else:
        centroid_limit = 0.0005 if penetration == 0.0005 else 0.0008
        if case["load_n"] <= 0.1 or case["centroid_error_m"] >= centroid_limit or case["depth_m"] >= 0:
            raise RuntimeError("Localized contact failed")
        if case["analytic_depth_error_m"] is None or case["analytic_depth_error_m"] >= 0.00005:
            raise RuntimeError("Analytic sphere depth failed")


def verify_subset(scene, context, sensor):
    import torch
    import warp as wp

    probe = scene["probe"]
    sensor.update(1 / 240)
    point = sensor.data.taxel_points_w[1, 400].clone()
    normal = sensor.data.taxel_normals_w[1, 400].clone()
    pose = probe.data.root_link_pose_w.torch[1:2].clone()
    pose[0, :3] = point + 0.0025 * normal
    pose[0, 3:] = pose.new_tensor([0, 0, 0, 1])
    probe.write_root_pose_to_sim_index(root_pose=pose, env_ids=[1])
    probe.write_root_velocity_to_sim_index(root_velocity=torch.zeros(1, 6, device=context.device), env_ids=[1])
    scene.write_data_to_sim()
    context.step(render=False)
    scene.update(1 / 240)
    old = {
        name: getattr(sensor._data, name)[0].clone()
        for name in ("normal_forces", "depths", "depth_dots", "taxel_points_w", "taxel_normals_w", "taxel_rot_w")
    }
    sensor._update_buffers_impl(wp.array([False, True], dtype=wp.bool, device=context.device))
    for name, value in old.items():
        if not torch.equal(getattr(sensor._data, name)[0], value):
            raise RuntimeError("Subset update changed untouched environment")
    force = sensor._data.normal_forces[1]
    centroid = (sensor._data.taxel_points_w[1] * force[:, None]).sum(0) / force.sum().clamp(min=1e-8)
    error = (centroid - point).norm().item()
    if force.sum() <= 0.1 or error >= 0.001:
        raise RuntimeError("Subset contact localization failed")
    return {"load_n": force.sum().item(), "centroid_error_m": error, "untouched_buffers_preserved": True}


if __name__ == "__main__":
    main()
