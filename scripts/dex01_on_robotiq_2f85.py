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

"""Grasp and release a free rigid block with the official 2F-85's tactile fingers."""

import argparse
import json
import sys
import tempfile
from pathlib import Path

BLOCK_MASS_KG = 0.1
HOLD_FORCE_DIFFERENCE_TOLERANCE_N = 0.1
MOMENTUM_TOLERANCE_N_S = 1e-6


def check_report(path):
    """Check a completed live run without launching the simulator."""
    import numpy as np

    report = json.loads(Path(path).read_text())
    if report.get("status") != "complete":
        raise ValueError("The simulation did not complete")
    observations = report["observations"]
    forces = np.array([x["forces_n"] for x in observations])
    hold = np.array([x["phase"] == "hold" for x in observations])
    release = np.array([x["phase"] == "released" for x in observations])
    assert report["taxel_grid_shapes"] == [[32, 32], [32, 32]]
    assert np.isfinite(forces).all() and forces.min() >= 0
    assert hold.sum() >= 30 and release.sum() >= 30
    assert forces[hold].min() > 0.1, "Both pads in every environment must sustain contact"
    assert forces[release].max() < 1e-4, "Pads must unload after opening"
    assert np.max([x["joint_positions_rad"] for x in observations]) > 0.05, "Gripper did not articulate"
    assert report["maximum_image_sum_error_n"] < 1e-5
    assert report["maximum_contact_projection_error_n"] < 0.01
    assert report["maximum_unresolved_hold_contact_n"] < 0.01
    assert np.max(np.abs(forces[hold, :, 0] - forces[hold, :, 1])) < HOLD_FORCE_DIFFERENCE_TOLERANCE_N
    assert report["maximum_hold_force_difference_n"] < HOLD_FORCE_DIFFERENCE_TOLERANCE_N
    assert report["maximum_momentum_residual_n_s"] < MOMENTUM_TOLERANCE_N_S
    for observation in observations:
        contact = np.asarray(observation["normal_forces_on_block_n"]) + np.asarray(
            observation["friction_forces_on_block_n"]
        )
        dv = np.asarray(observation["block_velocity_m_s"]) - np.asarray(observation["previous_block_velocity_m_s"])
        residual = contact.sum(axis=1) * report["physics_dt_s"] - report["block_mass_kg"] * dv
        assert np.max(np.abs(residual)) < MOMENTUM_TOLERANCE_N_S, "Block momentum does not match contact impulses"
    assert report["maximum_base_displacement_m"] < 1e-5
    assert report["maximum_base_rotation_rad"] < 1e-4
    print(f"Passed: both 32 x 32 pads contact and release; hold medians {np.median(forces[hold], axis=0)} N")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sources = parser.add_mutually_exclusive_group()
    sources.add_argument("--gripper-usd", help="Explicit gripper USD URL or local file.")
    sources.add_argument("--asset-root", help="Isaac library root; defaults to Isaac Sim's configured asset root.")
    parser.add_argument(
        "--max-steps", type=int, default=1440, help="240 Hz steps; 1440 completes one six-second cycle."
    )
    parser.add_argument("--num-envs", type=int, default=1)
    parser.add_argument("--verification-report", type=Path)
    parser.add_argument("--check-report", type=Path)
    parser.add_argument("--debug-vis", action="store_true")
    parser.add_argument("--no-tactile-ui", action="store_true")
    preliminary, _ = parser.parse_known_args()
    if preliminary.check_report:
        check_report(preliminary.check_report)
        return

    # Run this checkout's sensor package even when the environment was installed
    # from another checkout (for example, when developing in a Git worktree).
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "source/dex01_sim_asset"))

    from isaaclab.app import AppLauncher

    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    launcher = AppLauncher(args)
    app = launcher.app
    report = {"status": "running", "observations": []}
    try:
        run(args, app, report)
        report["status"] = "complete"
    except BaseException as exc:
        report.update(status="failed", error=f"{type(exc).__name__}: {exc}")
        raise
    finally:
        if args.verification_report:
            args.verification_report.parent.mkdir(parents=True, exist_ok=True)
            args.verification_report.write_text(json.dumps(report, indent=2) + "\n")
        app.close()


def run(args, app, report):
    import isaaclab.sim as sim_utils
    import numpy as np
    import omni.client
    import torch
    import trimesh
    from build_robotiq_asset import (
        ASSET,
        GRIPPER_ASSET_PATH,
        LINK_ROOT,
        MAX_CLOSURE_DEG,
        MOUNT_Y_M,
        MOUNT_Z_M,
        build_gripper,
    )
    from dex01_sim_asset import TactileSensorCfg, taxel_patterns, vis_utils
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
    from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
    from isaaclab.utils import configclass
    from isaacsim.storage.native import get_assets_root_path
    from pxr import Sdf, Usd, UsdGeom, UsdPhysics

    temporary = tempfile.TemporaryDirectory(prefix="dex01-robotiq-")
    directory = Path(temporary.name)
    integrated = directory / "gripper.usda"
    asset_root = None if args.gripper_usd else args.asset_root or get_assets_root_path(skip_check=True)
    source = args.gripper_usd or str(asset_root).rstrip("/") + "/" + GRIPPER_ASSET_PATH
    if omni.client.stat(source)[0] != omni.client.Result.OK:
        raise RuntimeError(f"Cannot resolve Robotiq 2F-85 asset: {source}")
    build_gripper(integrated, args.gripper_usd, ASSET / "finger.usda", asset_root=asset_root)
    report["resolved_gripper_asset"] = source
    block_path = directory / "block.usda"
    stage = Usd.Stage.CreateNew(str(block_path))
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    root = UsdGeom.Xform.Define(stage, "/Block").GetPrim()
    stage.SetDefaultPrim(root)
    UsdPhysics.RigidBodyAPI.Apply(root)
    UsdPhysics.MassAPI.Apply(root).CreateMassAttr(BLOCK_MASS_KG)
    box = trimesh.creation.box(extents=(0.010, 0.030, 0.018))
    mesh = UsdGeom.Mesh.Define(stage, "/Block/mesh")
    mesh.CreatePointsAttr(box.vertices.tolist())
    mesh.CreateFaceVertexCountsAttr([3] * len(box.faces))
    mesh.CreateFaceVertexIndicesAttr(box.faces.ravel().tolist())
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateDisplayColorAttr([(0.2, 0.45, 0.75)])
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("sdf")
    mesh.GetPrim().AddAppliedSchema("PhysxSDFMeshCollisionAPI")
    mesh.GetPrim().CreateAttribute("physxSDFMeshCollision:sdfResolution", Sdf.ValueTypeNames.Int).Set(128)
    stage.GetRootLayer().Save()

    def sensor(side):
        return TactileSensorCfg(
            prim_path=f"{{ENV_REGEX_NS}}/Robot/{LINK_ROOT}/{side}_inner_finger",
            mesh_prim_path="/World/envs/env_.*/Block",
            pattern_cfg=taxel_patterns.RectangularDex01PatternCfg(),
            offset=TactileSensorCfg.OffsetCfg(
                pos=(0.0, -MOUNT_Y_M if side == "left" else MOUNT_Y_M, MOUNT_Z_M),
                rot=(1.0, 0.0, 0.0, 0.0) if side == "left" else (0.0, 0.0, 0.0, 1.0),
            ),
            debug_vis=args.debug_vis,
        )

    @configclass
    class SceneCfg(InteractiveSceneCfg):
        ground = AssetBaseCfg(prim_path="/World/Ground", spawn=sim_utils.GroundPlaneCfg())
        light = AssetBaseCfg(prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=2500))
        robot = ArticulationCfg(
            prim_path="{ENV_REGEX_NS}/Robot",
            spawn=sim_utils.UsdFileCfg(
                usd_path=str(integrated),
                activate_contact_sensors=True,
                rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=True),
                articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                    enabled_self_collisions=False,
                    solver_position_iteration_count=192,
                    solver_velocity_iteration_count=4,
                ),
            ),
            init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, 0.02), joint_pos={".*": 0.0}),
            actuators={
                "drive": ImplicitActuatorCfg(
                    joint_names_expr=["finger_joint"],
                    effort_limit_sim=5.0,
                    velocity_limit_sim=1.0,
                    stiffness=11.25,
                    damping=0.1,
                ),
                "passive": ImplicitActuatorCfg(
                    joint_names_expr=[".*inner.*joint", "right_outer_knuckle_joint"],
                    effort_limit_sim=5.0,
                    velocity_limit_sim=2.0,
                    stiffness=0.0,
                    damping=0.0,
                ),
            },
        )
        block = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Block",
            spawn=sim_utils.UsdFileCfg(
                usd_path=str(block_path),
                activate_contact_sensors=True,
                rigid_props=sim_utils.RigidBodyPropertiesCfg(
                    disable_gravity=True, linear_damping=0.0, angular_damping=0.0
                ),
            ),
            init_state=RigidObjectCfg.InitialStateCfg(pos=(0.0, 0.0, 0.186)),
        )
        left_sensor = sensor("left")
        right_sensor = sensor("right")

    dt = 1.0 / 240.0
    sim = sim_utils.SimulationContext(
        sim_utils.SimulationCfg(
            device=args.device,
            dt=dt,
            physx=sim_utils.PhysxCfg(
                max_position_iteration_count=192, max_velocity_iteration_count=4, gpu_max_num_partitions=1
            ),
        )
    )
    sim.set_camera_view([0.28, -0.30, 0.25], [0.0, 0.0, 0.12])
    scene = InteractiveScene(SceneCfg(num_envs=args.num_envs, env_spacing=0.6))
    sim.reset()
    scene.reset()
    robot = scene["robot"]
    sensors = [scene["left_sensor"], scene["right_sensor"]]
    contact_views = [
        s._physics_sim_view.create_rigid_contact_view(
            s.cfg.prim_path.replace(".*", "*"),
            filter_patterns=[s._counter_body_path],
            max_contact_data_count=1024 * args.num_envs,
        )
        for s in sensors
    ]
    previous_block_velocity = scene["block"].data.root_lin_vel_w.clone()
    joint = robot.find_joints("finger_joint")[0][0]
    base = robot.find_bodies("base_link")[0][0]
    base_position = robot.data.body_pos_w[:, base].clone()
    base_quaternion = robot.data.body_quat_w[:, base].clone()
    ui = vis_utils.ViewportTactileViewer(
        sensors, ["Left", "Right"], enabled=not args.no_tactile_ui and (not args.headless or bool(args.livestream))
    )
    target = torch.zeros(args.num_envs, 1, device=sim.device)
    report.update(
        physics_dt_s=dt,
        num_envs=args.num_envs,
        taxel_grid_shapes=[[32, 32], [32, 32]],
        maximum_image_sum_error_n=0.0,
        maximum_contact_projection_error_n=0.0,
        maximum_unresolved_contact_n=0.0,
        maximum_unresolved_hold_contact_n=0.0,
        maximum_base_displacement_m=0.0,
        maximum_base_rotation_rad=0.0,
        block_mass_kg=BLOCK_MASS_KG,
        maximum_hold_force_difference_n=0.0,
        maximum_momentum_residual_n_s=0.0,
    )
    for step in range(args.max_steps):
        if not app.is_running():
            raise RuntimeError("Simulation closed before completion")
        time = step * dt
        previous = torch.stack([s.data.normal_forces.sum(-1) for s in sensors], dim=-1).max(-1).values
        if time < 0.25:
            phase = "open"
            target.zero_()
        elif time < 3.0:
            phase = "closing"
            target[:, 0] += torch.where(previous < 2.0, 0.20 * dt, -0.20 * dt)
        elif time < 4.0:
            phase = "hold"
            target[:, 0] += torch.clamp(2.0 - previous, -0.5, 0.5) * 0.06 * dt
        elif time < 5.25:
            phase = "opening"
            target -= 0.6 * dt
        else:
            phase = "released"
            target.zero_()
        target.clamp_(0.0, np.deg2rad(MAX_CLOSURE_DEG))
        robot.set_joint_position_target(target, joint_ids=[joint])
        scene.write_data_to_sim()
        sim.step()
        scene.update(dt)
        forces = [s.data.normal_forces.sum(-1) for s in sensors]
        # Newton's law provides a check independent of the taxel redistribution.
        # These views report forces on each finger; the block receives their opposites.
        normal_on_block = -torch.stack([v.get_contact_force_matrix(dt)[:, 0] for v in contact_views], dim=1)
        friction_on_block = torch.zeros_like(normal_on_block)
        for side, view in enumerate(contact_views):
            friction, _, counts, starts = view.get_friction_data(dt)
            for env in range(args.num_envs):
                first, count = int(starts[env, 0]), int(counts[env, 0])
                friction_on_block[env, side] = -friction[first : first + count].sum(0)
        block_velocity = scene["block"].data.root_lin_vel_w.clone()
        net_contact = (normal_on_block + friction_on_block).sum(1)
        residual = net_contact * dt - BLOCK_MASS_KG * (block_velocity - previous_block_velocity)
        report["maximum_momentum_residual_n_s"] = max(
            report["maximum_momentum_residual_n_s"], residual.abs().max().item()
        )
        if phase == "hold":
            report["maximum_hold_force_difference_n"] = max(
                report["maximum_hold_force_difference_n"], (forces[0] - forces[1]).abs().max().item()
            )
        for s, force in zip(sensors, forces):
            image_error = (s.get_tactile_image().sum(dim=(-1, -2)) - force).abs().max().item()
            contact = s._contact_view.get_contact_force_matrix(dt=dt)[:, 0]
            expected = -(contact * s.data.taxel_normals_w[:, 0]).sum(-1)
            active = (s.data.depths < 0.0).any(-1)
            differences = (force - expected.clamp(min=0.0)).abs()
            force_error = differences[active].max().item() if active.any() else 0.0
            unresolved = expected[~active].clamp(min=0.0).max().item() if (~active).any() else 0.0
            report["maximum_image_sum_error_n"] = max(report["maximum_image_sum_error_n"], image_error)
            report["maximum_contact_projection_error_n"] = max(
                report["maximum_contact_projection_error_n"], force_error
            )
            report["maximum_unresolved_contact_n"] = max(report["maximum_unresolved_contact_n"], unresolved)
            if phase == "hold":
                report["maximum_unresolved_hold_contact_n"] = max(
                    report["maximum_unresolved_hold_contact_n"], unresolved
                )
        report["maximum_base_displacement_m"] = max(
            report["maximum_base_displacement_m"],
            (robot.data.body_pos_w[:, base] - base_position).norm(dim=-1).max().item(),
        )
        dot = (robot.data.body_quat_w[:, base] * base_quaternion).sum(-1).abs().clamp(max=1.0)
        report["maximum_base_rotation_rad"] = max(
            report["maximum_base_rotation_rad"], (2 * torch.acos(dot)).max().item()
        )
        if step % 4 == 0:
            report["observations"].append({
                "step": step,
                "phase": phase,
                "joint_positions_rad": robot.data.joint_pos[:, joint].tolist(),
                "target_rad": target[:, 0].tolist(),
                "forces_n": torch.stack(forces, dim=-1).tolist(),
                "normal_forces_on_block_n": normal_on_block.tolist(),
                "friction_forces_on_block_n": friction_on_block.tolist(),
                "block_velocity_m_s": block_velocity.tolist(),
                "previous_block_velocity_m_s": previous_block_velocity.tolist(),
            })
            ui.update(status=f"{phase}: left {forces[0][0]:.2f} N, right {forces[1][0]:.2f} N")
        previous_block_velocity = block_velocity
        if step % 120 == 0:
            print(
                f"step={step} phase={phase} q={robot.data.joint_pos[0, joint].item():.3f} force={[round(f[0].item(), 3) for f in forces]}",
                flush=True,
            )
    ui.close()
    temporary.cleanup()


if __name__ == "__main__":
    main()
