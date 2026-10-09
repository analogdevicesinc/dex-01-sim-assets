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

"""Newton scenes and actuator-driven DEX-01 demonstrations."""

import hashlib
import importlib.metadata
import json
import subprocess
import sys
import time

import isaaclab.sim as sim
import numpy as np
import torch
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.app import launch_simulation
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sensors import CameraCfg, ContactSensorCfg
from isaaclab.sim import SimulationCfg, SimulationContext
from isaaclab.sim.schemas import UsdPhysicsCollisionCfg, UsdPhysicsRigidBodyCfg
from isaaclab.utils import configclass
from isaaclab.utils.math import quat_apply, quat_from_matrix
from isaaclab_newton.physics import MJWarpSolverCfg, NewtonCfg, NewtonManager
from isaaclab_newton.renderers import NewtonWarpRendererCfg
from isaaclab_newton.sim.schemas import (
    MujocoCollisionCfg,
    MujocoRigidBodyCfg,
    NewtonArticulationCfg,
    NewtonSDFCollisionCfg,
)

from ..taxel_patterns import Dex01PatternCfg
from ..vis_utils import ViewportTactileViewer
from .assets import (
    indenter_mesh,
    prepare_hand_asset,
    prepare_sensor_asset,
    write_indenter,
)
from .tactile_sensor import TactileSensor, TactileSensorCfg


def rigid_state(asset):
    return lambda: (asset.data.root_link_pose_w.torch, asset.data.root_com_pos_w.torch, asset.data.root_com_vel_w.torch)


def link_state(robot, index):
    return lambda: (
        robot.data.body_link_pose_w.torch[:, index],
        robot.data.body_com_pose_w.torch[:, index, :3],
        robot.data.body_com_vel_w.torch[:, index],
    )


def make_scene(args, root, mesh):
    @configclass
    class SceneCfg(InteractiveSceneCfg):
        pass

    cfg = SceneCfg(num_envs=args.num_envs, env_spacing=0.5)
    cfg.light = AssetBaseCfg(prim_path="/World/Light", spawn=sim.DomeLightCfg(intensity=3000))
    if args.mode == "sensor":
        cfg.pad = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Pad",
            spawn=sim.UsdFileCfg(
                usd_path=prepare_sensor_asset(root / "assets/robots/tesollo/dex01.usda"),
                rigid_props=UsdPhysicsRigidBodyCfg(kinematic_enabled=True),
            ),
        )
    else:
        cfg.robot = ArticulationCfg(
            prim_path="{ENV_REGEX_NS}/Robot",
            spawn=sim.UsdFileCfg(
                usd_path=prepare_hand_asset(root / "assets/robots/tesollo/dg5f_right.usda"),
                articulation_props=NewtonArticulationCfg(self_collision_enabled=False),
                rigid_props={"/.*": [MujocoRigidBodyCfg(gravcomp=1.0)]},
            ),
            init_state=ArticulationCfg.InitialStateCfg(
                pos=(0.2, 0, 0.05),
                rot=(0, 0.7071067811865475, 0, -0.7071067811865476),
            ),
            actuators={
                "fingers": ImplicitActuatorCfg(
                    joint_names_expr=["rj_dg_[1-5]_[1-4]"],
                    joint_effort_limit=17,
                    stiffness=2000,
                    damping=40,
                )
            },
        )
    cfg.probe = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Probe",
        spawn=sim.UsdFileCfg(
            usd_path=write_indenter(mesh),
            rigid_props=UsdPhysicsRigidBodyCfg(kinematic_enabled=args.mode == "press"),
            mass_props=sim.MassPropertiesCfg(mass=0.1),
            collision_props=UsdPhysicsCollisionCfg(),
            visual_material=sim.PreviewSurfaceCfg(diffuse_color=(0.9, 0.2, 0.05)),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0, 0, 2)),
    )
    fingers = [0] if args.mode == "sensor" else list(range(1, 6))
    for finger in fingers:
        path = "{ENV_REGEX_NS}/Pad" if finger == 0 else f"{{ENV_REGEX_NS}}/Robot/rl_dg_{finger}_sensor"
        setattr(
            cfg, f"contact{finger}", ContactSensorCfg(prim_path=path, filter_prim_paths_expr=["{ENV_REGEX_NS}/Probe"])
        )
    if args.record:
        cfg.camera = CameraCfg(
            prim_path="{ENV_REGEX_NS}/Camera",
            spawn=sim.PinholeCameraCfg(clipping_range=(0.001, 10)),
            width=640,
            height=480,
            renderer_cfg=NewtonWarpRendererCfg(),
        )
    return cfg, fingers


def run(args, root, launcher, stop=None):
    reference = json.loads((root / "assets/sensors/dex01/reference.json").read_text())
    physics = NewtonCfg(
        num_substeps=16 if args.mode == "press" else 4,
        collision_decimation=1,
        solver_cfg=MJWarpSolverCfg(use_mujoco_contacts=False, integrator="implicitfast", nconmax=160, njmax=960),
    )
    with launch_simulation(physics) as resolved:
        context = SimulationContext(SimulationCfg(dt=1 / 240, physics=resolved, device=args.device))
        mesh = indenter_mesh(args.indenter, args.mode == "press")
        cfg, fingers = make_scene(args, root, mesh)
        scene = InteractiveScene(cfg)
        sensors = create_sensors(args, fingers, reference)
        context.reset()
        probe = scene["probe"]
        robot = scene["robot"] if args.mode != "sensor" else None
        for finger, sensor in zip(fingers, sensors):
            state = (
                rigid_state(scene["pad"])
                if finger == 0
                else link_state(robot, robot.body_names.index(f"rl_dg_{finger}_sensor"))
            )
            sensor.bind(state, rigid_state(probe), scene[f"contact{finger}"], mesh.vertices, mesh.faces)
            sensor.update(1 / 240)
        viewer = ViewportTactileViewer(
            sensors,
            [f"Finger {i}" for i in fingers],
            enabled=args.viz == "kit" and not args.no_tactile_ui,
            compact=True,
        )
        camera = scene["camera"] if args.record else None
        eye, target = ([0.25, -0.3, 0.3], [0.08, 0, 0.09]) if robot else ([0.06, -0.08, 0.06], [-0.015, 0, -0.005])
        if launcher:
            context.set_camera_view(eye, target)
        if camera:
            camera.set_world_poses_from_view([eye], [target])
        rest = robot.data.default_joint_pos.torch.clone() if robot else None
        selected = sensors[1] if robot else sensors[0]
        joint_ids = [robot.joint_names.index(n) for n in ("rj_dg_2_2", "rj_dg_2_3", "rj_dg_2_4")] if robot else []
        bend = None
        if args.mode == "press":
            bend = robot.data.joint_pos_limits_upper.torch[0, joint_ids] * torch.tensor(
                [0.225, 0.225, 0.10], device=args.device
            )
            endpoint = rest.clone()
            endpoint[:, joint_ids] = bend
            robot.write_joint_state_to_sim_index(position=endpoint, velocity=torch.zeros_like(endpoint))
            robot.actuators.target_command.set_position_index(value=endpoint)
            scene.write_data_to_sim()
            context.step(render=False)
            scene.update(1 / 240)
            selected.update(1 / 240)
            place_probe(probe, selected, 400, float(mesh.extents[2] / 2) - 0.0003, orient=args.indenter != "sphere")
            robot.write_joint_state_to_sim_index(position=rest, velocity=torch.zeros_like(rest))
        else:
            place_probe(probe, selected, 400, float(mesh.extents[2] / 2) + 0.0005, orient=args.indenter != "sphere")
        observations, rgb, tactile = [], [], []
        peak_contacts = 0
        step_times = []
        sensor_times = []
        physics_times = []
        if args.benchmark:
            torch.cuda.reset_peak_memory_stats()
        initial_wrist = robot.data.body_link_pose_w.torch[:, 0].clone() if robot else None
        initial_probe = probe.data.root_link_pose_w.torch.clone()
        started = time.perf_counter()
        print("NEWTON_SCENE_READY", flush=True)
        try:
            for step in range(args.max_steps):
                if args.benchmark:
                    torch.cuda.synchronize()
                    step_started = time.perf_counter()
                if (stop is not None and stop.is_set()) or (launcher and not launcher.app.is_running()):
                    break
                if robot:
                    command = rest.clone()
                    if args.mode == "press":
                        fraction = press_fraction(step / 240)
                        command[:, joint_ids] = fraction * (bend + 0.002)
                    robot.actuators.target_command.set_position_index(value=command)
                if args.mode == "hand" and step % 360 == 0:
                    selected = sensors[1 + (step // 360) % 4]
                    place_probe(
                        probe, selected, 400, float(mesh.extents[2] / 2) + 0.0005, orient=args.indenter != "sphere"
                    )
                scene.write_data_to_sim()
                if args.benchmark:
                    torch.cuda.synchronize()
                    physics_started = time.perf_counter()
                context.step(render=step % 4 == 0)
                if args.report:
                    peak_contacts = check_contact_capacity(peak_contacts)
                if args.benchmark:
                    torch.cuda.synchronize()
                    physics_elapsed = time.perf_counter() - physics_started
                if launcher and args.viz == "none" and step % 8 == 0:
                    launcher.app.update()
                scene.update(1 / 240)
                if args.benchmark:
                    torch.cuda.synchronize()
                    sensing_started = time.perf_counter()
                if not args.no_sensing:
                    for sensor in sensors:
                        sensor.update(1 / 240)
                    loads = [sensor.data.normal_forces[0].sum().item() for sensor in sensors]
                else:
                    loads = [0.0] * len(sensors)
                if not np.isfinite(loads).all():
                    raise RuntimeError("Nonfinite tactile force")
                if args.benchmark:
                    torch.cuda.synchronize()
                    sensing_elapsed = time.perf_counter() - sensing_started
                if step % 12 == 0:
                    viewer.update(f"Newton {args.mode} | step {step}")
                if args.report:
                    entry = {
                        "step": step,
                        "forces_n": loads,
                        "elapsed_wall_s": time.perf_counter() - started,
                        "normal_force_w_n": selected.get_filtered_normal_force_w()[0].cpu().tolist(),
                        "probe_pose": probe.data.root_link_pose_w.torch[0].cpu().tolist(),
                        "probe_velocity_w": probe.data.root_com_vel_w.torch[0].cpu().tolist(),
                        "target_finger": fingers[sensors.index(selected)],
                        "min_depth_m": selected.data.depths[0].min().item(),
                        "penetration_normal_w": selected.penetration_normal_w[0].cpu().tolist(),
                        "tip_position_m": selected.data.taxel_points_w[0].mean(0).cpu().tolist(),
                        "taxel_forces_n": selected.data.normal_forces[0].cpu().tolist(),
                        "frame_n": selected.get_tactile_image()[0].cpu().tolist(),
                    }
                    if robot:
                        entry["joints_rad"] = robot.data.joint_pos.torch[0].cpu().tolist()
                        entry["wrist_pose"] = robot.data.body_link_pose_w.torch[0, 0].cpu().tolist()
                    observations.append(entry)
                if camera and step > 60 and step % 8 == 0:
                    rgb.append(camera.data.output["rgb"].torch[0, :, :, :3].cpu().numpy().copy())
                    tactile.append([sensor.get_tactile_image()[0].cpu().numpy().copy() for sensor in sensors])
                if args.benchmark:
                    torch.cuda.synchronize()
                    if step >= 120:
                        step_times.append(time.perf_counter() - step_started)
                        sensor_times.append(sensing_elapsed)
                        physics_times.append(physics_elapsed)
        finally:
            if args.report:
                args.report.parent.mkdir(parents=True, exist_ok=True)
                args.report.write_text(
                    json.dumps(
                        {
                            "backend": "newton",
                            "failure": None,
                            "interrupted": bool(stop is not None and stop.is_set()),
                            "mode": args.mode,
                            "physics_dt_s": 1 / 240,
                            "taxels_per_sensor": [s.num_taxels for s in sensors],
                            "taxel2pixel": selected.taxel2pixel.cpu().tolist(),
                            "indenter": args.indenter,
                            "joint_ids": joint_ids,
                            "fixed_base": robot.is_fixed_base if robot else None,
                            "initial_wrist": initial_wrist.cpu().tolist() if robot else None,
                            "initial_probe": initial_probe.cpu().tolist(),
                            "solver_cfg": physics.to_dict(),
                            "peak_contacts": peak_contacts,
                            "provenance": provenance(root),
                            "observations": observations,
                        },
                        indent=2,
                    )
                    + "\n"
                )
        if camera:
            save_recording(args.record, rgb, tactile, sensors)
        if args.benchmark:
            write_benchmark(args, root, step_times, sensor_times, physics_times)
        viewer.close()


def provenance(root):
    packages = {
        name: importlib.metadata.version(name)
        for name in ("isaaclab", "newton", "warp-lang", "mujoco", "mujoco-warp", "torch", "usd-exchange")
    }
    gpu = subprocess.run(
        ["nvidia-smi", "--query-gpu=name,driver_version,memory.total", "--format=csv,noheader"],
        capture_output=True,
        text=True,
        timeout=10,
    )
    return {
        "command": sys.argv,
        "packages": packages,
        "gpu": gpu.stdout,
        "gpu_returncode": gpu.returncode,
        "source_hashes": {
            str(path.relative_to(root)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in (
                root / "assets/sensors/dex01/reference.json",
                root / "source/dex01_sim_asset/dex01_sim_asset/newton/tactile_sensor.py",
            )
        },
    }


def place_probe(probe, sensor, taxel, distance, orient=False):
    data = sensor.data
    if orient:
        # The flat region normal is link +Z; selecting an arbitrary curved taxel
        # can put the indenter beside the pad or facing another finger.
        link_pose, _, _ = sensor.frame_state()
        face_normal = quat_apply(
            link_pose[:, 3:], torch.tensor([0.0, 0.0, 1.0], device=link_pose.device).expand(len(link_pose), 3)
        )
        mask = (data.taxel_normals_w * face_normal[:, None]).sum(-1) > 0.995
        if (mask.sum(-1) < 10).any():
            raise ValueError("No planar contact patch found")
        points = (data.taxel_points_w * mask[:, :, None]).sum(1) / mask.sum(1)[:, None]
        normals = face_normal
    else:
        points, normals = data.taxel_points_w[:, taxel], data.taxel_normals_w[:, taxel]
    pose = torch.zeros(len(points), 7, device=points.device)
    pose[:, :3] = points + distance * normals
    pose[:, 6] = 1
    if orient:
        x = torch.cross(normals, torch.tensor([1.0, 0, 0], device=points.device).expand_as(normals), dim=-1)
        fallback = torch.cross(normals, torch.tensor([0.0, 1.0, 0], device=points.device).expand_as(normals), dim=-1)
        x = torch.where((x.norm(dim=-1) < 0.1)[:, None], fallback, x)
        x = x / x.norm(dim=-1, keepdim=True)
        y = torch.cross(normals, x, dim=-1)
        pose[:, 3:] = quat_from_matrix(torch.stack([x, y, normals], dim=-1))
    probe.write_root_pose_to_sim_index(root_pose=pose)
    probe.write_root_velocity_to_sim_index(root_velocity=torch.zeros(len(points), 6, device=points.device))


def save_recording(path, rgb, frames, sensors):
    import imageio_ffmpeg

    from ..vis_utils.viewport import force_frame_rgba

    if not rgb or np.std(rgb[-1]) < 1:
        raise RuntimeError("Recording camera has no meaningful image output")
    path.parent.mkdir(parents=True, exist_ok=True)
    width = max(rgb[0].shape[1], 128 * len(sensors))
    height = rgb[0].shape[0] + 128
    writer = imageio_ffmpeg.write_frames(str(path), (width, height), fps=30)
    writer.send(None)
    try:
        for image, tactile in zip(rgb, frames):
            composed = np.zeros((height, width, 3), dtype=np.uint8)
            composed[: image.shape[0], : image.shape[1]] = image
            vmax = max(0.01, max(float(f.max()) for f in tactile))
            for index, (sensor, frame) in enumerate(zip(sensors, tactile)):
                panel = force_frame_rgba(frame, sensor.taxel2pixel.cpu().numpy(), vmax)[:, :, :3]
                panel = np.repeat(np.repeat(panel, 4, 0), 4, 1)
                composed[-128:, index * 128 : (index + 1) * 128] = panel
            writer.send(composed)
    finally:
        writer.close()


def press_fraction(time_s):
    t = time_s % 2.5
    if t < 0.25:
        return 0.0
    if t < 1.0:
        return min(1.0, (t - 0.25) / 0.75)
    if t < 1.5:
        return 1.0
    return max(0.0, 1.0 - (t - 1.5) / 0.65)


def write_benchmark(args, root, step_times, sensor_times, physics_times):
    if not step_times:
        raise ValueError("Benchmark requires more than 120 warm-up steps")
    args.benchmark.parent.mkdir(parents=True, exist_ok=True)
    args.benchmark.write_text(
        json.dumps(
            {
                "num_envs": args.num_envs,
                "mode": args.mode,
                "sensing": not args.no_sensing,
                "steps": len(step_times),
                "median_step_s": float(np.median(step_times)),
                "p95_step_s": float(np.percentile(step_times, 95)),
                "median_physics_s": float(np.median(physics_times)),
                "median_sensor_s": float(np.median(sensor_times)),
                "sim_seconds_per_wall_second": len(step_times) / 240 / sum(step_times),
                "peak_torch_vram_bytes": torch.cuda.max_memory_allocated(),
                "provenance": provenance(root),
            },
            indent=2,
        )
        + "\n"
    )


def create_sensors(args, fingers, reference):
    sensors = []
    for finger in fingers:
        path = "/World/envs/env_.*/Pad" if finger == 0 else f"/World/envs/env_.*/Robot/rl_dg_{finger}_sensor"
        sensors.append(
            TactileSensor(
                TactileSensorCfg(
                    prim_path=path,
                    mesh_prim_path="/World/envs/env_.*/Probe",
                    pattern_cfg=Dex01PatternCfg(),
                    offset_pos=tuple(reference["mount_translation_m"]),
                    offset_rot_wxyz=tuple(reference["mount_rotation_wxyz"]),
                    debug_vis=args.debug_vis,
                )
            )
        )
    for finger in fingers:
        body_path = "/World/envs/env_0/Pad" if finger == 0 else f"/World/envs/env_0/Robot/rl_dg_{finger}_sensor"
        sim.schemas.apply_mesh_collision_properties(
            body_path + "/collision",
            [
                NewtonSDFCollisionCfg(
                    sdf_max_resolution=384,
                    sdf_narrow_band_inner=-0.002,
                    sdf_narrow_band_outer=0.002,
                    sdf_padding=0.001,
                )
            ],
        )
        sim.schemas.apply_collision_properties(
            body_path + "/collision",
            [
                MujocoCollisionCfg(
                    condim=3,
                    solref=(0.02, 1.0),
                    solimp=(0.9, 0.95, 0.001, 0.5, 2.0),
                    priority=1,
                )
            ],
        )
    sim.schemas.apply_collision_properties(
        "/World/envs/env_0/Probe/collision",
        [
            MujocoCollisionCfg(
                condim=3,
                solref=(0.02, 1.0),
                solimp=(0.9, 0.95, 0.001, 0.5, 2.0),
                priority=1,
            )
        ],
    )
    return sensors


def check_contact_capacity(peak):
    contacts = NewtonManager.get_contacts()
    count = int(contacts.rigid_contact_count.numpy()[0])
    if count >= contacts.rigid_contact_max:
        raise RuntimeError("Native collision contact capacity exhausted")
    return max(peak, count)
