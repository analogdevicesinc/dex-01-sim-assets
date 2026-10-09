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

import argparse
import hashlib
import json
import tempfile
from pathlib import Path

if __name__ == "__main__":
    bootstrap = argparse.ArgumentParser(add_help=False)
    bootstrap.add_argument("-h", "--help", action="store_true")
    bootstrap.add_argument("--stationary-press", action="store_true")
    bootstrap.add_argument("--verification-report")
    startup_args, _ = bootstrap.parse_known_args()
    if startup_args.stationary_press:
        import runpy

        from dex01_articulated_press import run_with_report

        if startup_args.help:
            runpy.run_path(str(Path(__file__).resolve()), run_name="_dex01_hand_run")
            raise SystemExit(0)
        # Enter the report lifecycle before importing or launching Isaac Sim.
        run_with_report(
            lambda: runpy.run_path(str(Path(__file__).resolve()), run_name="_dex01_hand_run"),
            startup_args.verification_report,
        )
        raise SystemExit(0)

from isaaclab.app import AppLauncher

REPO_ROOT = Path(__file__).resolve().parents[1]
DEX01_REFERENCE = json.loads((REPO_ROOT / "assets/sensors/dex01/reference.json").read_text())

parser = argparse.ArgumentParser(description="DEX-01 tactile sensors on a Tesollo DG-5F hand.")
parser.add_argument("--record", action="store_true", help="Render an offscreen camera and write a video.")
parser.add_argument("--video-dir", default=str(REPO_ROOT / "videos"), help="Directory the recording is written to.")
parser.add_argument("--max-steps", type=int, default=0, help="Stop after this many steps (0 runs continuously).")
parser.add_argument("--verification-report", help="Write per-step force and pose observations as JSON.")
parser.add_argument("--snapshot", help="Save one rendered RGB frame (requires --enable_cameras).")
parser.add_argument("--inspection-dir", help="Save CAD views from multiple angles (requires --enable_cameras).")
parser.add_argument("--debug-vis", action="store_true", help="Show markers at the taxel sampling positions.")
parser.add_argument(
    "--flat-indenter", action="store_true", help="Use a flat 0.1 kg verification load instead of the gear."
)
parser.add_argument(
    "--verify-thumb", action="store_true", help="Press the thumb with a local spherical probe after the demo cycle."
)

parser.add_argument("--no-tactile-ui", action="store_true", help="Hide the streamed tactile overlay.")
parser.add_argument("--local-plot", action="store_true", help="Also open the separate desktop matplotlib plot.")
parser.add_argument(
    "--stationary-press", action="store_true", help="Articulate finger 2 against a fixed gear with the wrist fixed."
)
parser.add_argument(
    "--press-force-n", type=float, default=1.0, help="Measured normal-load target for the bounded demo controller."
)
parser.add_argument(
    "--press-time-scale",
    type=float,
    default=0.25,
    help="Scale the joint-motion cycle duration (default: 2.5 simulation seconds).",
)

AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
if not 0.1 <= args_cli.press_time_scale <= 1:
    parser.error("--press-time-scale must be in [0.1, 1]")
if not 0 < args_cli.press_force_n <= 2:
    parser.error("--press-force-n must be in (0, 2] for this fixture")
if args_cli.stationary_press and (args_cli.flat_indenter or args_cli.verify_thumb):
    parser.error("Stationary pressing uses the gear; run flat-load/thumb verification separately")
if args_cli.stationary_press and (args_cli.record or args_cli.snapshot or args_cli.inspection_dir):
    parser.error("Stationary pressing supports the live GUI and JSON report; use the gravity demo for recordings")
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import isaaclab.sim as sim_utils
import numpy as np
import omni.usd
import torch
from dex01_sim_asset import TactileSensorCfg, taxel_patterns, vis_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sensors.camera import CameraCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR
from pxr import Usd, UsdGeom

# Demo parameters, not measured sensor or hand properties. Keep values fixed
# when comparing with the known-load verification results.
GRAVITY_M_S2 = 9.81
PHYSICS_DT_S = 1.0 / 240.0  # Validated with this compliant pad and 0.1 kg load.
INDENTER_MASS_KG = 0.1  # Known-load reference: mass * gravity = 0.981 N.
GEAR_DISPLAY_SCALE = 0.15  # Only the demo gear: keep its footprint on the active pad.
CONTACT_OFFSET_M = 0.0001  # PhysX starts generating contacts 0.1 mm before the surface.
POSITION_SOLVER_ITERATIONS = 192  # Conservative inherited setting for compliant contact.
VELOCITY_SOLVER_ITERATIONS = 4  # Keeps settled-load momentum residual within tolerance.
FLAT_INDENTER_SIZE_M = (0.006, 0.006, 0.006)  # 6 mm cube fits the flat sensing region.
THUMB_PROBE_RADIUS_M = 0.003  # Localized spherical contact, not a hardware dimension.
THUMB_PRESS_DEPTH_M = 0.001
THUMB_APPROACH_SPEED_M_S = 0.05
PROBE_SDF_RESOLUTION = 128  # Numerical field resolution for the generated test objects.
HORIZONTAL_NORMAL_MIN_Z = 0.995  # Within about 5.7 degrees of vertical for gravity loading.
OVERLAP_RADIUS_TOLERANCE_M = 1e-6  # Numerical allowance; not manufacturing clearance.


def finger_sensor(finger: int) -> TactileSensorCfg:
    """Sensor configuration for one DG-5F fingertip pad.

    The asset and taxel pattern share the explicit mount transform in reference.json.
    """
    return TactileSensorCfg(
        prim_path=f"{{ENV_REGEX_NS}}/Robot/rl_dg_{finger}_sensor",
        mesh_prim_path="/World/envs/env_.*/Asset/factory_gear_medium",
        pattern_cfg=taxel_patterns.Dex01PatternCfg(),
        offset=TactileSensorCfg.OffsetCfg(
            pos=tuple(DEX01_REFERENCE["mount_translation_m"]),
            rot=tuple(DEX01_REFERENCE["mount_rotation_wxyz"]),
        ),
        debug_vis=args_cli.debug_vis and not args_cli.record,
    )


@configclass
class TactileSensorSceneCfg(InteractiveSceneCfg):
    # Ground
    ground = AssetBaseCfg(prim_path="/World/defaultGroundPlane", spawn=sim_utils.GroundPlaneCfg())

    # Light
    dome_light = AssetBaseCfg(
        prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    )

    asset = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Asset",
        spawn=sim_utils.UsdFileCfg(
            usd_path=f"{ISAACLAB_NUCLEUS_DIR}/Factory/factory_gear_medium.usd",
            mass_props=sim_utils.MassPropertiesCfg(mass=INDENTER_MASS_KG),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                articulation_enabled=False,
            ),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                rigid_body_enabled=True,
                linear_damping=0.0,
                angular_damping=0.0,
                max_linear_velocity=1000.0,
                max_angular_velocity=3666.0,
                enable_gyroscopic_forces=True,
                solver_position_iteration_count=POSITION_SOLVER_ITERATIONS,
                solver_velocity_iteration_count=1,
                max_contact_impulse=1e32,
            ),
            collision_props=sim_utils.CollisionPropertiesCfg(
                contact_offset=CONTACT_OFFSET_M,
                rest_offset=0.0,
            ),
            # Keep the indenter footprint inside the active surface; a larger
            # gear bridges onto the rigid backing and does not load the sensor.
            scale=(GEAR_DISPLAY_SCALE,) * 3,
        ),
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.11),
        ),
    )

    robot = ArticulationCfg(
        prim_path="/World/envs/env_.*/Robot",
        spawn=sim_utils.UsdFileCfg(
            usd_path=str(REPO_ROOT / "assets" / "robots" / "tesollo" / "dg5f_right.usda"),
            variants={"fingertip": "dex01"},
            activate_contact_sensors=True,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=True,
                max_depenetration_velocity=5.0,
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=False, solver_position_iteration_count=12, solver_velocity_iteration_count=1
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            joint_pos={f"rj_dg_{i}_{j}": 0.0 for i in range(1, 6) for j in range(1, 5)},
            pos=(0.2, 0.0, 0.05),
            rot=(-0.7071067811865476, 0.0, 0.7071067811865475, 0.0),
        ),
        # Inherited demo joint-holding parameters, not DG-5F hardware specifications.
        actuators={
            "fingers": ImplicitActuatorCfg(
                joint_names_expr=["rj_dg_[1-5]_[1-4]"],
                effort_limit=17.0,
                velocity_limit=0.175,
                stiffness=200.0,
                damping=80.0,
            ),
        },
    )

    finger1_sensor = finger_sensor(1)
    finger2_sensor = finger_sensor(2)
    finger3_sensor = finger_sensor(3)
    finger4_sensor = finger_sensor(4)
    finger5_sensor = finger_sensor(5)


# Clearance left between the gear and the taxel face so it settles onto the pad under gravity
# instead of starting interpenetrated.
SETTLE_CLEARANCE = 0.0005

# Fingers the gear is rested on, in order. The thumb is excluded: at the zero joint pose its pad
# faces the other fingers, so a gear placed on it lands inside them.
PRESSED_FINGERS = (2, 3, 4, 5)

# 1.25 seconds per finger at PHYSICS_DT_S; the final 20% is checked as settled load.
STEPS_PER_FINGER = 300

# Simulation steps between printed force reports (12 Hz at the demo timestep).
REPORT_INTERVAL = 20

# Simulation steps between heatmap redraws. Drawing every physics step would dominate the loop.
PLOT_INTERVAL = 5


def gear_half_height() -> float:
    """Half the spawned gear's vertical extent, i.e. how far its underside sits below its origin."""
    stage = omni.usd.get_context().get_stage()
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    extent = bbox_cache.ComputeWorldBound(stage.GetPrimAtPath("/World/envs/env_0/Asset")).ComputeAlignedRange()
    return 0.5 * extent.GetSize()[2]


def run_simulator(sim, scene):
    robot = scene["robot"]
    asset = scene["asset"]
    sensors = [scene[f"finger{i}_sensor"] for i in range(1, 6)]
    gear_half_h = gear_half_height()
    cycle_steps = STEPS_PER_FINGER * len(PRESSED_FINGERS)
    labels = [f"f{i}" for i in range(1, 6)]
    viewer = vis_utils.LiveDex01Viewer(labels) if args_cli.local_plot else None
    viewport_viewer = vis_utils.ViewportTactileViewer(
        sensors, labels, enabled=not args_cli.no_tactile_ui and sim.has_gui()
    )
    if args_cli.stationary_press:
        run_stationary_press(sim, scene, sensors, viewport_viewer, viewer)
        viewport_viewer.close()
        return

    if args_cli.record or args_cli.snapshot or args_cli.inspection_dir:
        camera = scene["camera"]
        camera_positions = torch.tensor([[0.55, 0.1, 0.75]], device=sim.device)
        camera_targets = torch.tensor([[0.1, 0.0, 0.05]], device=sim.device)
        camera.set_world_poses_from_view(camera_positions, camera_targets)

    if args_cli.record:
        recorder = vis_utils.SceneRecorder(
            camera=camera,
            sensors=sensors,
            visualizer=vis_utils.FiveFingerDex01Visualizer(writer="ffmpeg"),
        )

    sim_dt = sim.get_physics_dt()
    step = 0
    reset_count = 0
    observations = []
    thumb_observation = None
    total_steps = 0
    while simulation_app.is_running():
        if step % cycle_steps == 0:
            if args_cli.record and reset_count > 0:
                video_dir = Path(args_cli.video_dir)
                video_dir.mkdir(parents=True, exist_ok=True)
                # save_video appends the writer's extension, so it takes a path stem.
                recorder.save_video(str(video_dir / Path(__file__).stem), start=10)
                break

            root_state = asset.data.default_root_state.clone()
            root_state[:, :3] += scene.env_origins

            asset.write_root_pose_to_sim(root_state[:, :7])
            asset.write_root_velocity_to_sim(root_state[:, 7:])

            joint_pos = robot.data.default_joint_pos.clone()
            joint_vel = robot.data.default_joint_vel.clone()
            robot.write_joint_state_to_sim(joint_pos, joint_vel)
            scene.reset()

            step = 0
            reset_count += 1

        if step % STEPS_PER_FINGER == 0:
            # Rest the gear on top of the taxel face so the sensor reads its weight. Derived from
            # the taxel positions rather than a hand-tuned fingertip offset: the pad is its own
            # rigid body whose pose differs per finger.
            probe = sensors[PRESSED_FINGERS[step // STEPS_PER_FINGER] - 1]
            taxel_points = probe.data.taxel_points_w
            # A known gravity load needs a horizontal patch; averaging the tip
            # and side faces makes the indenter rock on the curved surface.
            horizontal = probe.data.taxel_normals_w[0, :, 2] > HORIZONTAL_NORMAL_MIN_Z
            if not horizontal.any():
                raise RuntimeError("No upward-facing sensor patch for the gravity-load demonstration")
            root_state = asset.data.default_root_state.clone()
            root_state[:, 0:2] = taxel_points[:, horizontal].mean(dim=1)[:, 0:2]
            # Height still clears the whole pad, otherwise the gear can start inside its far end.
            root_state[:, 2] = taxel_points[..., 2].max(dim=1).values + gear_half_h + SETTLE_CLEARANCE
            asset.write_root_pose_to_sim(root_state[:, :7])
            asset.write_root_velocity_to_sim(torch.zeros_like(root_state[:, 7:]))
            asset.reset()

        scene.write_data_to_sim()
        sim.step()
        scene.update(sim_dt)

        if step % REPORT_INTERVAL == 0:
            # Taxel forces are the net normal load redistributed by depth, so summing recovers it.
            print("  ".join(f"f{i} {s.data.normal_forces[0].sum().item():7.3f}N" for i, s in enumerate(sensors, 1)))

        if args_cli.verification_report:
            observations.append({
                "step": total_steps,
                "target_finger": PRESSED_FINGERS[step // STEPS_PER_FINGER],
                "forces_n": [s.data.normal_forces[0].sum().item() for s in sensors],
                "min_forces_n": [s.data.normal_forces[0].min().item() for s in sensors],
                "gear_position_m": asset.data.root_pos_w[0].cpu().tolist(),
                "sensor_positions_m": [s._view.get_transforms()[0, :3].cpu().tolist() for s in sensors],
                "depth_min_m": [s.data.depths[0].min().item() for s in sensors],
                "contact_forces_w_n": [s.get_filtered_normal_force_w()[0].cpu().tolist() for s in sensors],
                "weighted_normals_w": [
                    (s.data.taxel_normals_w[0] * s.data.normal_forces[0, :, None]).sum(0).cpu().tolist()
                    for s in sensors
                ],
                "gear_velocity_w": asset.data.root_vel_w[0].cpu().tolist(),
            })

        if step % PLOT_INTERVAL == 0:
            viewport_viewer.update()
            if viewer is not None:
                viewer.update([s.get_tactile_image()[0].cpu().numpy() for s in sensors])

        step += 1
        total_steps += 1
        if args_cli.record:
            recorder.step()
        if args_cli.max_steps and total_steps >= args_cli.max_steps:
            break

    viewport_viewer.close()
    if args_cli.verify_thumb:
        probe = scene["thumb_probe"]
        thumb_sensor = scene["thumb_check_sensor"]
        pixels = thumb_sensor.taxel2pixel
        idx = torch.where((pixels == torch.tensor([25, 15], device=sim.device)).all(-1))[0].item()
        target = thumb_sensor.data.taxel_points_w[0, idx]
        normal = thumb_sensor.data.taxel_normals_w[0, idx]
        pose = probe.data.default_root_state[:, :7].clone()
        pose[0, :3] = target + normal * (THUMB_PROBE_RADIUS_M - THUMB_PRESS_DEPTH_M)
        probe.write_root_pose_to_sim(pose)
        velocity = torch.zeros(1, 6, device=sim.device)
        velocity[0, :3] = -normal * THUMB_APPROACH_SPEED_M_S
        probe.write_root_velocity_to_sim(velocity)
        scene.write_data_to_sim()
        sim.step()
        scene.update(sim_dt)
        force = thumb_sensor.data.normal_forces[0]
        thumb_observation = {
            "probe_taxel_force_n": force.sum().item(),
            "probe_centroid_error_m": (
                ((thumb_sensor.data.taxel_points_w[0] * force[:, None]).sum(0) / force.sum().clamp(min=1e-8) - target)
                .norm()
                .item()
            ),
            "gear_filtered_force_n": sensors[0].data.normal_forces[0].sum().item(),
            "net_contact_force_n": thumb_sensor.get_filtered_normal_force_w()[:, None, :].cpu().tolist(),
            "target_position_m": target.cpu().tolist(),
            "target_normal": normal.cpu().tolist(),
            "probe_position_m": probe.data.root_pos_w[0].cpu().tolist(),
            "minimum_depth_m": thumb_sensor.data.depths[0].min().item(),
        }
        print("THUMB", thumb_observation, flush=True)
        assert force.sum() > 0.1
        assert thumb_observation["probe_centroid_error_m"] < 0.001
        assert thumb_observation["gear_filtered_force_n"] == 0

    if args_cli.verification_report:
        Path(args_cli.verification_report).write_text(
            json.dumps(
                {
                    "pattern_sha256": DEX01_REFERENCE["pattern_sha256"],
                    "contact_pattern_sha256": DEX01_REFERENCE["contact_pattern_sha256"],
                    "asset_sha256": hashlib.sha256(
                        (REPO_ROOT / "assets/robots/tesollo/dex01.usda").read_bytes()
                    ).hexdigest(),
                    "hand_asset_sha256": hashlib.sha256(
                        (REPO_ROOT / "assets/robots/tesollo/dg5f_right.usda").read_bytes()
                    ).hexdigest(),
                    "taxels_per_finger": [s.num_taxels for s in sensors],
                    "steps_per_finger": STEPS_PER_FINGER,
                    "physics_dt_s": sim_dt,
                    "indenter": "flat" if args_cli.flat_indenter else "gear",
                    "observations": observations,
                    "thumb": thumb_observation,
                },
                indent=2,
            )
            + "\n"
        )
    if args_cli.snapshot:
        import matplotlib.pyplot as plt

        plt.imsave(args_cli.snapshot, scene["camera"].data.output["rgb"][0].cpu().numpy())
    if args_cli.inspection_dir:
        inspect_hand(sim, scene, robot, asset, sensors, sim_dt)


def press_camera(context_stage, sim):
    viewport = None
    if sim.has_gui():
        from omni.kit.viewport.utility import get_active_viewport

        viewport = get_active_viewport()
    camera_path = "/World/PressDemoCamera"
    camera_prim = context_stage.GetPrimAtPath(camera_path)
    if not camera_prim:
        # Batch physics verification still needs a declared projection for its
        # geometric plan, even though there is no GUI camera to render it.
        camera_geom = UsdGeom.Camera.Define(context_stage, camera_path)
        camera_geom.CreateHorizontalApertureAttr(20.955)
        camera_geom.CreateVerticalApertureAttr(15.2908)
        camera_geom.CreateFocalLengthAttr(24.0)
        camera_prim = camera_geom.GetPrim()
    camera_geom = UsdGeom.Camera(camera_prim)
    if not camera_geom:
        raise RuntimeError(f"No perspective camera at {camera_path}")
    if viewport is not None:
        width, height = viewport.resolution
        if width <= 0 or height <= 0:
            raise RuntimeError("Viewport has no valid render resolution")
        # Match camera projection to the real streaming surface dimensions.
        camera_geom.GetVerticalApertureAttr().Set(camera_geom.GetHorizontalApertureAttr().Get() * height / width)
    if viewport is not None:
        viewport.camera_path = camera_path
    return viewport, camera_path, camera_geom


def prepare_press_view(sim):
    if sim.has_gui():
        import omni.ui as ui

        for window in ui.Workspace.get_windows():
            if window.title in {
                "Simulation Settings",
                "Content",
                "Console",
                "Stage",
                "Property",
                "Layer",
                "Render Settings",
                "Semantics Schema Editor",
            }:
                window.visible = False
        from omni.kit.viewport.utility import get_active_viewport_window

        viewport_window = get_active_viewport_window()
        viewport_window.focus()
        # Refresh layout after panels close and flush camera/pose to renderer.
        for _ in range(8):
            sim.render()


def flush_press_view(sim):
    if sim.has_gui():
        for _ in range(8):
            sim.render()


def set_press_camera_pose(sim, context_stage, camera_path, camera_geom, plan):
    from pxr import Gf

    world = (
        Gf.Matrix4d().SetLookAt(Gf.Vec3d(*plan["eye_m"]), Gf.Vec3d(*plan["target_m"]), Gf.Vec3d(0, 0, 1)).GetInverse()
    )
    transform = Gf.Transform(world)
    position = transform.GetTranslation()
    orientation = transform.GetRotation().GetQuat()
    scene_distance = np.linalg.norm(np.asarray(plan["eye_m"]) - np.asarray(plan["target_m"]))
    camera_geom.CreateClippingRangeAttr((scene_distance / 1000, scene_distance * 100))
    prim = camera_geom.GetPrim()
    prim.GetAttribute("xformOp:translate").Set(position)
    prim.GetAttribute("xformOp:orient").Set(orientation)
    if sim.has_gui():
        import usdrt

        rt_stage = usdrt.Usd.Stage.Attach(omni.usd.get_context().get_stage_id())
        rt_camera = usdrt.Rt.Xformable(rt_stage.GetPrimAtPath(camera_path))
        rt_camera.SetWorldXformFromUsd()
        rt_camera.GetWorldPositionAttr().Set(usdrt.Gf.Vec3d(*position))
        rt_camera.GetWorldOrientationAttr().Set(
            usdrt.Gf.Quatf(orientation.GetReal(), usdrt.Gf.Vec3f(*orientation.GetImaginary()))
        )


def run_stationary_press(sim, scene, sensors, viewport_viewer, local_viewer):
    from dex01_articulated_press import run_articulated_press

    run_articulated_press(
        sim,
        scene,
        sensors,
        viewport_viewer,
        args_cli,
        simulation_app,
        prepare_press_view,
        press_camera,
        set_press_camera_pose,
        flush_press_view,
        local_viewer=local_viewer,
    )


def inspect_hand(sim, scene, robot, asset, sensors, sim_dt):
    """Save multiple angles and conservatively check sensor/hand collision overlap."""
    import matplotlib.pyplot as plt

    directory = Path(args_cli.inspection_dir)
    directory.mkdir(parents=True, exist_ok=True)
    # Move the loose indenter away so it cannot hide the fingertip surfaces.
    pose = asset.data.root_state_w[:, :7].clone()
    pose[0, :3] = torch.tensor([0, 0, 2.0], device=sim.device)
    asset.write_root_pose_to_sim(pose)
    asset.write_root_velocity_to_sim(torch.zeros(1, 6, device=sim.device))
    robot.write_joint_state_to_sim(robot.data.default_joint_pos, torch.zeros_like(robot.data.default_joint_vel))
    scene.reset()
    ground = omni.usd.get_context().get_stage().GetPrimAtPath("/World/defaultGroundPlane")
    if ground:
        UsdGeom.Imageable(ground).MakeInvisible()
    for sensor in sensors:
        sensor.set_debug_vis(False)
    sim.step()
    scene.update(sim_dt)
    # Test the actual articulated pose, not just authored USD transforms.
    from pxr import Gf, UsdPhysics
    from scipy.optimize import linprog
    from scipy.spatial import ConvexHull
    from scipy.spatial.transform import Rotation

    stage = omni.usd.get_context().get_stage()
    cache = UsdGeom.XformCache()
    hulls = {}
    for i, name in enumerate(robot.body_names):
        body = stage.GetPrimAtPath(f"/World/envs/env_0/Robot/{name}")
        inv = cache.GetLocalToWorldTransform(body).GetInverse()
        q = robot.data.body_quat_w[0, i].cpu().numpy()
        rotation = Rotation.from_quat(q[[1, 2, 3, 0]])
        position = robot.data.body_pos_w[0, i].cpu().numpy()
        for prim in Usd.PrimRange(body, Usd.TraverseInstanceProxies()):
            if prim.IsA(UsdGeom.Mesh) and (
                prim.HasAPI(UsdPhysics.CollisionAPI) or "/collisions/" in str(prim.GetPath())
            ):
                mesh = UsdGeom.Mesh(prim)
                points = mesh.GetPointsAttr().Get()
                if not points:
                    continue
                transform = cache.GetLocalToWorldTransform(prim) * inv
                local = np.array([transform.Transform(Gf.Vec3d(*v)) for v in points])
                world = rotation.apply(local) + position
                hulls[str(prim.GetPath())] = (name, world.min(0), world.max(0), ConvexHull(world).equations)
    checks = []
    for path, (name, lo, hi, equations) in hulls.items():
        if not name.endswith("sensor"):
            continue
        for other, (other_name, other_lo, other_hi, other_equations) in hulls.items():
            if other_name == name or other <= path and other_name.endswith("sensor"):
                continue
            if np.any(hi <= other_lo) or np.any(other_hi <= lo):
                continue
            planes = np.vstack([equations, other_equations])
            # A positive-radius inscribed ball proves volume intersection.
            result = linprog(
                [0, 0, 0, -1],
                A_ub=np.column_stack([planes[:, :3], np.ones(len(planes))]),
                b_ub=-planes[:, 3],
                bounds=[(None, None)] * 3 + [(0, None)],
                method="highs",
            )
            if not result.success and result.status != 2:
                raise RuntimeError(f"Overlap solver failed for {name} and {other_name}: {result.message}")
            radius = result.x[3] if result.success else 0.0
            checks.append({"sensor": name, "other": other_name, "intersection_radius_m": float(radius)})
            assert radius < OVERLAP_RADIUS_TOLERANCE_M, f"Collision overlap: {name} with {other_name}: {radius} m"
    assert sum(name.endswith("sensor") for name, *_ in hulls.values()) == 5
    assert len(hulls) >= 25
    (directory / "overlap-check.json").write_text(
        json.dumps({"collision_meshes_checked": len(hulls), "checks": checks}, indent=2) + "\n"
    )
    positions = [s.data.taxel_points_w[0].mean(0) for s in sensors]
    center = torch.stack(positions).mean(0)
    camera = scene["camera"]
    views = {
        "palm": (center + torch.tensor([0.25, -0.25, 0.45], device=sim.device), center),
        "back": (center + torch.tensor([0.2, 0.2, -0.35], device=sim.device), center),
        "thumb_side": (center + torch.tensor([0.45, 0, 0.12], device=sim.device), center),
        "little_side": (center + torch.tensor([-0.45, 0, 0.12], device=sim.device), center),
        "distal": (center + torch.tensor([0, 0.4, 0.12], device=sim.device), center),
    }
    for i, position in enumerate(positions):
        views[f"finger_{i + 1}_close"] = (position + torch.tensor([0.06, -0.07, 0.08], device=sim.device), position)
        body_index = robot.body_names.index(f"rl_dg_{i + 1}_sensor")
        quaternion = robot.data.body_quat_w[0, body_index].cpu().numpy()
        orientation = Rotation.from_quat(quaternion[[1, 2, 3, 0]])
        side_offset = torch.tensor(orientation.apply([0.0, 0.18, 0.01]), device=sim.device, dtype=position.dtype)
        views[f"finger_{i + 1}_side"] = (position + side_offset, position)
    for name, (eye, target) in views.items():
        camera.set_world_poses_from_view(eye.unsqueeze(0), target.unsqueeze(0))
        for _ in range(6):
            sim.render()
        camera.update(sim_dt, force_recompute=True)
        plt.imsave(directory / f"{name}.png", camera.data.output["rgb"][0].cpu().numpy())
    # Also show taxel registration close up.
    for sensor in sensors:
        sensor.set_debug_vis(True)
    camera.set_world_poses_from_view(
        (positions[1] + torch.tensor([0.04, -0.05, 0.06], device=sim.device)).unsqueeze(0),
        positions[1].unsqueeze(0),
    )
    for _ in range(6):
        sim.render()
    camera.update(sim_dt, force_recompute=True)
    plt.imsave(directory / "taxel_registration.png", camera.data.output["rgb"][0].cpu().numpy())


def main():
    sim_cfg = sim_utils.SimulationCfg(
        device=args_cli.device,
        dt=(1.0 / 120.0 if args_cli.stationary_press else PHYSICS_DT_S),
        gravity=(0.0, 0.0, -GRAVITY_M_S2),
        physx=sim_utils.PhysxCfg(
            solver_type=1,
            max_position_iteration_count=POSITION_SOLVER_ITERATIONS,
            bounce_threshold_velocity=0.2,
            friction_offset_threshold=0.01,
            friction_correlation_distance=0.00625,
            gpu_max_rigid_contact_count=2**23,
            gpu_max_rigid_patch_count=2**23,
            gpu_max_num_partitions=1,  # Important for stable simulation.
            max_velocity_iteration_count=VELOCITY_SOLVER_ITERATIONS,
        ),
        physics_material=sim_utils.RigidBodyMaterialCfg(
            static_friction=1.0,
            dynamic_friction=1.0,
        ),
    )
    sim = sim_utils.SimulationContext(sim_cfg)

    sim.set_camera_view([0.55, 0.1, 0.75], [0.1, 0.0, 0.05])

    scene_cfg = TactileSensorSceneCfg(num_envs=1, env_spacing=1.0)
    if args_cli.stationary_press:
        # Keep the original world-fixed wrist. Only finger actuators move.
        scene_cfg.robot.actuators["fingers"].stiffness = 2000.0
        scene_cfg.robot.actuators["fingers"].damping = 40.0
        scene_cfg.robot.actuators["fingers"].velocity_limit = None
        scene_cfg.robot.actuators["fingers"].velocity_limit_sim = 3.0
        scene_cfg.robot.spawn.articulation_props.solver_position_iteration_count = POSITION_SOLVER_ITERATIONS
        scene_cfg.robot.spawn.articulation_props.solver_velocity_iteration_count = VELOCITY_SOLVER_ITERATIONS
        # Stationary contact does not use weight calibration. Fit a larger
        # gear on the 22 mm pad so its actual outline is resolved by more taxels.
        source_gear_stage = Usd.Stage.Open(scene_cfg.asset.spawn.usd_path)
        gear_bound = (
            UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
            .ComputeWorldBound(source_gear_stage.GetDefaultPrim())
            .ComputeAlignedRange()
        )
        with np.load(REPO_ROOT / DEX01_REFERENCE["contact_pattern_path"]) as pattern:
            poses = pattern["poses"]
        from scipy.spatial.transform import Rotation

        q = np.array(DEX01_REFERENCE["mount_rotation_wxyz"])
        rotation = Rotation.from_quat(q[[1, 2, 3, 0]])
        normals = rotation.apply(poses[:, :, 2])
        flat = normals[:, 2] > HORIZONTAL_NORMAL_MIN_Z
        patch_points = rotation.apply(poses[flat, :, 3])
        pad_width = min(np.ptp(patch_points, axis=0)[:2])
        # Leave a quarter of the pad width as lateral clearance; resolve the
        # remaining gear face using the actual available sensor footprint.
        gear_scale = 0.75 * pad_width / max(gear_bound.GetSize()[0], gear_bound.GetSize()[1])
        scene_cfg.asset.spawn.scale = (gear_scale,) * 3
        print(f"[PRESS]: pad width {pad_width * 1000:.2f} mm, gear width {0.75 * pad_width * 1000:.2f} mm", flush=True)
        scene_cfg.asset.spawn.visual_material = sim_utils.PreviewSurfaceCfg(diffuse_color=(0.9, 0.45, 0.05))
        scene_cfg.asset.spawn.rigid_props.solver_position_iteration_count = POSITION_SOLVER_ITERATIONS
        scene_cfg.asset.spawn.rigid_props.kinematic_enabled = True
        scene_cfg.asset.spawn.rigid_props.disable_gravity = True
        scene_cfg.asset.spawn.activate_contact_sensors = True
    if args_cli.flat_indenter:
        import trimesh
        from pxr import Gf, Sdf, UsdPhysics

        path = Path(tempfile.mkdtemp(prefix="dex01-load-")) / "load.usda"
        stage = Usd.Stage.CreateNew(str(path))
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        root = UsdGeom.Xform.Define(stage, "/factory_gear_medium").GetPrim()
        stage.SetDefaultPrim(root)
        UsdPhysics.RigidBodyAPI.Apply(root)
        UsdPhysics.MassAPI.Apply(root).CreateMassAttr(INDENTER_MASS_KG)
        box = trimesh.creation.box(extents=FLAT_INDENTER_SIZE_M)
        mesh = UsdGeom.Mesh.Define(stage, "/factory_gear_medium/mesh")
        mesh.CreatePointsAttr([Gf.Vec3f(*v) for v in box.vertices])
        mesh.CreateFaceVertexCountsAttr([3] * len(box.faces))
        mesh.CreateFaceVertexIndicesAttr(box.faces.flatten().tolist())
        mesh.CreateSubdivisionSchemeAttr("none")
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("sdf")
        mesh.GetPrim().AddAppliedSchema("PhysxSDFMeshCollisionAPI")
        mesh.GetPrim().CreateAttribute("physxSDFMeshCollision:sdfResolution", Sdf.ValueTypeNames.Int).Set(
            PROBE_SDF_RESOLUTION
        )
        stage.GetRootLayer().Save()
        scene_cfg.asset.spawn = sim_utils.UsdFileCfg(
            usd_path=str(path),
            mass_props=sim_utils.MassPropertiesCfg(mass=INDENTER_MASS_KG),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                solver_position_iteration_count=POSITION_SOLVER_ITERATIONS,
                solver_velocity_iteration_count=VELOCITY_SOLVER_ITERATIONS,
            ),
            collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=CONTACT_OFFSET_M, rest_offset=0.0),
            activate_contact_sensors=True,
        )
        for finger in range(1, 6):
            setattr(
                scene_cfg,
                f"finger{finger}_sensor",
                finger_sensor(finger).replace(mesh_prim_path="/World/envs/env_.*/Asset"),
            )
    if args_cli.record or args_cli.snapshot or args_cli.inspection_dir:
        scene_cfg.camera = CameraCfg(
            prim_path="{ENV_REGEX_NS}/Camera",
            update_period=0,
            height=480,
            width=640,
            data_types=["rgb"],
            spawn=sim_utils.PinholeCameraCfg(focal_length=40.0),
        )

    if args_cli.verify_thumb:
        import trimesh
        from pxr import Gf, Sdf, UsdPhysics

        path = Path(tempfile.mkdtemp(prefix="dex01-thumb-")) / "probe.usda"
        stage = Usd.Stage.CreateNew(str(path))
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        root = UsdGeom.Xform.Define(stage, "/Probe").GetPrim()
        stage.SetDefaultPrim(root)
        UsdPhysics.RigidBodyAPI.Apply(root)
        UsdPhysics.MassAPI.Apply(root).CreateMassAttr(INDENTER_MASS_KG)
        sphere = trimesh.creation.icosphere(subdivisions=3, radius=THUMB_PROBE_RADIUS_M)
        mesh = UsdGeom.Mesh.Define(stage, "/Probe/mesh")
        mesh.CreatePointsAttr([Gf.Vec3f(*v) for v in sphere.vertices])
        mesh.CreateFaceVertexCountsAttr([3] * len(sphere.faces))
        mesh.CreateFaceVertexIndicesAttr(sphere.faces.flatten().tolist())
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("sdf")
        mesh.GetPrim().AddAppliedSchema("PhysxSDFMeshCollisionAPI")
        mesh.GetPrim().CreateAttribute("physxSDFMeshCollision:sdfResolution", Sdf.ValueTypeNames.Int).Set(
            PROBE_SDF_RESOLUTION
        )
        stage.GetRootLayer().Save()
        scene_cfg.thumb_probe = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/ThumbProbe",
            spawn=sim_utils.UsdFileCfg(
                usd_path=str(path),
                rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=True),
                mass_props=sim_utils.MassPropertiesCfg(mass=INDENTER_MASS_KG),
                collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=CONTACT_OFFSET_M, rest_offset=0.0),
                activate_contact_sensors=True,
            ),
            init_state=RigidObjectCfg.InitialStateCfg(pos=(0, 0, 2)),
        )
        scene_cfg.thumb_check_sensor = finger_sensor(1).replace(mesh_prim_path="/World/envs/env_.*/ThumbProbe")

    scene = InteractiveScene(scene_cfg)
    if args_cli.stationary_press:
        camera = UsdGeom.Camera.Define(omni.usd.get_context().get_stage(), "/World/PressDemoCamera")
        camera.CreateHorizontalApertureAttr(20.955)
        camera.CreateVerticalApertureAttr(15.2908)
        camera.CreateFocalLengthAttr(24.0)
        camera.AddTranslateOp().Set((0, 0, 1))
        camera.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble)

    sim.reset()

    print("[INFO]: Setup complete...")
    run_simulator(sim, scene)


if __name__ in {"__main__", "_dex01_hand_run"}:
    from dex01_articulated_press import run_with_report

    try:
        run_with_report(main, args_cli.verification_report if args_cli.stationary_press else None)
    finally:
        # Prevent Isaac Lab's interactive stop callback from rendering forever
        # when this standalone script closes its app after an error or test.
        context = sim_utils.SimulationContext.instance()
        if context is not None:
            context._disable_app_control_on_stop_handle = True
        simulation_app.close()
