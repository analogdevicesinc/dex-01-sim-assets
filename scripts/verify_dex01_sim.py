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

"""Exercise the actual DEX-01 asset and tactile sensor in a headless PhysX scene."""

import argparse
import hashlib
import json
import tempfile
from pathlib import Path

# Acceptance bounds for this simulation fixture; none are hardware accuracy claims.
REFERENCE_LOAD_MASS_KG = 0.1
STANDARD_GRAVITY_M_S2 = 9.81
REFERENCE_WEIGHT_N = REFERENCE_LOAD_MASS_KG * STANDARD_GRAVITY_M_S2
SETTLED_CYCLE_FRACTION = 0.8  # Check the last 20% of each finger's load period.
LOAD_RELATIVE_TOLERANCE = 0.05  # Individual readings within 5% of the known weight.
CONTACT_MEAN_RELATIVE_TOLERANCE = 0.01
UNLOADED_FORCE_TOLERANCE_N = 1e-5
FORCE_AGREEMENT_TOLERANCE_N = 0.01
SETTLED_SPEED_TOLERANCE_M_S = 0.01
MINIMUM_SETTLED_SAMPLES = 30
MINIMUM_PROBE_CONTACT_FORCE_N = 0.1
THUMB_CENTROID_TOLERANCE_M = 0.001


def check_hand_report(path):
    import numpy as np

    report = json.loads(path.read_text())
    steps_per_finger = report.get("steps_per_finger", 100)
    assert report["taxels_per_finger"] == [738] * 5
    for finger in range(2, 6):
        entries = [
            entry
            for entry in report["observations"]
            if entry["target_finger"] == finger
            and entry["step"] % steps_per_finger >= steps_per_finger * SETTLED_CYCLE_FRACTION
        ]
        readings = np.array([entry["forces_n"] for entry in entries])
        assert len(readings) >= MINIMUM_SETTLED_SAMPLES, f"Missing settling samples for finger {finger}"
        assert np.isfinite(readings).all() and (readings >= 0).all()
        median = np.median(readings[:, finger - 1])
        assert (
            abs(median - REFERENCE_WEIGHT_N) / REFERENCE_WEIGHT_N < LOAD_RELATIVE_TOLERANCE
        ), f"Finger {finger}: {median} N differs from 0.1 kg weight"
        assert np.max(np.delete(readings, finger - 1, axis=1)) < UNLOADED_FORCE_TOLERANCE_N
        if report.get("indenter") == "flat":
            loads = readings[:, finger - 1]
            assert (
                np.max(np.abs(loads - REFERENCE_WEIGHT_N)) / REFERENCE_WEIGHT_N < LOAD_RELATIVE_TOLERANCE
            ), f"Finger {finger}: flat load did not settle"
            # Compare independently filtered PhysX vertical load with the taxel sum.
            contact = np.array([entry["contact_forces_w_n"][finger - 1] for entry in entries])
            assert (
                abs(-contact[:, 2].mean() - REFERENCE_WEIGHT_N) / REFERENCE_WEIGHT_N < CONTACT_MEAN_RELATIVE_TOLERANCE
            )
            assert np.max(np.abs(loads + contact[:, 2])) < FORCE_AGREEMENT_TOLERANCE_N
            speed = np.linalg.norm([entry["gear_velocity_w"][:3] for entry in entries], axis=1)
            assert speed.max() < SETTLED_SPEED_TOLERANCE_M_S
            # Vertical momentum balance provides a second reference independent of
            # the sensor's force redistribution and the steady-load median.
            dt = report["physics_dt_s"]
            vz = np.array([entry["gear_velocity_w"][2] for entry in entries])
            impulse_residual = (-contact[1:, 2] - REFERENCE_WEIGHT_N) * dt - REFERENCE_LOAD_MASS_KG * np.diff(vz)
            assert np.max(np.abs(impulse_residual)) < CONTACT_MEAN_RELATIVE_TOLERANCE * REFERENCE_WEIGHT_N * dt
        print(f"Finger {finger}: {median:.6f} N; unloaded fingers zero")
    thumb = report["thumb"]
    assert thumb["probe_taxel_force_n"] > MINIMUM_PROBE_CONTACT_FORCE_N
    assert thumb["probe_centroid_error_m"] < THUMB_CENTROID_TOLERANCE_M
    assert thumb["gear_filtered_force_n"] == 0
    print("Thumb: localized force and counter-object filtering passed")


parser = argparse.ArgumentParser()
parser.add_argument("--report", default="/tmp/dex01-simulation-verification.json")
parser.add_argument(
    "--transformed-probe", action="store_true", help="Exercise an offset, rotated and scaled collision mesh."
)
parser.add_argument(
    "--sweep-all", action="store_true", help="Check a localized contact at every occupied firmware cell."
)
parser.add_argument("--no-tactile-ui", action="store_true", help="Hide the streamed tactile overlay.")
parser.add_argument("--num-envs", type=int, default=1, choices=[1, 2])
parser.add_argument("--check-hand-report", type=Path, help="Check a saved hand report without launching Isaac Sim.")
known_args, _ = parser.parse_known_args()
if known_args.check_hand_report:
    check_hand_report(known_args.check_hand_report)
    raise SystemExit(0)

from isaaclab.app import AppLauncher

AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app = AppLauncher(args).app

import isaaclab.sim as sim
import numpy as np
import torch
import trimesh
from dex01_sim_asset import TactileSensorCfg, taxel_patterns, vis_utils
from isaaclab.assets import RigidObject, RigidObjectCfg
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
reference = json.loads((ROOT / "assets/sensors/dex01/reference.json").read_text())


def make_probe(path):
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    root = UsdGeom.Xform.Define(stage, "/Probe").GetPrim()
    stage.SetDefaultPrim(root)
    UsdPhysics.RigidBodyAPI.Apply(root)
    UsdPhysics.MassAPI.Apply(root).CreateMassAttr(0.1)
    sphere = trimesh.creation.icosphere(subdivisions=3, radius=0.003 / (1.25 if args.transformed_probe else 1.0))
    mesh = UsdGeom.Mesh.Define(stage, "/Probe/collision")
    mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in sphere.vertices])
    mesh.CreateFaceVertexCountsAttr([3] * len(sphere.faces))
    mesh.CreateFaceVertexIndicesAttr(sphere.faces.flatten().tolist())
    mesh.CreateSubdivisionSchemeAttr("none")
    if args.transformed_probe:
        xf = UsdGeom.Xformable(mesh.GetPrim())
        xf.AddTranslateOp().Set(Gf.Vec3d(0.004, -0.002, 0.001))
        xf.AddRotateXYZOp().Set(Gf.Vec3f(17.0, 25.0, 31.0))
        xf.AddScaleOp().Set(Gf.Vec3f(1.25))
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("sdf")
    mesh.GetPrim().AddAppliedSchema("PhysxSDFMeshCollisionAPI")
    mesh.GetPrim().CreateAttribute("physxSDFMeshCollision:sdfResolution", Sdf.ValueTypeNames.Int).Set(128)
    stage.GetRootLayer().Save()


def main():
    probe_file = Path(tempfile.mkdtemp(prefix="dex01-verify-")) / "probe.usda"
    make_probe(probe_file)
    context = sim.SimulationContext(sim.SimulationCfg(device=args.device, dt=1 / 240, gravity=(0, 0, -9.81)))
    for env in range(args.num_envs):
        sim.create_prim(f"/World/envs/env_{env}", "Xform")
    sensor_body = RigidObject(
        RigidObjectCfg(
            prim_path="/World/envs/env_.*/Sensor",
            spawn=sim.UsdFileCfg(
                usd_path=str(ROOT / "assets/robots/tesollo/dex01.usda"),
                rigid_props=sim.RigidBodyPropertiesCfg(kinematic_enabled=True),
                activate_contact_sensors=True,
            ),
        )
    )
    probe = RigidObject(
        RigidObjectCfg(
            prim_path="/World/envs/env_.*/Probe",
            spawn=sim.UsdFileCfg(
                usd_path=str(probe_file),
                rigid_props=sim.RigidBodyPropertiesCfg(disable_gravity=True, solver_position_iteration_count=192),
                collision_props=sim.CollisionPropertiesCfg(contact_offset=0.0001, rest_offset=0.0),
                activate_contact_sensors=True,
            ),
            init_state=RigidObjectCfg.InitialStateCfg(pos=(0, 0, 0.1)),
        )
    )
    cfg = TactileSensorCfg(
        prim_path="/World/envs/env_.*/Sensor",
        mesh_prim_path="/World/envs/env_.*/Probe",
        pattern_cfg=taxel_patterns.Dex01PatternCfg(),
        offset=TactileSensorCfg.OffsetCfg(
            pos=tuple(reference["mount_translation_m"]), rot=tuple(reference["mount_rotation_wxyz"])
        ),
    )
    sensor = cfg.class_type(cfg)
    context.reset()
    dt = context.get_physics_dt()
    if args.num_envs == 2:
        pose = sensor_body.data.default_root_state[:, :7].clone()
        pose[1, :3] = torch.tensor([0.2, 0.1, 0.07], device=context.device)
        pose[1, 3:] = torch.tensor([np.cos(0.35), 0, np.sin(0.35), 0], device=context.device)
        sensor_body.write_root_pose_to_sim(pose)
        pose = probe.data.default_root_state[:, :7].clone()
        pose[1, :3] = torch.tensor([0.2, 0.1, 0.3], device=context.device)
        probe.write_root_pose_to_sim(pose)
    context.step(render=context.has_gui())
    probe.update(dt)
    sensor_body.update(dt)
    sensor.update(dt, force_recompute=True)
    viewer = vis_utils.ViewportTactileViewer([sensor], ["Probe"], enabled=not args.no_tactile_ui and context.has_gui())
    points = sensor.data.taxel_points_w[0].clone()
    normals = sensor.data.taxel_normals_w[0].clone()
    pixels = sensor.taxel2pixel
    report = {
        "pattern_sha256": reference["pattern_sha256"],
        "contact_pattern_sha256": reference["contact_pattern_sha256"],
        "asset_sha256": hashlib.sha256((ROOT / "assets/robots/tesollo/dex01.usda").read_bytes()).hexdigest(),
        "sensor_code_sha256": hashlib.sha256(
            (ROOT / "source/dex01_sim_asset/dex01_sim_asset/tactile_sensor.py").read_bytes()
        ).hexdigest(),
        "num_taxels": sensor.num_taxels,
        "environments": args.num_envs,
        "transformed_probe": args.transformed_probe,
        "cases": [],
    }
    target_pixels = pixels.cpu().tolist() if args.sweep_all else ((25, 15), (25, 2), (25, 29), (3, 15), (12, 15))
    for row, col in target_pixels:
        index = torch.where((pixels == torch.tensor([row, col], device=pixels.device)).all(-1))[0].item()
        point = points[index]
        normal = normals[index]
        penetrations = (0.0005,) if args.sweep_all else (-0.001, 0.0, 0.0005, 0.001)
        for penetration in penetrations:
            pose = probe.data.default_root_state[:, :7].clone()
            if args.transformed_probe:
                pose[0, 3:] = torch.tensor([np.cos(0.45), 0, 0, np.sin(0.45)], device=context.device)
                q = Gf.Rotation(Gf.Vec3d(0, 0, 1), np.rad2deg(0.9))
                local_offset = torch.tensor(list(q.TransformDir(Gf.Vec3d(0.004, -0.002, 0.001))), device=context.device)
            else:
                local_offset = torch.zeros(3, device=context.device)
            pose[0, :3] = point + normal * (0.003 - penetration) - local_offset
            probe.write_root_pose_to_sim(pose)
            probe.write_root_velocity_to_sim(torch.zeros(args.num_envs, 6, device=context.device))
            for _ in range(1):
                context.step(render=context.has_gui())
                probe.update(dt)
                sensor_body.update(dt)
                sensor.update(dt, force_recompute=True)
            viewer.update(f"Probe cell ({row}, {col}) | indentation {penetration * 1000:.2f} mm")
            force = sensor.data.normal_forces[0].clone()
            net = sensor.get_filtered_normal_force_w()[0].clone()
            depths = sensor.data.depths[0].clone()
            case = {
                "pixel": [row, col],
                "penetration_m": penetration,
                "net_force": net.cpu().tolist(),
                "sum_force": force.sum().item(),
                "min_force": force.min().item(),
                "contact_taxels": int((depths < 0).sum()),
                "min_depth_m": depths.min().item(),
                "force_centroid_m": ((points * force[:, None]).sum(0) / force.sum().clamp(min=1e-8)).cpu().tolist(),
            }
            report["cases"].append(case)
            if not args.sweep_all or len(report["cases"]) % 100 == 0:
                print(case, flush=True)
            assert torch.isfinite(force).all() and force.min() >= 0
            image = sensor.get_tactile_image()[0]
            assert torch.equal(image[pixels[:, 0], pixels[:, 1]], force)
            if args.num_envs == 2:
                assert sensor.data.normal_forces[1].sum() == 0
            if penetration < 0:
                assert force.sum() == 0 and depths.min() == 0
            if penetration > 0:
                assert force.sum() > 0.1
                centroid = (points * force[:, None]).sum(0) / force.sum()
                assert (centroid - point).norm() < (0.0005 if args.sweep_all else 0.0008)
                contact_normal = (normals * depths[:, None]).sum(0) / depths.sum()
                contact_normal /= contact_normal.norm()
                projected = -(net * contact_normal).sum().clamp(max=0)
                assert torch.allclose(force.sum(), projected, rtol=1e-5, atol=1e-6)
                # Independently evaluate signed sphere distance in world coordinates.
                current_q = probe.data.root_quat_w[0].cpu().numpy()
                current_rotation = Rotation.from_quat(current_q[[1, 2, 3, 0]])
                offset = [0.004, -0.002, 0.001] if args.transformed_probe else [0.0, 0.0, 0.0]
                current_offset = torch.tensor(current_rotation.apply(offset), device=context.device)
                analytic = (points - probe.data.root_pos_w[0] - current_offset).norm(dim=-1) - 0.003
                contact = depths < 0
                assert torch.max(torch.abs(depths[contact] - analytic[contact])) < 0.00005
                case["analytic_depth_error_m"] = torch.max(torch.abs(depths[contact] - analytic[contact])).item()
                case["centroid_error_m"] = (centroid - point).norm().item()
                case["target_depth_m"] = depths[index].item()
                assert depths[index] < 0
        if not args.sweep_all:
            loaded = [c["sum_force"] for c in report["cases"][-4:] if c["penetration_m"] > 0]
            assert loaded[1] > loaded[0]
    # Verify depth rate against an independent world-space finite difference.
    # The stationary sensor and moving probe include translation and rotation;
    # an offset mesh center moves due to angular velocity as well.
    index = torch.where((pixels == torch.tensor([25, 15], device=pixels.device)).all(-1))[0].item()
    point = points[index]
    normal = normals[index]
    pose = probe.data.default_root_state[:, :7].clone()
    pose[0, :3] = point + normal * 0.0025
    if args.transformed_probe:
        pose[0, 3:] = torch.tensor([np.cos(0.45), 0, 0, np.sin(0.45)], device=context.device)
        rotation = Rotation.from_rotvec([0, 0, 0.9])
        offset = np.array([0.004, -0.002, 0.001])
        pose[0, :3] -= torch.tensor(rotation.apply(offset), device=context.device)
    else:
        rotation = Rotation.identity()
        offset = np.zeros(3)
    velocity = torch.tensor([[0.02, -0.01, 0.015, 0.3, -0.2, 0.4]], device=context.device)
    probe.write_root_pose_to_sim(pose)
    probe.write_root_velocity_to_sim(velocity, env_ids=torch.tensor([0], device=context.device))
    depth, rate = sensor._compute_depths(torch.tensor([0], device=context.device), points.unsqueeze(0))
    body_position = pose[0, :3].cpu().numpy()
    com_offset = probe.root_physx_view.get_coms()[0, :3].cpu().numpy()
    com_world = body_position + rotation.apply(com_offset)
    xyz = points.cpu().numpy()
    epsilon = 1e-5

    def signed_distance(time):
        rot = Rotation.from_rotvec(velocity[0, 3:].cpu().numpy() * time) * rotation
        center = com_world + velocity[0, :3].cpu().numpy() * time + rot.apply(offset - com_offset)
        return np.linalg.norm(xyz - center, axis=1) - 0.003

    expected = (signed_distance(epsilon) - signed_distance(-epsilon)) / (2 * epsilon)
    contacting = depth[0].cpu().numpy() < 0
    error = np.max(np.abs(rate[0].cpu().numpy()[contacting] - expected[contacting]))
    print(
        "MOTION",
        {
            "error": float(error),
            "actual_rate": rate[0].cpu().numpy()[contacting].tolist(),
            "expected_rate": expected[contacting].tolist(),
            "sensor_velocity": sensor._view.get_velocities().cpu().tolist(),
            "probe_velocity": sensor._mesh_view.get_velocities().cpu().tolist(),
            "probe_pose": sensor._mesh_view.get_transforms().cpu().tolist(),
        },
        flush=True,
    )
    # The meshed/cooked SDF gradient is not exactly an ideal sphere gradient.
    # Check the kinematics independently against finite differences of that SDF.
    local_rotation = (
        Rotation.from_euler("xyz", [17, 25, 31], degrees=True) if args.transformed_probe else Rotation.identity()
    )
    raw_scale = 1.25 if args.transformed_probe else 1.0

    def sampled_distance(time):
        body_rot = Rotation.from_rotvec(velocity[0, 3:].cpu().numpy() * time) * rotation
        mesh_rot = body_rot * local_rotation
        center = com_world + velocity[0, :3].cpu().numpy() * time + body_rot.apply(offset - com_offset)
        local = mesh_rot.inv().apply(xyz - center) / raw_scale
        query = torch.zeros(args.num_envs, sensor.num_taxels, 3, device=context.device)
        query[0] = torch.tensor(local, device=context.device, dtype=torch.float32)
        return sensor._sdf_view.get_sdf_and_gradients(query)[0, :, 3].cpu().numpy() * raw_scale

    finite_difference = (sampled_distance(0.001) - sampled_distance(-0.001)) / 0.002
    sdf_error = np.max(np.abs(rate[0].cpu().numpy()[contacting] - finite_difference[contacting]))
    print(
        "SDF MOTION",
        {
            "error": float(sdf_error),
            "rates": finite_difference[contacting].tolist(),
            "stored_rotation": sensor._sdf_rot_l.cpu().tolist(),
            "reference_rotation": local_rotation.as_quat().tolist(),
            "stored_offset": sensor._sdf_pos_l.cpu().tolist(),
        },
        flush=True,
    )
    assert sdf_error < 0.0001
    # Bound the approximation separately rather than treating it as exact hardware.
    assert error < 0.002
    report["motion_ideal_sphere_error_m_s"] = float(error)
    report["motion_sdf_finite_difference_error_m_s"] = float(sdf_error)
    if args.num_envs == 2:
        # Query only the second sensor after PhysX advances, leaving environment 0 untouched.
        sensor.update(0, force_recompute=True)
        before = sensor._data.normal_forces[0].clone()
        position = sensor._data.taxel_points_w[1].clone()
        normal = sensor._data.taxel_normals_w[1].clone()
        pose = probe.data.default_root_state[:, :7].clone()
        pose[0, :3] = torch.tensor([0, 0, 1], device=context.device)
        second_offset = torch.tensor([0.004, -0.002, 0.001], device=context.device) if args.transformed_probe else 0.0
        pose[1, :3] = position[index] + normal[index] * 0.0025 - second_offset
        probe.write_root_pose_to_sim(pose)
        probe.write_root_velocity_to_sim(torch.zeros(2, 6, device=context.device))
        context.step(render=context.has_gui())
        probe.update(dt)
        sensor_body.update(dt)
        sensor._update_buffers_impl(torch.tensor([1], device=context.device))
        assert torch.equal(sensor._data.normal_forces[0], before)
        force = sensor._data.normal_forces[1]
        assert force.sum() > 0.1
        centroid = (sensor._data.taxel_points_w[1] * force[:, None]).sum(0) / force.sum()
        assert (centroid - position[index]).norm() < 0.001
        report["subset_env_contact_force_n"] = force.sum().item()
        report["subset_env_centroid_error_m"] = (centroid - position[index]).norm().item()
    report["status"] = "passed"
    destination = Path(args.report)
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_suffix(destination.suffix + ".tmp")
    temporary.write_text(json.dumps(report, indent=2) + "\n")
    temporary.replace(destination)
    viewer.close()
    print("REPORT", args.report, flush=True)


if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback

        traceback.print_exc()
        raise
    finally:
        # No asynchronous Replicator writers are used by this verification.
        context = sim.SimulationContext.instance()
        if context is not None:
            context._disable_app_control_on_stop_handle = True
        app.close(wait_for_replicator=False)
