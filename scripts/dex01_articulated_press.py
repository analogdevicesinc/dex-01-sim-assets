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

"""Fixed-wrist fingertip pressing through real finger-joint actuators."""

import hashlib
import json
import math
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time
from contextlib import contextmanager, suppress
from pathlib import Path


def run_articulated_press(
    sim, scene, sensors, viewer, args, app, prepare_view, camera_setup, set_camera, flush_view, local_viewer=None
):
    import numpy as np
    import omni.usd
    import torch
    from isaaclab.utils.math import quat_apply, quat_from_matrix, quat_inv
    from object_geometry import (
        collision_triangles,
        lower_face_center,
        projected_face_mask,
    )

    robot, gear, sensor = scene["robot"], scene["asset"], sensors[1]
    dt = sim.get_physics_dt()
    ids, names = robot.find_joints(["rj_dg_2_2", "rj_dg_2_3", "rj_dg_2_4"], preserve_order=True)
    if not robot.is_fixed_base or "press_slide" in robot.joint_names:
        raise RuntimeError("Articulated demo requires the original fixed wrist and no sliding stage")
    # Setup-only pose sampling measures the endpoint and camera bounds. Runtime
    # motion below uses position targets exclusively, never joint-state writes.
    rest = robot.data.default_joint_pos.clone()
    # Moderate closure: about 22% of MCP range, 22% of PIP range and
    # 10% of DIP range, leaving ample actuation headroom for pressing.
    limits = robot.data.joint_pos_limits[0, ids]
    bend = limits[:, 1] * torch.tensor([0.225, 0.225, 0.10], device=sim.device)
    pose = gear.data.default_root_state[:, :7].clone()
    pose[:, :3] = torch.tensor([0, 0, 2], device=sim.device)
    gear.write_root_pose_to_sim(pose)
    scene.write_data_to_sim()
    sim.step(render=False)
    scene.update(dt)
    wrist = robot.data.body_state_w[0, robot.body_names.index("rl_dg_mount"), :7].clone()
    initial_points = sensor.data.taxel_points_w[0].clone()
    target_joints = rest.clone()
    target_joints[:, ids] = bend
    robot.write_joint_state_to_sim(target_joints, torch.zeros_like(target_joints))
    robot.set_joint_position_target(target_joints)
    sim.step(render=False)
    scene.update(dt)
    sampled_before_settle = sensor.data.taxel_points_w[0].clone()
    settle_joint_pose(sim, scene, robot, sensor, target_joints)
    print(
        "[PRESS]: endpoint settling displacement mm",
        float((sensor.data.taxel_points_w[0] - sampled_before_settle).norm(dim=-1).max() * 1000),
        flush=True,
    )
    points = sensor.data.taxel_points_w[0].clone()
    normals = sensor.data.taxel_normals_w[0].clone()
    # Choose the planar pad samples using their shared normal, not all curved faces.
    local_normal = torch.tensor([0, 0, 1.0], device=sim.device)
    body_id = robot.body_names.index("rl_dg_2_sensor")
    face_normal = quat_apply(robot.data.body_quat_w[0, body_id], local_normal)
    flat = planar_patch(points, normals, face_normal)
    center = points[flat].mean(0)
    normal = face_normal / face_normal.norm()
    # Gear axis follows the endpoint pad normal. The gear is positioned once
    # during setup and remains kinematic and stationary through every cycle.
    stage = omni.usd.get_context().get_stage()
    support = torch.tensor(
        lower_face_center(stage, gear.root_physx_view.prim_paths[0]), device=sim.device, dtype=torch.float32
    )
    z = normal
    x = torch.cross(torch.tensor([0, 1.0, 0], device=sim.device), z, dim=0)
    if x.norm() < 0.1:
        x = torch.cross(torch.tensor([1.0, 0, 0], device=sim.device), z, dim=0)
    x /= x.norm()
    y = torch.cross(z, x, dim=0)
    pose[:, 3:] = quat_from_matrix(torch.stack([x, y, z], dim=1)).unsqueeze(0)
    # Approach a small clearance, then seek contact during hold. A prescribed
    # overlap loads the entire face before the force threshold can act.
    pose[:, :3] = center - quat_apply(pose[:, 3:], support.unsqueeze(0)) + normal * 0.00002
    triangles = collision_triangles(stage, gear.root_physx_view.prim_paths[0])
    underside = triangles[np.isclose(triangles[:, :, 2], float(support[2]), rtol=0, atol=1e-8).all(1)]
    if not len(underside):
        raise RuntimeError("The gear has no planar underside")
    projected = quat_apply(quat_inv(pose[0, 3:]).expand(len(points), 4), points - pose[0, :3]).cpu().numpy()
    footprint = projected_face_mask(projected[:, :2], underside[:, :, :2]) & flat.cpu().numpy()
    footprint_ids = torch.tensor(np.flatnonzero(footprint), device=sim.device)
    if len(footprint_ids) < 12:
        raise RuntimeError("Too few planar taxels under the gear face")
    quadrants = (projected[footprint, 0] >= float(support[0])).astype(int) + 2 * (
        projected[footprint, 1] >= float(support[1])
    )
    quadrant_ids = torch.tensor(quadrants, device=sim.device)
    bottom_vertices = torch.tensor(np.unique(underside.reshape(-1, 3), axis=0), device=sim.device, dtype=torch.float32)
    bottom_world = quat_apply(pose[0, 3:].expand(len(bottom_vertices), 4), bottom_vertices) + pose[0, :3]
    sensor_inverse = quat_inv(robot.data.body_quat_w[0, body_id])
    patch_local = quat_apply(sensor_inverse.expand(int(flat.sum()), 4), points[flat] - center)
    face_local = quat_apply(sensor_inverse.expand(len(bottom_world), 4), bottom_world - center)
    if not bool(
        (
            (face_local[:, :2] >= patch_local[:, :2].min(0).values - 1e-6)
            & (face_local[:, :2] <= patch_local[:, :2].max(0).values + 1e-6)
        ).all()
    ):
        raise RuntimeError("The gear underside extends beyond the planar sampling patch")
    gear.write_root_pose_to_sim(pose)
    gear.write_root_velocity_to_sim(torch.zeros(1, 6, device=sim.device))
    robot.write_joint_state_to_sim(rest, torch.zeros_like(rest))
    robot.set_joint_position_target(rest)
    scene.reset()
    settle_joint_pose(sim, scene, robot, sensor, rest)
    prepare_view(sim)
    viewport, camera_path, camera = camera_setup(stage, sim)
    # Camera is fixed and faces the flexion plane. Include wrist and both tips.
    anchor = wrist[:3].cpu().numpy()
    tip0 = initial_points.mean(0).cpu().numpy()
    tip1 = center.cpu().numpy()
    extent = max(np.linalg.norm(tip0 - anchor), np.linalg.norm(tip1 - anchor))
    target = (anchor + tip0 + tip1) / 3 - np.array([0, 0, extent * 0.15])
    eye = target + np.array([0.1, -1.3, 0.65]) * extent * 2
    plan = {"eye_m": eye.tolist(), "target_m": target.tolist()}
    set_camera(sim, stage, camera_path, camera, plan)
    flush_view(sim)
    observations = []
    started = time.perf_counter()
    press_distance = 0.0
    orientation_integral = torch.zeros(3, device=sim.device)
    hold_joints = bend.clone()
    failure = None
    step = 0

    with graceful_stop() as stop:
        while app.is_running() and not stop.is_set():
            cycle = cycle_time(step * dt / args.press_time_scale)
            fraction, phase = press_motion(step * dt / args.press_time_scale)
            if cycle < 1:
                press_distance = 0.0
                orientation_integral.zero_()
                hold_joints = bend.clone()
            targets = rest.clone()
            targets[:, ids] = fraction * (hold_joints if phase == "retract" else bend)
            if phase == "hold":
                load = float(sensor.data.normal_forces[0].sum())
                speed = 0.001 if load < 0.01 else 0.003 * (args.press_force_n - load)
                press_distance = min(0.0006, max(0.0, press_distance + dt * max(-0.001, min(0.001, speed))))
                patch_center = sensor.data.taxel_points_w[0, flat].mean(0)
                patch_normal = sensor.data.taxel_normals_w[0, flat].mean(0)
                patch_normal /= patch_normal.norm()
                rotation_error = torch.cross(patch_normal, normal, dim=0)
                orientation_integral = (orientation_integral + 20 * dt * rotation_error).clamp(-0.02, 0.02)
                # Solve translation and orientation together. Increasing all
                # flexion joints rotates the pad and produces edge contact.
                jacobian = robot.root_physx_view.get_jacobians()[0, body_id - 1][:, ids]
                offset = patch_center - robot.data.body_pos_w[0, body_id]
                correction = cartesian_joint_correction(
                    jacobian,
                    offset,
                    center + normal * press_distance - patch_center,
                    3 * rotation_error + orientation_integral,
                )
                hold_joints = (robot.data.joint_pos[0, ids] + correction.clamp(-0.025, 0.025)).clamp(
                    limits[:, 0], limits[:, 1]
                )
                targets[:, ids] = hold_joints
            robot.set_joint_position_target(targets)
            scene.write_data_to_sim()
            sim.step(render=sim.has_gui() and step % 4 == 0)
            scene.update(dt)
            forces = torch.stack([s.data.normal_forces[0].sum() for s in sensors]).cpu().tolist()
            actual = robot.data.joint_pos[0, ids]
            quality = face_contact_quality(sensor, flat, footprint_ids, quadrant_ids, normal, bottom_world)
            wrist_now = robot.data.body_state_w[0, robot.body_names.index("rl_dg_mount"), :7]
            if (wrist_now - wrist).abs().max() > 1e-5:
                failure = "Wrist moved during finger articulation"
            if (gear.data.root_state_w[0, :7] - pose[0]).abs().max() > 1e-5:
                failure = "Stationary object moved"
            if phase == "hold" and cycle > 5.5 and forces[1] <= 0.01:
                failure = "Articulated fingertip did not establish sustained measured contact"
            if (
                phase == "hold"
                and cycle >= 4.5
                and (
                    quality["face_angle_deg"] > 0.05
                    or quality["face_coverage"] < 0.9
                    or quality["face_max_gap_m"] > 2e-6
                    or min(quality["face_quadrant_forces_n"]) < 0.05 * forces[1]
                )
            ):
                failure = f"Incomplete planar face contact: {quality}"
            if step % 12 == 0:
                print(phase, "joints deg", torch.rad2deg(actual).cpu().tolist(), "forces N", forces, flush=True)
                viewer.update(
                    f"{phase} | wrist fixed | finger 2 joints "
                    + "/".join(f"{v:.0f}°" for v in torch.rad2deg(actual).cpu().tolist())
                )
                update_local_viewer(sensors, local_viewer)
            if args.verification_report and step % 3 == 0:
                depths = sensor.data.depths[0]
                weighted = (sensor.data.taxel_normals_w[0] * depths[:, None]).sum(0) / depths.sum().clamp(max=-1e-8)
                weighted /= weighted.norm().clamp(min=1e-8)
                observations.append({
                    "step": step,
                    "phase": phase,
                    "elapsed_wall_s": time.perf_counter() - started,
                    "joint_targets_rad": targets[0, ids].cpu().tolist(),
                    "press_distance_m": press_distance,
                    "joint_positions_rad": actual.cpu().tolist(),
                    "all_joint_positions_rad": robot.data.joint_pos[0].cpu().tolist(),
                    "wrist_pose": wrist_now.cpu().tolist(),
                    "object_pose": gear.data.root_state_w[0, :7].cpu().tolist(),
                    "tip_position_m": sensor.data.taxel_points_w[0].mean(0).cpu().tolist(),
                    "forces_n": forces,
                    "min_depth_m": depths.min().item(),
                    "contact_force_w_n": sensor._contact_view.get_contact_force_matrix(dt=dt)[0, 0].cpu().tolist(),
                    "penetration_normal_w": weighted.cpu().tolist(),
                    "frame_n": sensor.get_tactile_image()[0].cpu().tolist(),
                    "taxel_forces_n": sensor.data.normal_forces[0].cpu().tolist(),
                    **quality,
                })
            step += 1
            if failure or (args.max_steps and step >= args.max_steps):
                break
        if args.verification_report:
            Path(args.verification_report).write_text(
                json.dumps(
                    {
                        "mode": "fixed-wrist-planar-press",
                        "failure": failure,
                        "demo_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                        "sensor_code_sha256": hashlib.sha256(
                            Path(sys.modules[type(sensor).__module__].__file__).read_bytes()
                        ).hexdigest(),
                        "joint_names": names,
                        "all_joint_names": robot.joint_names,
                        "actuated_joint_indices": ids,
                        "endpoint_joint_targets_rad": bend.cpu().tolist(),
                        "endpoint_tip_center_m": center.cpu().tolist(),
                        "initial_tip_center_m": initial_points[flat].mean(0).cpu().tolist(),
                        "fixed_base": robot.is_fixed_base,
                        "physics_dt_s": dt,
                        "trajectory_time_scale": args.press_time_scale,
                        "check_wall_clock": bool(sim.has_gui()),
                        "observation_stride": 3,
                        "taxel2pixel": sensor.taxel2pixel.cpu().tolist(),
                        "face_taxel_indices": footprint_ids.cpu().tolist(),
                        "face_taxel_quadrants": quadrants.tolist(),
                        "observations": observations,
                    },
                    indent=2,
                )
                + "\n"
            )
        if failure:
            raise RuntimeError(failure)


def planar_patch(points, normals, face_normal):
    """Select truly parallel, coplanar taxels, excluding the rounded transition."""
    face_normal = face_normal / face_normal.norm()
    alignment = (normals @ face_normal) / normals.norm(dim=-1)
    mask = alignment > 1 - 1e-6
    if int(mask.sum()) < 10:
        raise RuntimeError("No planar sampling patch at articulated endpoint")
    heights = points[mask] @ face_normal
    if float(heights.max() - heights.min()) > 1e-6:
        raise RuntimeError("Selected contact samples are not coplanar")
    return mask


def cartesian_joint_correction(jacobian, offset, position_error, rotation_error):
    """Move the patch center while correcting its orientation through real joint targets."""
    import torch

    jacobian = jacobian.clone()
    jacobian[:3] += torch.cross(jacobian[3:].T, offset.expand(jacobian.shape[1], 3), dim=1).T
    return torch.linalg.lstsq(jacobian, torch.cat([position_error, rotation_error])).solution


def face_contact_quality(sensor, flat, footprint_ids, quadrant_ids, normal, bottom_world):
    """Measure face alignment and coverage independently of the total-force threshold."""
    import torch

    current_normal = sensor.data.taxel_normals_w[0, flat].mean(0)
    current_normal /= current_normal.norm()
    current_center = sensor.data.taxel_points_w[0, flat].mean(0)
    forces = sensor.data.normal_forces[0, footprint_ids]
    return {
        "face_normal_w": current_normal.cpu().tolist(),
        "face_angle_deg": float(
            torch.rad2deg(torch.atan2(torch.cross(current_normal, normal, dim=0).norm(), current_normal @ normal))
        ),
        "face_max_gap_m": float(((bottom_world - current_center) @ current_normal).max()),
        "face_coverage": float((forces > 1e-8).float().mean()),
        "face_quadrant_forces_n": [float(forces[quadrant_ids == q].sum()) for q in range(4)],
    }


def press_motion(time_s):
    """Joint target envelope: unloaded, approach, hold, retract, unloaded.

    Return a normalized closure fraction and phase. Smoothstep keeps
    joint target velocity zero at phase boundaries.
    """
    t = cycle_time(time_s)
    if t < 1.0:
        return 0.0, "unloaded"
    if t < 4.0:
        # Reach the aligned, unloaded pose before the force-seeking phase so
        # residual joint lag does not turn first contact into an edge press.
        u = min(1.0, (t - 1.0) / 2.5)
        return u * u * (3 - 2 * u), "approach"
    if t < 6.0:
        return 1.0, "hold"
    if t < 9.0:
        u = (t - 6.0) / 3.0
        return 1 - u * u * (3 - 2 * u), "retract"
    return 0.0, "unloaded"


def settle_joint_pose(sim, scene, robot, sensor, targets):
    """Wait for the sensor's fixed-joint body after setup-only articulation teleports."""
    previous = sensor.data.taxel_points_w.clone()
    stable = 0
    for _ in range(120):
        scene.write_data_to_sim()
        sim.step(render=False)
        scene.update(sim.get_physics_dt())
        points = sensor.data.taxel_points_w
        moved = float((points - previous).norm(dim=-1).max())
        joint_error = float((robot.data.joint_pos - targets).abs().max())
        stable = stable + 1 if moved < 1e-6 and joint_error < 1e-4 else 0
        if stable >= 3:
            return
        previous = points.clone()
    raise RuntimeError("The fingertip did not settle at the setup joint pose")


def cycle_time(time_s):
    """Normalize only roundoff at trajectory phase boundaries (in seconds)."""
    t = time_s % 10.0
    for boundary in (0.0, 1.0, 4.0, 4.5, 5.5, 6.0, 9.0, 9.5, 10.0):
        if math.isclose(t, boundary, rel_tol=0, abs_tol=1e-10):
            return boundary % 10.0
    return t


def _require(condition, message):
    if not condition:
        raise ValueError(message)


def check_articulated_report(report):
    """Audit the original observations using one timeline, without rebasing."""
    import numpy as np

    _require(
        report.get("mode") in {"fixed-wrist-articulated-press", "fixed-wrist-planar-press"},
        "Expected fixed-wrist articulation report",
    )
    _require(not report.get("failure"), f"Demo failed: {report.get('failure')}")
    entries = report["observations"]
    _require(len(entries) > 10, "Missing observations")
    dt, scale = report["physics_dt_s"], report["trajectory_time_scale"]
    _require(np.isfinite(dt) and dt > 0 and np.isfinite(scale) and scale > 0, "Invalid trajectory timing")
    steps = np.asarray([e["step"] for e in entries], dtype=float)
    _require(
        np.isfinite(steps).all() and (steps >= 0).all() and (steps == np.floor(steps)).all(),
        "Invalid observation steps",
    )
    stride = report.get("observation_stride", 1)
    _require(isinstance(stride, int) and stride > 0, "Invalid observation stride")
    _require(steps[0] == 0 and (np.diff(steps) == stride).all(), "Missing or unordered observations")
    times = steps * dt / scale
    phases = np.asarray([cycle_time(t) for t in times])
    # Decide cycle membership once. Exact cycle boundaries belong to the new
    # cycle even when floating point division lands infinitesimally below it.
    cycles = np.floor(times / 10).astype(int)
    near_end = (phases == 0) & (times > 0)
    cycles[near_end] = np.rint(times[near_end] / 10).astype(int)
    data = _report_arrays(report, entries)
    _check_report_invariants(report, data)
    wall = data["elapsed_wall_s"]
    _require((wall >= 0).all() and (np.diff(wall) > 0).all(), "Invalid observed wall-clock timing")
    loads = []
    check_wall_clock = report.get("check_wall_clock", True)
    _require(isinstance(check_wall_clock, bool), "Invalid wall-clock check setting")
    for cycle in np.unique(cycles):
        indices = np.flatnonzero(cycles == cycle)
        if phases[indices[-1]] < 9.5:
            continue
        boundary_wall = 0.0 if cycle == 0 else np.interp(cycle * 10, times, wall)
        loads.append(_check_articulated_cycle(data, indices, phases[indices], boundary_wall, check_wall_clock))
    _require(loads, "Missing full release cycle")
    return float(np.median(loads))


def _report_arrays(report, entries):
    """Check numeric data before any cycle filtering can hide a bad sample."""
    import numpy as np

    fields = {
        "wrist_pose": (7,),
        "object_pose": (7,),
        "joint_positions_rad": (3,),
        "tip_position_m": (3,),
        "forces_n": (5,),
        "min_depth_m": (),
        "contact_force_w_n": (3,),
        "penetration_normal_w": (3,),
        "elapsed_wall_s": (),
    }
    if report["mode"] == "fixed-wrist-planar-press":
        fields.update(
            face_normal_w=(3,), face_angle_deg=(), face_max_gap_m=(), face_coverage=(), face_quadrant_forces_n=(4,)
        )
    data = {}
    for field, shape in fields.items():
        _require(all(field in entry for entry in entries), f"Missing {field}")
        values = np.asarray([entry[field] for entry in entries], dtype=float)
        _require(values.shape == (len(entries), *shape) and np.isfinite(values).all(), f"Invalid {field}")
        data[field] = values
    for field in ("frame_n", "taxel_forces_n", "all_joint_positions_rad"):
        _require(all(field in entry for entry in entries), f"Missing {field}")
        values = np.asarray([entry[field] for entry in entries], dtype=float)
        _require(np.isfinite(values).all(), f"Invalid {field}")
        data[field] = values
    frame, taxels = data["frame_n"], data["taxel_forces_n"]
    _require(frame.ndim == 3 and taxels.ndim == 2, "Invalid firmware array dimensions")
    pixels = np.asarray(report["taxel2pixel"])
    _require(pixels.shape == (taxels.shape[1], 2) and np.issubdtype(pixels.dtype, np.integer), "Invalid taxel mapping")
    _require(
        (pixels >= 0).all() and (pixels[:, 0] < frame.shape[1]).all() and (pixels[:, 1] < frame.shape[2]).all(),
        "Taxel mapping is outside the frame",
    )
    _require(len(np.unique(pixels, axis=0)) == len(pixels), "Duplicate taxel mapping")
    _require((data["forces_n"] >= 0).all() and (frame >= 0).all() and (taxels >= 0).all(), "Invalid forces")
    _require(np.max(np.delete(data["forces_n"], 1, axis=1)) < 0.01, "Unexpected contact on an uncommanded finger")
    _require(np.array_equal(frame[:, pixels[:, 0], pixels[:, 1]], taxels), "Firmware mapping mismatch")
    mask = np.zeros(frame.shape[1:], dtype=bool)
    mask[pixels[:, 0], pixels[:, 1]] = True
    _require((frame[:, ~mask] == 0).all(), "Absent firmware cells contain force")
    _require(np.allclose(frame.sum((1, 2)), data["forces_n"][:, 1], atol=1e-5), "Frame load mismatch")
    _require((data["min_depth_m"] <= 0).all(), "Invalid positive penetration depth")
    loaded = data["forces_n"][:, 1] > 0.01
    _require(
        np.allclose(np.linalg.norm(data["penetration_normal_w"][loaded], axis=1), 1, rtol=0, atol=1e-5),
        "Invalid loaded surface normal",
    )
    projected = np.maximum(0, -(data["contact_force_w_n"] * data["penetration_normal_w"]).sum(1))
    _require(
        np.allclose(projected, data["forces_n"][:, 1], rtol=1e-4, atol=1e-5),
        "Contact force disagrees with tactile normal load",
    )
    if report["mode"] == "fixed-wrist-planar-press":
        _check_face_measurements(report, data)
    return data


def _check_face_measurements(report, data):
    import numpy as np

    indices = np.asarray(report["face_taxel_indices"])
    quadrants = np.asarray(report["face_taxel_quadrants"])
    _require(
        indices.ndim == 1
        and len(indices) >= 4
        and np.issubdtype(indices.dtype, np.integer)
        and len(np.unique(indices)) == len(indices)
        and (indices >= 0).all()
        and (indices < data["taxel_forces_n"].shape[1]).all(),
        "Invalid face taxel indices",
    )
    _require(
        quadrants.shape == indices.shape
        and np.issubdtype(quadrants.dtype, np.integer)
        and set(quadrants) == {0, 1, 2, 3},
        "Invalid face quadrants",
    )
    force = data["taxel_forces_n"][:, indices]
    _require(
        np.allclose((force > 1e-8).mean(1), data["face_coverage"], atol=1e-6), "Face coverage disagrees with taxels"
    )
    totals = np.stack([force[:, quadrants == q].sum(1) for q in range(4)], axis=1)
    _require(np.allclose(totals, data["face_quadrant_forces_n"], atol=1e-6), "Face quadrant loads disagree with taxels")
    normals = data["face_normal_w"]
    _require(np.allclose(np.linalg.norm(normals, axis=1), 1, atol=1e-5), "Invalid face normal")
    w, x, y, z = data["object_pose"][:, 3:].T
    gear_normal = np.stack([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)], axis=1)
    angle = np.rad2deg(
        np.arctan2(np.linalg.norm(np.cross(normals, gear_normal), axis=1), (normals * gear_normal).sum(1))
    )
    _require(np.allclose(angle, data["face_angle_deg"], atol=2e-4), "Face angle disagrees with object pose")


def _check_report_invariants(report, data):
    """Use the first run observation as the baseline for every cycle."""
    import numpy as np

    for field, name in (("wrist_pose", "Wrist"), ("object_pose", "Object")):
        poses = data[field]
        _require(np.max(np.abs(poses[:, :3] - poses[0, :3])) < 1e-5, f"{name} translated")
        quats = poses[:, 3:]
        _require(np.allclose(np.linalg.norm(quats, axis=1), 1, rtol=0, atol=1e-5), f"Invalid {name.lower()} quaternion")
        _require(np.min(np.abs(quats @ quats[0])) > 1 - 1e-6, f"{name} rotated")
    for field in ("all_joint_names", "actuated_joint_indices", "fixed_base"):
        _require(field in report, f"Missing {field}")
    joints = data["all_joint_positions_rad"]
    _require(joints.ndim == 2 and joints.shape[1] == len(report["all_joint_names"]), "Invalid all-joint dimensions")
    moving = report["actuated_joint_indices"]
    _require(
        len(moving) == 3
        and len(set(moving)) == 3
        and all(isinstance(i, int) and 0 <= i < joints.shape[1] for i in moving),
        "Invalid actuated joint indices",
    )
    stationary = [i for i in range(joints.shape[1]) if i not in moving]
    if stationary:
        _require(np.max(np.ptp(joints[:, stationary], axis=0)) < 0.005, "Uncommanded finger joints moved")
    _require(np.array_equal(joints[:, moving], data["joint_positions_rad"]), "Joint observation mismatch")
    _require(
        report["fixed_base"] is True and "press_slide" not in report["all_joint_names"],
        "Hand is not the original fixed-base articulation",
    )


def _check_articulated_cycle(data, indices, phase_times, boundary_wall, check_wall_clock=True):
    """Check a completed cycle using original indices and normalized phases."""
    import numpy as np

    joints, tips, forces = (data[field][indices] for field in ("joint_positions_rad", "tip_position_m", "forces_n"))
    initial = phase_times < 0.8
    _require(initial.any() and (forces[initial, 1] < 0.01).all(), "Fingertip starts loaded")
    _require((np.ptp(joints, axis=0) > np.deg2rad(5)).all(), "Finger joints did not articulate")
    travel = np.linalg.norm(tips - tips[0], axis=1)
    _require(travel.max() > 0.03, "Fingertip did not move visibly")
    hold = (phase_times >= 4.5) & (phase_times < 6)
    _require(hold.any() and (forces[hold, 1] > 0.01).all(), "No sustained fingertip load")
    _require((data["min_depth_m"][indices][hold] < 0).all(), "No loaded taxel penetration")
    if "face_coverage" in data:
        _require((data["face_coverage"][indices][hold] >= 0.9).all(), "Incomplete gear-face coverage")
        _require((data["face_angle_deg"][indices][hold] <= 0.05).all(), "Gear and sensor faces are not parallel")
        _require((data["face_max_gap_m"][indices][hold] <= 2e-6).all(), "An edge of the gear face remains separated")
        _require(
            (data["face_quadrant_forces_n"][indices][hold] >= 0.05 * forces[hold, 1, None]).all(),
            "Contact does not span all four sides of the gear face",
        )
    released = phase_times >= 9
    _require(released.any() and forces[released, 1].max() < 0.01, "Fingertip failed to release")
    _require(np.max(np.abs(joints[released] - joints[0])) < 0.01, "Joints failed to return")
    _require(travel[released].max() < 0.001, "Fingertip failed to return")
    contacts = np.flatnonzero(forces[:, 1] > 0.01)
    _require(len(contacts) > 0, "No measured fingertip contact")
    if check_wall_clock:
        wall = data["elapsed_wall_s"][indices] - boundary_wall
        _require(wall[-1] < 12, "Cycle exceeds twelve seconds")
        first_motion = np.flatnonzero(travel > 0.002)[0]
        _require(wall[first_motion] < 2, "Motion begins too late")
        _require(wall[contacts[0]] < 5, "Contact occurs too late")
        _require(wall[hold][-1] - wall[hold][0] >= 1, "Hold is too brief")
    return float(np.median(forces[hold, 1]))


def update_local_viewer(sensors, local_viewer):
    """Refresh the optional desktop plot from the same firmware frames."""
    if local_viewer is not None:
        local_viewer.update([sensor.get_tactile_image()[0].cpu().numpy() for sensor in sensors])


def run_subprocess_with_report(script, arguments, report_path):
    """Reject stale or missing reports even when the simulator exits with code zero."""
    destination = Path(report_path).resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.unlink(missing_ok=True)
    with tempfile.TemporaryDirectory(prefix="dex01-demo-check-") as directory:
        report = Path(directory) / "report.json"
        result = subprocess.run(
            [
                sys.executable,
                *subprocess._args_from_interpreter_flags(),
                str(script),
                *arguments,
                "--verification-worker",
                "--verification-report",
                str(report),
            ],
            check=False,
        )
        data = json.loads(report.read_text()) if report.exists() else {}
        if result.returncode or not data.get("observations") or data.get("failure"):
            data["failure"] = (
                data.get("failure") or f"Verification worker failed or produced no report (exit {result.returncode})"
            )
            destination.write_text(json.dumps(data, indent=2) + "\n")
            raise RuntimeError(data["failure"])
        shutil.copy2(report, destination)


def run_with_report(main, report_path=None, mode="fixed-wrist-articulated-press"):
    """Invalidate a previous run's report and preserve current-run diagnostics."""
    destination = Path(report_path) if report_path else None
    if destination is not None:
        destination.unlink(missing_ok=True)
        destination.parent.mkdir(parents=True, exist_ok=True)
    try:
        main()
    except (Exception, KeyboardInterrupt, SystemExit) as exc:
        # argparse's successful help exit is handled outside this lifecycle.
        # Other exits must never leave current-run success marked valid.
        if isinstance(exc, SystemExit) and exc.code in (None, 0):
            raise
        if destination is not None:
            current = None
            if destination.exists():
                with suppress(ValueError, OSError):
                    current = json.loads(destination.read_text())
            if not isinstance(current, dict):
                current = {"mode": mode, "observations": []}
            # Preserve diagnostics from a failure already recorded by this run.
            # A later exception (including app shutdown) invalidates success.
            if not current.get("failure"):
                current["failure"] = str(exc) or type(exc).__name__
                destination.write_text(json.dumps(current, indent=2) + "\n")
        raise


@contextmanager
def graceful_stop():
    """Let Ctrl+C finish the current physics step and persist observations.

    Isaac Sim's default SIGINT handler unloads plugins and exits immediately.
    Keep the simulator alive until this runner has written its requested report.
    """
    stop = threading.Event()
    previous = signal.getsignal(signal.SIGINT)
    signal.signal(signal.SIGINT, lambda signum, frame: stop.set())
    try:
        yield stop
    finally:
        signal.signal(signal.SIGINT, previous)
