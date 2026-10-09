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

"""Independent checks for saved Newton demonstration reports."""

import numpy as np


def check_report(report):
    """Reject incomplete or physically inconsistent reports rather than only inspecting peaks."""
    if report.get("failure"):
        raise ValueError(f"Demo failed: {report['failure']}")
    if report.get("interrupted"):
        raise ValueError("Demo interrupted before completion")
    entries = report["observations"]
    if report.get("backend") != "newton" or not entries:
        raise ValueError("Missing Newton observations")
    if report["taxels_per_sensor"] != [738] * 5:
        raise ValueError("Unexpected taxel count")
    pixels = np.array(report["taxel2pixel"])
    if (
        pixels.shape != (738, 2)
        or not np.issubdtype(pixels.dtype, np.integer)
        or (pixels < 0).any()
        or (pixels >= 32).any()
        or len(np.unique(pixels, axis=0)) != 738
    ):
        raise ValueError("Invalid firmware mapping")
    wall = np.asarray([e["elapsed_wall_s"] for e in entries])
    if not np.isfinite(wall).all() or (wall < 0).any() or (np.diff(wall) <= 0).any():
        raise ValueError("Invalid observed wall-clock timeline")
    steps = np.array([e["step"] for e in entries])
    if not np.array_equal(steps, np.arange(len(entries))):
        raise ValueError("Report must contain every physics tick from zero")
    if report["mode"] not in ("hand", "sensor", "press"):
        raise ValueError("Unknown validation mode")
    if report["mode"] != "press" and report.get("indenter") != "flat":
        raise ValueError("Known-load verification requires the flat indenter; gear runs are demonstrations")
    if report["mode"] == "sensor":
        raise ValueError("Use the all-taxel contact sweep for sensor verification")
    if report["physics_dt_s"] != 1 / 240:
        raise ValueError("Unexpected verification timestep")
    forces = np.array([e["forces_n"] for e in entries])
    if forces.shape != (len(entries), 5) or not np.isfinite(forces).all() or (forces < 0).any():
        raise ValueError("Invalid forces")
    check_states(entries)
    check_frames(report, entries, pixels)
    if report["mode"] == "hand":
        check_load(report, entries)
    if report["mode"] == "press":
        check_press(report, entries, forces)
    return True


def check_frames(report, entries, pixels):
    for e in entries:
        frame = np.array(e["frame_n"])
        taxels = np.array(e["taxel_forces_n"])
        if (
            taxels.shape != (738,)
            or not np.isfinite(taxels).all()
            or (taxels < 0).any()
            or not np.isfinite(frame).all()
        ):
            raise ValueError("Invalid taxel/frame values")
        if frame.shape != (32, 32) or not np.array_equal(frame[pixels[:, 0], pixels[:, 1]], taxels):
            raise ValueError("Firmware mapping mismatch")
        target = e["target_finger"] - 1 if report["mode"] != "sensor" else 0
        if not isinstance(target, int) or not 0 <= target < 5:
            raise ValueError("Invalid target finger")
        if not np.isclose(taxels.sum(), e["forces_n"][target], rtol=1e-5, atol=1e-6):
            raise ValueError("Frame load mismatch")
        projected = max(0, -np.dot(e["normal_force_w_n"], e["penetration_normal_w"]))
        if not np.isclose(projected, taxels.sum(), rtol=1e-5, atol=1e-6):
            raise ValueError("Filtered normal-force mismatch")
        occupied = np.zeros((32, 32), dtype=bool)
        occupied[pixels[:, 0], pixels[:, 1]] = True
        if frame[~occupied].any():
            raise ValueError("Absent firmware cells must be zero")


def check_load(report, entries):
    for finger in range(2, 6):
        windows = {}
        for e in entries:
            if e["target_finger"] == finger and e["step"] % 360 >= 288:
                windows.setdefault(e["step"] // 360, []).append(e)
        if not windows or not any(len(window) >= 30 for window in windows.values()):
            raise ValueError("Missing settling samples")
        for selected in windows.values():
            check_settled_load(report, selected, finger)


def check_settled_load(report, selected, finger):
    """Validate each settling window independently, including repeated finger visits."""
    if (np.diff([e["step"] for e in selected]) != 1).any():
        raise ValueError("Settling observations must be contiguous")
    load = np.array([e["forces_n"][finger - 1] for e in selected])
    if np.max(np.abs(load - 0.981)) >= 0.05 * 0.981:
        raise ValueError("Known load did not settle")
    other = np.delete(np.array([e["forces_n"] for e in selected]), finger - 1, axis=1)
    if other.max() >= 1e-5:
        raise ValueError("Unloaded finger reports force")
    contact = np.array([e["normal_force_w_n"][2] for e in selected])
    velocity = np.array([e["probe_velocity_w"] for e in selected])
    if abs(-contact.mean() - 0.981) >= 0.01 * 0.981:
        raise ValueError("Contact mean load mismatch")
    if np.linalg.norm(velocity[:, :3], axis=1).max() >= 0.01:
        raise ValueError("Load still moving")
    dt = report["physics_dt_s"]
    residual = (-contact[1:] - 0.981) * dt - 0.1 * np.diff(velocity[:, 2])
    if residual.size and np.abs(residual).max() >= 0.01 * 0.981 * dt:
        raise ValueError("Momentum balance mismatch")


def check_press(report, entries, forces):
    if report.get("fixed_base") is not True:
        raise ValueError("Hand wrist is not fixed")
    dt = report["physics_dt_s"]
    times = np.array([e["step"] * dt for e in entries])
    if times[-1] < 7.49:
        raise ValueError("Missing three complete press cycles")
    wrist = np.array([e["wrist_pose"] for e in entries])
    obj = np.array([e["probe_pose"] for e in entries])
    if np.max(np.abs(wrist[:, :3] - wrist[0, :3])) >= 1e-5:
        raise ValueError("Wrist moved")
    if np.min(np.abs(wrist[:, 3:] @ wrist[0, 3:])) <= 1 - 1e-6:
        raise ValueError("Wrist rotated")
    if np.max(np.abs(obj[:, :3] - obj[0, :3])) >= 1e-5:
        raise ValueError("Stationary indenter moved")
    if np.min(np.abs(obj[:, 3:] @ obj[0, 3:])) <= 1 - 1e-6:
        raise ValueError("Stationary indenter rotated")
    joints = np.array([e["joints_rad"] for e in entries])
    ids = report["joint_ids"]
    if len(ids) != 3 or len(set(ids)) != 3 or min(ids) < 0 or max(ids) >= joints.shape[1]:
        raise ValueError("Invalid actuated joint indices")
    stationary = [i for i in range(joints.shape[1]) if i not in ids]
    if np.ptp(joints[:, stationary], axis=0).max() >= 0.005:
        raise ValueError("Uncommanded joints moved")
    if not (np.ptp(joints[:, ids], axis=0) > np.deg2rad(5)).all():
        raise ValueError("No real joint articulation")
    tips = np.array([e["tip_position_m"] for e in entries])
    if np.linalg.norm(tips - tips[0], axis=1).max() <= 0.03:
        raise ValueError("Insufficient fingertip travel")
    for cycle in range(3):
        phase = times - cycle * 2.5
        hold = (phase >= 1.125) & (phase < 1.5)
        release = (phase >= 2.25) & (phase < 2.5)
        if not hold.any() or (forces[hold, 1] <= 0.01).any():
            raise ValueError("Missing sustained contact")
        if not release.any() or forces[release, 1].max() >= 0.01:
            raise ValueError("Failed release")
        if np.abs(joints[release][:, ids] - joints[0, ids]).max() >= 0.01:
            raise ValueError("Joints failed to return")
        if np.linalg.norm(tips[release] - tips[0], axis=1).max() >= 0.001:
            raise ValueError("Fingertip failed to return")


def check_states(entries):
    for e in entries:
        for name, width in (
            ("probe_pose", 7),
            ("probe_velocity_w", 6),
            ("normal_force_w_n", 3),
            ("penetration_normal_w", 3),
            ("tip_position_m", 3),
        ):
            value = np.asarray(e[name])
            if value.shape != (width,) or not np.isfinite(value).all():
                raise ValueError("Invalid state observation")
        if not np.isclose(np.linalg.norm(e["probe_pose"][3:]), 1.0, atol=1e-5):
            raise ValueError("Invalid pose quaternion")
        if "wrist_pose" in e:
            wrist = np.asarray(e["wrist_pose"])
            if (
                wrist.shape != (7,)
                or not np.isfinite(wrist).all()
                or not np.isclose(np.linalg.norm(wrist[3:]), 1.0, atol=1e-5)
            ):
                raise ValueError("Invalid wrist pose")
        if "joints_rad" in e and not np.isfinite(e["joints_rad"]).all():
            raise ValueError("Invalid joint positions")
        if not np.isfinite(e["min_depth_m"]) or e["min_depth_m"] > 0:
            raise ValueError("Invalid penetration depth")
