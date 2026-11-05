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

"""Finger 2 articulates against a fixed gear with the wrist fixed (MuJoCo port of the stationary press).

Each cycle: unloaded, approach to an aligned pose just short of the gear, a force-seeking hold that
drives the measured load to --press-force-n while keeping the planar pad parallel to the gear face,
then retract and release. Only joint position targets move the finger.
"""

import argparse
import json
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np
from dex01_mujoco import TactileSensor, dex01_mount, dex01_pattern
from dex01_mujoco.figures import camera, save_figure
from dex01_mujoco.scenes import gear_body_xml, hand_scene
from dex01_mujoco.viewer import TactileHeatmaps, draw_taxels

parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
parser.add_argument("--headless", action="store_true", help="Run without the viewer and heatmap windows.")
parser.add_argument(
    "--max-steps", type=int, default=0, help="Stop after this many physics steps (0 runs until closed)."
)
parser.add_argument("--no-tactile-ui", action="store_true", help="Hide the heatmap window.")
parser.add_argument("--figure", type=Path, help="Save a still render with the tactile frames to this image.")
parser.add_argument("--figure-time", type=float, default=1.45, help="Simulation time of the --figure still (s).")
parser.add_argument("--press-force-n", type=float, default=1.0, help="Measured normal-load target during the hold.")
parser.add_argument("--press-time-scale", type=float, default=0.25, help="Scale of the 10 s cycle (default 2.5 s).")
parser.add_argument("--verification-report", type=Path, help="Write per-step observations and checks as JSON.")
args = parser.parse_args()
if not 0 < args.press_force_n <= 2:
    parser.error("--press-force-n must be in (0, 2] for this fixture")
if not 0.1 <= args.press_time_scale <= 1:
    parser.error("--press-time-scale must be in [0.1, 1]")

HAND_POS = (0.2, 0.0, 0.05)
HAND_QUAT = (-0.7071067811865476, 0.0, 0.7071067811865475, 0.0)
GEAR_SCALE = 0.15
PRESS_JOINTS = ("rj_dg_2_2", "rj_dg_2_3", "rj_dg_2_4")
# Moderate closure: about 22% of MCP range, 22% of PIP range and 10% of DIP range.
BEND_FRACTION = np.array([0.225, 0.225, 0.10])
APPROACH_CLEARANCE_M = 0.00002
MAX_PRESS_DISTANCE_M = 0.0006


def press_motion(t):
    """Closure fraction and phase over the 10 s cycle; smoothstep keeps target velocity zero at boundaries."""
    t = t % 10.0
    if t < 1.0:
        return 0.0, "unloaded"
    if t < 4.0:
        u = min(1.0, (t - 1.0) / 2.5)
        return u * u * (3 - 2 * u), "approach"
    if t < 6.0:
        return 1.0, "hold"
    if t < 9.0:
        u = (t - 6.0) / 3.0
        return 1 - u * u * (3 - 2 * u), "retract"
    return 0.0, "unloaded"


def frame_from_z(z):
    x = np.cross([0, 1.0, 0], z)
    if np.linalg.norm(x) < 0.1:
        x = np.cross([1.0, 0, 0], z)
    x /= np.linalg.norm(x)
    rotation = np.stack([x, np.cross(z, x), z], axis=1)
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, rotation.ravel())
    return rotation, quat


def main():
    model = hand_scene(
        gear_body_xml("gear", scale=GEAR_SCALE, mass=0.1, pos=(0, 0, 0.5), free=False), HAND_POS, HAND_QUAT
    )
    data = mujoco.MjData(model)
    dt = model.opt.timestep
    mount_pos, mount_quat = dex01_mount()
    sensors = [
        TactileSensor(model, f"rl_dg_{i}_sensor", "gear", dex01_pattern(), mount_pos, mount_quat) for i in range(1, 6)
    ]
    sensor = sensors[1]
    qadr = np.array([model.jnt_qposadr[model.joint(n).id] for n in PRESS_JOINTS])
    dadr = np.array([model.jnt_dofadr[model.joint(n).id] for n in PRESS_JOINTS])
    act = np.array([model.actuator(n).id for n in PRESS_JOINTS])
    limits = np.array([model.joint(n).range for n in PRESS_JOINTS])
    bend = limits[:, 1] * BEND_FRACTION
    body = sensor.sensor_body
    wrist = model.body("rl_dg_mount").id

    # The planar pad: taxels sharing the most common normal (the rounded transition is excluded).
    normals_l = sensor.taxel_normals_l
    shared = normals_l @ normals_l.T > 1 - 1e-6
    face_normal_l = normals_l[np.argmax(shared.sum(1))]
    flat = normals_l @ face_normal_l > 1 - 1e-6

    # Setup-only pose sampling at the endpoint places the gear; the run itself uses joint targets only.
    data.qpos[qadr] = bend
    mujoco.mj_forward(model, data)
    sensor.update(data)
    center = sensor.data.taxel_points_w[flat].mean(0)
    normal = data.xmat[body].reshape(3, 3) @ face_normal_l
    gear_rotation, gear_quat = frame_from_z(normal)
    support = np.array([0, 0, -model.geom("gear_hub").size[1]])  # underside centre in the gear frame
    data.mocap_pos[0] = center - gear_rotation @ support + normal * APPROACH_CLEARANCE_M
    data.mocap_quat[0] = gear_quat
    gear_pose = np.concatenate([data.mocap_pos[0], data.mocap_quat[0]])
    data.qpos[:] = 0
    data.ctrl[:] = 0
    mujoco.mj_forward(model, data)
    wrist_pose = np.concatenate([data.xpos[wrist], data.xquat[wrist]])
    print(f"Endpoint pad centre {np.round(center, 4)} m, normal {np.round(normal, 3)}, {flat.sum()} planar taxels")

    viewer = None if args.headless else mujoco.viewer.launch_passive(model, data)
    heatmaps = TactileHeatmaps(
        sensors, [f"f{i}" for i in range(1, 6)], enabled=viewer is not None and not args.no_tactile_ui
    )
    if viewer is not None:
        viewer.cam.lookat[:] = center
        viewer.cam.distance, viewer.cam.elevation, viewer.cam.azimuth = 0.25, -20, 90

    press_distance, integral, hold_joints = 0.0, np.zeros(3), bend.copy()
    jacp, jacr = np.zeros((3, model.nv)), np.zeros((3, model.nv))
    observations, failure = [], None
    hold_loads, unloaded_loads = [], []
    step, figure_saved = 0, False
    wall_start = time.perf_counter()
    while viewer is None or viewer.is_running():
        t = step * dt / args.press_time_scale
        fraction, phase = press_motion(t)
        if t % 10.0 < 1.0:
            press_distance, integral, hold_joints = 0.0, np.zeros(3), bend.copy()
        targets = fraction * (hold_joints if phase == "retract" else bend)
        if phase == "hold":
            load = float(sensor.data.normal_forces.sum())
            speed = 0.001 if load < 0.01 else 0.003 * (args.press_force_n - load)
            press_distance = min(MAX_PRESS_DISTANCE_M, max(0.0, press_distance + dt * np.clip(speed, -0.001, 0.001)))
            patch_center = sensor.data.taxel_points_w[flat].mean(0)
            patch_normal = sensor.data.taxel_normals_w[flat].mean(0)
            patch_normal /= np.linalg.norm(patch_normal)
            rotation_error = np.cross(patch_normal, normal)
            integral = np.clip(integral + 20 * dt * rotation_error, -0.02, 0.02)
            # Solve translation and orientation together; bending every joint alone tilts the pad.
            mujoco.mj_jac(model, data, jacp, jacr, patch_center, body)
            jacobian = np.vstack([jacp[:, dadr], jacr[:, dadr]])
            error = np.concatenate([center + normal * press_distance - patch_center, 3 * rotation_error + integral])
            correction = np.linalg.lstsq(jacobian, error, rcond=None)[0]
            hold_joints = np.clip(data.qpos[qadr] + np.clip(correction, -0.025, 0.025), limits[:, 0], limits[:, 1])
            targets = hold_joints
        data.ctrl[act] = targets
        mujoco.mj_step(model, data)
        for s in sensors:
            s.update(data)
        step += 1
        if args.figure and not figure_saved and data.time >= args.figure_time:
            print(
                "Saved",
                save_figure(
                    args.figure,
                    model,
                    data,
                    sensors,
                    [f"Finger {i}" for i in range(1, 6)],
                    camera(center, 0.13, -40, 135),
                    f"{phase}, fixed gear",
                    taxel_size=0.00025,
                ),
            )
            figure_saved = True
            if args.headless:
                break

        forces = [float(s.data.normal_forces.sum()) for s in sensors]
        cycle = t % 10.0
        if np.abs(np.concatenate([data.xpos[wrist], data.xquat[wrist]]) - wrist_pose).max() > 1e-6:
            failure = "Wrist moved during finger articulation"
        if np.abs(np.concatenate([data.mocap_pos[0], data.mocap_quat[0]]) - gear_pose).max() > 1e-9:
            failure = "Stationary object moved"
        if phase == "hold" and cycle >= 5.0:
            hold_loads.append(forces[1])
        if phase == "unloaded" and cycle >= 9.5:
            unloaded_loads.append(max(forces))
        if args.verification_report and step % 3 == 0:
            observations.append({
                "step": step,
                "phase": phase,
                "joint_positions_rad": data.qpos[qadr].tolist(),
                "forces_n": forces,
                "press_distance_m": press_distance,
            })
        if step % 12 == 0:
            print(
                f"{phase:9s} joints deg {np.round(np.degrees(data.qpos[qadr]), 1)}  "
                + "  ".join(f"f{i} {f:6.3f}N" for i, f in enumerate(forces, 1))
            )
        if viewer is not None and step % 8 == 0:
            with viewer.lock():
                draw_taxels(viewer.user_scn, [sensor], size=0.00025)
            viewer.sync()
            heatmaps.update()
            lag = data.time - (time.perf_counter() - wall_start)
            if lag > 0:
                time.sleep(lag)
        if failure or (args.max_steps and step >= args.max_steps):
            break
    if viewer is not None:
        viewer.close()
    heatmaps.close()

    summary = {
        "failure": failure,
        "target_force_n": args.press_force_n,
        "late_hold_mean_force_n": float(np.mean(hold_loads)) if hold_loads else None,
        "late_hold_min_force_n": float(np.min(hold_loads)) if hold_loads else None,
        "released_max_force_n": float(np.max(unloaded_loads)) if unloaded_loads else None,
    }
    print("\nSummary:", json.dumps(summary))
    if args.verification_report:
        args.verification_report.write_text(json.dumps({**summary, "dt_s": dt, "observations": observations}) + "\n")
    if failure:
        raise SystemExit(failure)


if __name__ == "__main__":
    main()
