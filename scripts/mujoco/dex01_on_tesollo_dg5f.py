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

"""DEX-01 fingertips on the Tesollo DG-5F hand in MuJoCo (port of dex01_on_tesollo_dg5f.py).

A 0.1 kg gear rests on fingers 2-5 in turn. The loaded finger reports the known 0.981 N weight in
its firmware-mapped 32 x 32 frame; the other fingers read zero.
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
parser.add_argument("--figure-time", type=float, default=1.2, help="Simulation time of the --figure still (s).")
parser.add_argument("--flat-indenter", action="store_true", help="Use a 6 mm cube instead of the gear.")
parser.add_argument("--verification-report", type=Path, help="Write per-step forces and settled-load checks as JSON.")
args = parser.parse_args()

# Demo parameters carried over from the Isaac Lab script.
GRAVITY_M_S2 = 9.81
INDENTER_MASS_KG = 0.1  # mass * gravity = 0.981 N
GEAR_DISPLAY_SCALE = 0.15  # keeps the gear footprint on the active pad
FLAT_INDENTER_HALF_M = 0.003
HAND_POS = (0.2, 0.0, 0.05)
HAND_QUAT = (-0.7071067811865476, 0.0, 0.7071067811865475, 0.0)
SETTLE_CLEARANCE_M = 0.0005
HORIZONTAL_NORMAL_MIN_Z = 0.995
PRESSED_FINGERS = (2, 3, 4, 5)
SECONDS_PER_FINGER = 1.25  # the final 20% is checked as settled load
REPORT_PERIOD_S = 1 / 12


def indenter_xml():
    if args.flat_indenter:
        h = FLAT_INDENTER_HALF_M
        return f"""
    <body name="indenter" pos="0 0 0.3">
      <freejoint name="indenter_free"/>
      <geom name="indenter_hub" type="box" size="{h} {h} {h}" mass="{INDENTER_MASS_KG}" rgba="0.3 0.55 0.85 1"
            contype="1" conaffinity="3"/>
    </body>"""
    return gear_body_xml("indenter", scale=GEAR_DISPLAY_SCALE, mass=INDENTER_MASS_KG, pos=(0, 0, 0.3))


def place_indenter(model, data, sensor):
    """Rest the indenter just above the finger's upward-facing patch, at rest."""
    horizontal = sensor.data.taxel_normals_w[:, 2] > HORIZONTAL_NORMAL_MIN_Z
    if not horizontal.any():
        raise RuntimeError("No upward-facing sensor patch for the gravity-load demonstration")
    centre = sensor.data.taxel_points_w[horizontal].mean(0)
    half_height = model.geom("indenter_hub").size[2 if args.flat_indenter else 1]
    centre[2] = sensor.data.taxel_points_w[:, 2].max() + SETTLE_CLEARANCE_M + half_height
    joint = model.joint("indenter_free")
    qpos, qvel = model.jnt_qposadr[joint.id], model.jnt_dofadr[joint.id]
    data.qpos[qpos : qpos + 7] = (*centre, 1, 0, 0, 0)
    data.qvel[qvel : qvel + 6] = 0


def main():
    model = hand_scene(indenter_xml(), HAND_POS, HAND_QUAT)
    data = mujoco.MjData(model)
    pattern = dex01_pattern()
    mount_pos, mount_quat = dex01_mount()
    sensors = [
        TactileSensor(model, f"rl_dg_{i}_sensor", "indenter", pattern, mount_pos, mount_quat) for i in range(1, 6)
    ]
    print(f"{len(sensors)} DEX-01 sensors x {sensors[0].num_taxels} taxels, frame {pattern.frame_shape}")
    mujoco.mj_forward(model, data)
    for sensor in sensors:
        sensor.update(data)

    dt = model.opt.timestep
    steps_per_finger = round(SECONDS_PER_FINGER / dt)
    report_steps = round(REPORT_PERIOD_S / dt)
    draw_steps = max(1, round(0.04 / dt))
    viewer = None if args.headless else mujoco.viewer.launch_passive(model, data)
    labels = [f"f{i}" for i in range(1, 6)]
    heatmaps = TactileHeatmaps(sensors, labels, enabled=viewer is not None and not args.no_tactile_ui)
    if viewer is not None:
        viewer.cam.lookat[:] = np.mean([data.xpos[sensors[f - 1].sensor_body] for f in PRESSED_FINGERS], axis=0)
        viewer.cam.distance, viewer.cam.elevation, viewer.cam.azimuth = 0.16, -45, 0

    observations, settled = [], {f: [] for f in PRESSED_FINGERS}
    step, figure_saved = 0, False
    wall_start = time.perf_counter()
    while viewer is None or viewer.is_running():
        cycle_step = step % (steps_per_finger * len(PRESSED_FINGERS))
        finger = PRESSED_FINGERS[cycle_step // steps_per_finger]
        if cycle_step % steps_per_finger == 0:
            place_indenter(model, data, sensors[finger - 1])
            if viewer is not None:
                wall_start = time.perf_counter() - data.time

        mujoco.mj_step(model, data)
        for sensor in sensors:
            sensor.update(data)
        step += 1
        if args.figure and not figure_saved and data.time >= args.figure_time:
            lookat = np.mean([data.xpos[s.sensor_body] for s in sensors], axis=0)
            label = "6 mm cube" if args.flat_indenter else "gear"
            print(
                "Saved",
                save_figure(
                    args.figure,
                    model,
                    data,
                    sensors,
                    [f"Finger {i}" for i in range(1, 6)],
                    camera(lookat, 0.3, -35, -30),
                    f"{INDENTER_MASS_KG} kg {label}",
                    taxel_size=0.00025,
                ),
            )
            figure_saved = True
            if args.headless:
                break

        totals = [float(s.data.normal_forces.sum()) for s in sensors]
        if cycle_step % steps_per_finger >= 0.8 * steps_per_finger:
            settled[finger].append(totals)
        if args.verification_report:
            observations.append({"step": step, "target_finger": finger, "forces_n": totals})
        if step % report_steps == 0:
            print("  ".join(f"f{i} {t:7.3f}N" for i, t in enumerate(totals, 1)))
        if viewer is not None and step % draw_steps == 0:
            with viewer.lock():
                draw_taxels(viewer.user_scn, sensors, size=0.00025)
            viewer.sync()
            heatmaps.update()
            lag = data.time - (time.perf_counter() - wall_start)
            if lag > 0:
                time.sleep(lag)
        if args.max_steps and step >= args.max_steps:
            break
    if viewer is not None:
        viewer.close()
    heatmaps.close()

    expected = INDENTER_MASS_KG * GRAVITY_M_S2
    summary = {}
    for finger, rows in settled.items():
        if not rows:
            continue
        rows = np.asarray(rows)
        others = np.delete(rows, finger - 1, axis=1)
        summary[f"f{finger}"] = {
            "mean_load_n": float(rows[:, finger - 1].mean()),
            "error_pct": float(100 * (rows[:, finger - 1].mean() - expected) / expected),
            "max_other_finger_n": float(np.abs(others).max()),
        }
    print(f"\nSettled load (expected {expected:.3f} N):")
    for name, s in summary.items():
        print(
            f"  {name}: {s['mean_load_n']:.4f} N ({s['error_pct']:+.2f}%), other fingers max"
            f" {s['max_other_finger_n']:.4f} N"
        )
    if args.verification_report:
        args.verification_report.write_text(
            json.dumps(
                {
                    "dt_s": dt,
                    "indenter": "flat" if args.flat_indenter else "gear",
                    "settled": summary,
                    "observations": observations,
                },
                indent=1,
            )
            + "\n"
        )


if __name__ == "__main__":
    main()
