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

"""Flat 32 x 32 tactile pad with a 0.5 kg gear dropped onto it (MuJoCo port of simple_example.py).

The settled taxel forces sum to the gear's weight, 0.5 kg x 9.81 m/s^2 = 4.905 N.
"""

import argparse
import time

import mujoco
import mujoco.viewer
from dex01_mujoco import TactileSensor, grid_pattern
from dex01_mujoco.scenes import ENVIRONMENT_XML, OPTION_XML, gear_body_xml
from dex01_mujoco.viewer import TactileHeatmaps, draw_taxels

parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
parser.add_argument("--headless", action="store_true", help="Run without the viewer and heatmap windows.")
parser.add_argument(
    "--max-steps", type=int, default=0, help="Stop after this many physics steps (0 runs until closed)."
)
parser.add_argument("--no-tactile-ui", action="store_true", help="Hide the heatmap window.")
args = parser.parse_args()

SENSOR_SIZE = (0.2, 0.1, 0.01)
GRID = (32, 32)
RESOLUTION = (SENSOR_SIZE[0] / GRID[0], SENSOR_SIZE[1] / GRID[1])
GEAR_MASS_KG = 0.5
DROP_HEIGHT_M = 0.03
RESET_PERIOD_S = 100 / 120  # same cadence as the Isaac Lab example: 100 steps at 120 Hz
REPORT_PERIOD_S = 0.5

XML = f"""
<mujoco model="dex01_flat_pad">
  {OPTION_XML}
  <worldbody>
    {ENVIRONMENT_XML}
    <body name="pad" pos="0 0 0.05">
      <geom name="pad" type="box" size="{SENSOR_SIZE[0] / 2} {SENSOR_SIZE[1] / 2} {SENSOR_SIZE[2] / 2}"
            rgba="0.1 0.1 0.1 1" contype="1" conaffinity="1"/>
    </body>
    {gear_body_xml(mass=GEAR_MASS_KG, pos=(0, 0, 0.2))}
  </worldbody>
</mujoco>
"""


def reset(model, data):
    mujoco.mj_resetData(model, data)
    top = model.body("pad").pos[2] + SENSOR_SIZE[2] / 2
    gear_qpos = model.jnt_qposadr[model.joint("gear_free").id]
    data.qpos[gear_qpos : gear_qpos + 3] = (0, 0, top + model.geom("gear_hub").size[1] + DROP_HEIGHT_M)
    mujoco.mj_forward(model, data)


def main():
    model = mujoco.MjModel.from_xml_string(XML)
    data = mujoco.MjData(model)
    pattern = grid_pattern(size=(SENSOR_SIZE[0] - RESOLUTION[0], SENSOR_SIZE[1] - RESOLUTION[1]), resolution=RESOLUTION)
    sensor = TactileSensor(model, "pad", "gear", pattern, offset_pos=(0, 0, SENSOR_SIZE[2] / 2))
    print(f"Tactile sensor on 'pad': {sensor.num_taxels} taxels, frame {pattern.frame_shape}, counter 'gear'")
    reset(model, data)

    reset_steps = round(RESET_PERIOD_S / model.opt.timestep)
    report_steps = round(REPORT_PERIOD_S / model.opt.timestep)
    draw_steps = max(1, round(0.05 / model.opt.timestep))
    viewer = None if args.headless else mujoco.viewer.launch_passive(model, data)
    heatmaps = TactileHeatmaps([sensor], ["Pad"], enabled=viewer is not None and not args.no_tactile_ui)
    if viewer is not None:
        viewer.cam.lookat[:] = (0, 0, 0.05)
        viewer.cam.distance, viewer.cam.elevation, viewer.cam.azimuth = 0.35, -25, 180

    step = 0
    wall_start = time.perf_counter()
    while viewer is None or viewer.is_running():
        if step and step % reset_steps == 0:
            reset(model, data)
            print("reset")
        mujoco.mj_step(model, data)
        sensor.update(data)
        step += 1
        if step % report_steps == 0:
            forces = sensor.data.normal_forces
            print(f"peak taxel force {forces.max():7.4f} N   total {forces.sum():7.4f} N")
        if viewer is not None and step % draw_steps == 0:
            with viewer.lock():
                draw_taxels(viewer.user_scn, [sensor], size=0.0012, show_unloaded=False)
            viewer.sync()
            heatmaps.update()
            # Pace to real time.
            lag = data.time - (time.perf_counter() - wall_start)
            if lag > 0:
                time.sleep(lag)
        if args.max_steps and step >= args.max_steps:
            break
    if viewer is not None:
        viewer.close()
    heatmaps.close()


if __name__ == "__main__":
    main()
