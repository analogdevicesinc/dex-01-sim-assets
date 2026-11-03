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

"""Live tactile display: a matplotlib heatmap window and coloured taxel markers in the MuJoCo viewer."""

from __future__ import annotations

import mujoco
import numpy as np


class TactileHeatmaps:
    """One firmware-frame heatmap per sensor; grey marks cells with no taxel."""

    def __init__(self, sensors, labels, enabled=True):
        self.enabled = enabled
        if not enabled:
            return
        import matplotlib.pyplot as plt

        self._plt = plt
        plt.ion()
        self.sensors = sensors
        self.fig, axes = plt.subplots(1, len(sensors), figsize=(2.6 * len(sensors) + 1, 3.2), squeeze=False)
        self.fig.canvas.manager.set_window_title("DEX-01 tactile frames")
        cmap = plt.get_cmap("inferno").copy()
        cmap.set_bad((0.55, 0.55, 0.55))
        self.images, self.titles = [], []
        for ax, sensor, label in zip(axes[0], sensors, labels):
            frame = np.ma.masked_array(sensor.tactile_image(), ~sensor.frame_mask())
            self.images.append(ax.imshow(frame, cmap=cmap, vmin=0, vmax=0.01, interpolation="nearest"))
            self.titles.append(ax.set_title(label, fontsize=9))
            ax.set_xticks([])
            ax.set_yticks([])
        self.colorbar = self.fig.colorbar(self.images[-1], ax=axes[0].tolist(), label="taxel force (N)")
        plt.show(block=False)

    def update(self):
        if not self.enabled or not self._plt.fignum_exists(self.fig.number):
            return
        frames = [sensor.tactile_image() for sensor in self.sensors]
        vmax = max(1e-3, max(frame.max() for frame in frames))  # one shared scale per redraw
        for image, title, frame, sensor in zip(self.images, self.titles, frames, self.sensors):
            image.set_data(np.ma.masked_array(frame, ~sensor.frame_mask()))
            image.set_clim(0, vmax)
            title.set_text(f"{title.get_text().split()[0]}  {frame.sum():.3f} N")
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def close(self):
        if self.enabled:
            self._plt.close(self.fig)


def draw_taxels(scene: mujoco.MjvScene, sensors, size=0.0003, show_unloaded=True):
    """Draw taxels as small spheres coloured by force (grey when unloaded) into a viewer's user scene."""
    scene.ngeom = 0
    peak = max(1e-6, max(float(s.data.normal_forces.max()) for s in sensors))
    for sensor in sensors:
        forces = sensor.data.normal_forces
        for point, force in zip(sensor.data.taxel_points_w, forces):
            if scene.ngeom >= scene.maxgeom:
                return
            if force <= 0 and not show_unloaded:
                continue
            u = float(force) / peak
            rgba = np.array([1.0, 0.15 + 0.85 * u, 0.1, 1.0]) if force > 0 else np.array([0.6, 0.6, 0.65, 0.6])
            radius = size * (1.6 if force > 0 else 1.0)
            mujoco.mjv_initGeom(
                scene.geoms[scene.ngeom],
                mujoco.mjtGeom.mjGEOM_SPHERE,
                np.array([radius, 0, 0]),
                point,
                np.eye(3).ravel(),
                rgba.astype(np.float32),
            )
            scene.ngeom += 1
