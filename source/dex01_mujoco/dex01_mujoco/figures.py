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

"""Still figures for the documentation: an offscreen MuJoCo render above the firmware-frame heatmaps."""

from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np

from .viewer import add_taxels


def save_figure(
    path,
    model,
    data,
    sensors,
    labels,
    camera: mujoco.MjvCamera,
    caption: str,
    taxel_size=0.0003,
    show_unloaded=True,
    size=(1280, 720),
):
    """Render the current state and write it, with one heatmap per sensor on a shared scale, to ``path``."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from PIL import Image

    width, height = size
    model.vis.global_.offwidth = max(model.vis.global_.offwidth, width)
    model.vis.global_.offheight = max(model.vis.global_.offheight, height)
    with mujoco.Renderer(model, height, width) as renderer:
        renderer.update_scene(data, camera)
        add_taxels(renderer.scene, sensors, taxel_size, show_unloaded)
        render = Image.fromarray(renderer.render())

    frames = [sensor.tactile_image() for sensor in sensors]
    vmax = max(1e-3, max(float(frame.max()) for frame in frames))
    cmap = plt.get_cmap("inferno").copy()
    cmap.set_bad((0.82, 0.85, 0.88))
    dpi = 100
    panel_height = 280 if len(sensors) > 1 else 330
    fig, axes = plt.subplots(1, len(sensors), figsize=(width / dpi, panel_height / dpi), dpi=dpi, squeeze=False)
    for ax, frame, sensor, label in zip(axes[0], frames, sensors, labels):
        ax.imshow(
            np.ma.masked_array(frame, ~sensor.frame_mask()), cmap=cmap, vmin=0, vmax=vmax, interpolation="nearest"
        )
        ax.set_title(f"{label}  |  {frame.sum():.3f} N", fontsize=11, color="#1d2b3a")
        ax.set_xticks([])
        ax.set_yticks([])
        for spine in ax.spines.values():
            spine.set_visible(False)
    fig.text(0.5, 0.04, f"Force per taxel: 0–{vmax:.4f} N · grey: no taxel · {caption}", ha="center", color="#1d2b3a")
    fig.subplots_adjust(left=0.05, right=0.95, top=0.86, bottom=0.16, wspace=0.25)
    fig.canvas.draw()
    panel = Image.fromarray(np.asarray(fig.canvas.buffer_rgba())[..., :3])
    plt.close(fig)

    figure = Image.new("RGB", (width, height + panel.height), "white")
    figure.paste(render, (0, 0))
    figure.paste(panel, (0, height))
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.save(path, quality=88, method=6) if path.suffix == ".webp" else figure.save(path)
    return path


def camera(lookat, distance, elevation, azimuth) -> mujoco.MjvCamera:
    """A free camera orbiting ``lookat``."""
    cam = mujoco.MjvCamera()
    cam.lookat[:] = lookat
    cam.distance, cam.elevation, cam.azimuth = distance, elevation, azimuth
    return cam
