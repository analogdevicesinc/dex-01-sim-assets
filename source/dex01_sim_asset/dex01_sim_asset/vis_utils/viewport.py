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

"""Tactile images composited into the Kit viewport used by WebRTC.

Imports of Kit UI are deferred until construction, after AppLauncher startup.
Pure image conversion can be tested without Isaac Sim or matplotlib.
"""

import numpy as np


def force_frame_rgba(frame, pixels, vmax):
    """Map firmware cells to RGBA, preserving rows/columns and absent-cell mask.

    Blue is zero, cyan/yellow/red is increasing normal force. No interpolation
    or reshuffling: an enlarged cell still represents exactly one taxel.
    """
    frame = np.asarray(frame)
    pixels = np.asarray(pixels, dtype=int)
    if frame.ndim != 2 or not np.isfinite(frame).all() or (frame < 0).any():
        raise ValueError("Expected a finite nonnegative 2D force frame")
    if not np.isfinite(vmax) or vmax <= 0:
        raise ValueError("Colour scale must be finite and positive")
    mask = np.zeros(frame.shape, dtype=bool)
    mask[pixels[:, 0], pixels[:, 1]] = True
    t = np.clip(frame / vmax, 0, 1)
    # Piecewise blue -> cyan -> yellow -> red, identical for every sensor.
    anchors = np.array([[12, 20, 50], [0, 190, 220], [255, 230, 0], [255, 30, 0]])
    x = t * 3
    lo = np.minimum(x.astype(int), 2)
    rgb = anchors[lo] * (1 - (x - lo)[..., None]) + anchors[lo + 1] * (x - lo)[..., None]
    rgba = np.full((*frame.shape, 4), 255, dtype=np.uint8)
    rgba[..., :3] = rgb.astype(np.uint8)
    rgba[~mask, :3] = (65, 65, 65)
    return rgba


class ViewportTactileViewer:
    """Small viewport overlay: force frames, loads, scale and optional demo state.

    Unlike an OS plot window, this UI is part of Kit's streamed application.
    A headless batch run without a viewport simply skips the overlay. Offscreen
    Camera sensor recordings do not include this UI layer.
    """

    def __init__(self, sensors, labels, enabled=True, vmax_floor=0.01):
        self.sensors = list(sensors)
        self.labels = list(labels)
        if len(self.sensors) != len(self.labels) or not self.sensors:
            raise ValueError("Supply one label for each sensor")
        self.vmax_floor = vmax_floor
        self.frame = None
        self.providers = []
        self.force_labels = []
        if not enabled:
            return
        import omni.ui as ui
        from omni.kit.viewport.utility import get_active_viewport_window

        viewport = get_active_viewport_window()
        if viewport is None:
            print("[INFO]: No GUI viewport; tactile overlay disabled for batch execution.")
            return
        self.frame = viewport.get_frame("dex01.tactile.force_overlay")
        self.pixels = [sensor.taxel2pixel.cpu().numpy() for sensor in self.sensors]
        with self.frame:
            with ui.VStack():
                ui.Spacer()
                with ui.HStack(height=265):
                    ui.Spacer(width=12)
                    with ui.ZStack(width=185 * len(self.sensors), height=265):
                        ui.Rectangle(style={"background_color": 0xEE181818})
                        with ui.VStack(spacing=4):
                            self.status_label = ui.Label(
                                "Normal force (N); firmware rows/columns", height=28, style={"font_size": 18}
                            )
                            with ui.HStack(height=186, spacing=6):
                                for label in self.labels:
                                    with ui.VStack(width=178):
                                        self.force_labels.append(ui.Label(label, height=26, style={"font_size": 18}))
                                        provider = ui.ByteImageProvider()
                                        self.providers.append(provider)
                                        ui.ImageWithProvider(provider, width=160, height=160)
                            self.scale_label = ui.Label("", height=28, style={"font_size": 16})
                    ui.Spacer()
                ui.Spacer(height=12)
        self.update()

    def update(self, status=""):
        if self.frame is None:
            return
        images = [sensor.get_tactile_image()[0].detach().cpu().numpy() for sensor in self.sensors]
        vmax = max(self.vmax_floor, max(float(image.max()) for image in images))
        for label, text, provider, image, pixels in zip(
            self.labels, self.force_labels, self.providers, images, self.pixels
        ):
            text.text = f"{label}: {float(image.sum()):.3f} N"
            rgba = force_frame_rgba(image, pixels, vmax)
            # Explicit nearest-neighbour enlargement avoids smoothing a gear's teeth.
            rgba = np.repeat(np.repeat(rgba, 4, axis=0), 4, axis=1)
            provider.set_bytes_data(rgba.ravel().tolist(), [rgba.shape[1], rgba.shape[0]])
        self.scale_label.text = f"Blue 0 -> cyan -> yellow -> red {vmax:.4f} N/taxel; grey: absent"
        self.status_label.text = status or "Normal force (N); firmware rows/columns; env 0"

    def close(self):
        if self.frame is not None:
            self.frame.clear()
            self.frame = None
