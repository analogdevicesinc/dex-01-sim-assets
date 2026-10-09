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

from functools import lru_cache
from pathlib import Path

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import gridspec
from matplotlib.animation import FFMpegWriter, FuncAnimation, PillowWriter
from matplotlib.patches import PathPatch
from matplotlib.path import Path as PlotPath
from matplotlib.ticker import ScalarFormatter


@lru_cache(maxsize=1)
def _dex01_mask():
    with np.load(Path(__file__).parents[1] / "taxel_patterns" / "dex01_pattern.npz", allow_pickle=False) as pattern:
        pixel = pattern["taxel2pixel"]
    mask = np.zeros((32, 32), dtype=bool)
    mask[pixel[:, 0], pixel[:, 1]] = True
    return mask


def prettify_dex01(data):
    """Preserve firmware row/column indices and mask only absent DEX-01 taxels."""
    image = np.asarray(data)
    if image.shape != (32, 32):
        raise ValueError("DEX-01 tactile frames must have shape (32, 32)")
    return image, _dex01_mask()


def dex01_plot(ax, data, vmin=None, vmax=None, lw=0.2):
    """Plot the 32x32 firmware frame without changing its physical taxel mapping."""
    image, mask = prettify_dex01(data)
    cmap = plt.cm.viridis.copy()
    cmap.set_bad(color="white")
    im = ax.imshow(np.ma.array(image, mask=~mask), vmin=vmin, vmax=vmax, cmap=cmap)
    ax.axis("off")
    return im


def dex01_hand_plot(fig, ax, data, vmin=None, vmax=None):
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")

    # from thumb to pinky
    fingertip_poses = [
        (0.8, 0.3),
        (0.675, 0.7),
        (0.45, 0.75),
        (0.225, 0.7),
        (0.0, 0.5),
    ]
    width, height = 0.22, 0.22

    vertices = [
        np.add(fingertip_poses[0][:2], (width / 2, height / 2)),
        (fingertip_poses[0][0] + width / 2, 0.25),
        (fingertip_poses[1][0] + width / 2, 0.15),
        (fingertip_poses[1][0] + width / 2, 0.45),
        np.add(fingertip_poses[1][:2], (width / 2, height / 2)),
        (fingertip_poses[1][0] + width / 2, 0.45),
        (fingertip_poses[2][0] + width / 2, 0.45),
        np.add(fingertip_poses[2][:2], (width / 2, height / 2)),
        (fingertip_poses[2][0] + width / 2, 0.45),
        (fingertip_poses[3][0] + width / 2, 0.45),
        np.add(fingertip_poses[3][:2], (width / 2, height / 2)),
        (fingertip_poses[3][0] + width / 2, 0.45),
        (fingertip_poses[4][0] + width / 2, 0.45),
        np.add(fingertip_poses[4][:2], (width / 2, height / 2)),
        (fingertip_poses[4][0] + width / 2, 0.25),
        (0.3, 0.05),
        (fingertip_poses[1][0] + width / 2, 0.15),
    ]
    codes = [
        PlotPath.MOVETO,
        PlotPath.LINETO,
        PlotPath.LINETO,
        PlotPath.LINETO,
        PlotPath.MOVETO,
        PlotPath.LINETO,
        PlotPath.LINETO,
        PlotPath.LINETO,
        PlotPath.LINETO,
        PlotPath.LINETO,
        PlotPath.LINETO,
        PlotPath.LINETO,
        PlotPath.LINETO,
        PlotPath.LINETO,
        PlotPath.LINETO,
        PlotPath.LINETO,
        PlotPath.LINETO,
    ]
    path = PlotPath(vertices, codes)
    patch = PathPatch(path, facecolor="none", edgecolor="black", lw=0.2)
    ax.add_patch(patch)

    bbox = ax.get_position()

    ims = []
    for i, (x, y) in enumerate(fingertip_poses):
        # Convert positions to figure coordinates
        fx = bbox.x0 + x * bbox.width - 0.065
        fy = bbox.y0 + y * bbox.height
        ax_finger = fig.add_axes([fx, fy, width, height])
        im = dex01_plot(ax_finger, data[i], vmin=vmin, vmax=vmax)
        ims.append(im)

    return ims


NON_INTERACTIVE_BACKENDS = frozenset({"agg", "cairo", "pdf", "pgf", "ps", "svg", "template"})


class LiveDex01Viewer:
    """Live matplotlib window showing one DEX-01 taxel force image per sensor.

    Feed it the images produced by :meth:`TactileSensor.get_tactile_image`, shaped ``(32, 32)``.
    """

    def __init__(self, labels, vmax_floor: float = 0.01, vmax_scale: float = 0.5):
        self.labels = list(labels)
        # The colour scale follows the current peak so a light touch stays readable; this is the
        # floor that keeps an untouched sensor from rendering its own numerical noise full-scale.
        self.vmax_floor = vmax_floor
        # Scaling the peak down lets the contact patch saturate instead of leaving most of it in
        # the dark end of the colormap, where the shape of the load is hard to read.
        self.vmax_scale = vmax_scale
        # Headless runs fall back to a file-writing backend, where the window would never appear.
        self.enabled = matplotlib.get_backend().lower() not in NON_INTERACTIVE_BACKENDS
        if not self.enabled:
            print(f"[WARN]: matplotlib backend '{matplotlib.get_backend()}' is not interactive, skipping live plot")
            return

        plt.ion()
        self.fig, axes = plt.subplots(1, len(self.labels), figsize=(1.9 * len(self.labels), 2.4))
        self.axes = np.atleast_1d(axes)
        blank = np.zeros((32, 32), dtype=np.float32)
        self.ims = [dex01_plot(ax, blank, vmin=0.0, vmax=vmax_floor) for ax in self.axes]
        self.fig.colorbar(self.ims[-1], ax=self.axes.tolist(), label="Force (N)", shrink=0.85)
        self.fig.show()

    def update(self, images):
        if not self.enabled:
            return
        peak = max(self.vmax_floor, self.vmax_scale * max(float(img.max()) for img in images))
        for label, ax, im, img in zip(self.labels, self.axes, self.ims, images):
            image, mask = prettify_dex01(img)
            im.set_data(np.ma.array(image, mask=~mask))
            im.set_clim(0.0, peak)
            ax.set_title(f"{label}  {float(img.sum()):.3f} N", fontsize=7)
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()


class FiveFingerDex01Visualizer:
    """Renders a recorded run as a video: the scene camera beside five DEX-01 taxel images."""

    def __init__(self, writer="ffmpeg", fps=60):
        if writer == "ffmpeg":
            self.writer = FFMpegWriter(fps=fps)
            self.ext = ".mp4"
        elif writer == "gif":
            self.writer = PillowWriter(fps=fps)
            self.ext = ".gif"
        else:
            raise ValueError(f"Unknown writer '{writer}'. Expected 'ffmpeg' or 'gif'.")

    def __call__(
        self,
        cam_frames,
        sensor_frames: list,
        name: str,
        max_force: float = None,
        start: int = 0,
    ):
        fig = plt.figure(figsize=(8, 3), dpi=200)
        gs = gridspec.GridSpec(
            3,
            4,
            figure=fig,
            wspace=0.1,
            hspace=0.1,
            left=0.02,
            right=0.98,
            top=0.98,
            bottom=0.02,
            width_ratios=[6, 5, 0.3, 0.3],
            height_ratios=[0.1, 0.8, 0.1],
        )

        rgb_ax = fig.add_subplot(gs[:, 0])
        hand_ax = fig.add_subplot(gs[:, 1])
        cbar_ax = fig.add_subplot(gs[1, 2])
        rgb_ax.axis("off")

        if max_force is None:
            max_force = max(np.asarray(sf).max() for sf in sensor_frames)

        rgb_im = rgb_ax.imshow(cam_frames[0])
        sensor_ims = dex01_hand_plot(
            fig,
            hand_ax,
            [sf[0] for sf in sensor_frames],
            vmin=0,
            vmax=max_force,
        )
        cbar = plt.colorbar(sensor_ims[-1], cax=cbar_ax, shrink=0.3, fraction=0.1, label="Force (N)")
        formatter = ScalarFormatter(useMathText=True)
        formatter.set_powerlimits((0, 0))  # Always scientific
        cbar.ax.yaxis.set_major_formatter(formatter)

        def update(frame):
            i = frame + start
            rgb_im.set_data(cam_frames[i])
            for si, sf in enumerate(sensor_frames):
                image, mask = prettify_dex01(sf[i])
                masked_data = np.ma.array(image, mask=~mask)
                sensor_ims[si].set_data(masked_data)

            return rgb_im, *sensor_ims

        anim = FuncAnimation(fig, update, frames=len(cam_frames) - 1 - start, blit=True)
        anim.save(f"{name}{self.ext}", writer=self.writer)
        plt.close(fig)
        return f"{name}{self.ext}"
