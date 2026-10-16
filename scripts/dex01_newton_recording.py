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

"""Synchronized Newton video and force-array evidence."""

import hashlib
import json

import numpy as np


def save_recording(path, rgb, frames, sensors, steps, provenance):
    import imageio_ffmpeg
    from dex01_sim_asset.vis_utils.viewport import force_frame_rgba

    validate_recording(rgb, frames, sensors, steps)

    if any(np.std(image) < 1 for image in rgb):
        raise RuntimeError("Recording camera has no meaningful image output")
    path.parent.mkdir(parents=True, exist_ok=True)
    width = max(rgb[0].shape[1], 128 * len(sensors))
    height = rgb[0].shape[0] + 128
    vmax = max(0.01, max(float(np.max(f)) for f in frames))
    writer = imageio_ffmpeg.write_frames(str(path), (width, height), fps=30)
    writer.send(None)
    try:
        for image, tactile in zip(rgb, frames):
            composed = np.zeros((height, width, 3), dtype=np.uint8)
            composed[: image.shape[0], : image.shape[1]] = image

            for index, (sensor, frame) in enumerate(zip(sensors, tactile)):
                panel = force_frame_rgba(frame, sensor.taxel2pixel.cpu().numpy(), vmax)[:, :, :3]
                panel = np.repeat(np.repeat(panel, 4, 0), 4, 1)
                composed[-128:, index * 128 : (index + 1) * 128] = panel
            writer.send(composed)
    finally:
        writer.close()

    manifest = {
        "backend": "newton",
        "frame_count": len(rgb),
        "simulation_time_s": [(step + 1) / 240 for step in steps],
        "fps": 30,
        "force_scale_n": vmax,
        "provenance": provenance,
        "video_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "force_frames_interpolated": False,
    }
    arrays = path.with_suffix(".npz")
    np.savez_compressed(
        arrays, frames=np.asarray(frames), steps=steps, taxel2pixel=sensors[0].taxel2pixel.cpu().numpy()
    )
    manifest["arrays_sha256"] = hashlib.sha256(arrays.read_bytes()).hexdigest()
    path.with_suffix(".json").write_text(json.dumps(manifest, indent=2) + "\n")


def validate_recording(rgb, frames, sensors, steps):
    if not rgb or len(rgb) != len(frames) or len(rgb) != len(steps) or len(rgb) < 2:
        raise ValueError("Incomplete synchronized recording")
    if not np.array_equal(np.diff(steps), np.full(len(steps) - 1, 8)):
        raise ValueError("Recording must contain every sampled simulation tick")
    if not sensors or np.asarray(steps).dtype.kind not in "iu" or min(steps) < 0:
        raise ValueError("Invalid recording steps or sensors")
    shape = rgb[0].shape
    for image, tactile in zip(rgb, frames):
        if image.shape != shape or image.ndim != 3 or image.shape[-1] != 3 or image.dtype != np.uint8:
            raise ValueError("Invalid RGB recording frame")
        if len(tactile) != len(sensors) or not np.isfinite(image).all():
            raise ValueError("Incomplete recording sensor set")
        for sensor, frame in zip(sensors, tactile):
            pixels = sensor.taxel2pixel.cpu().numpy()
            occupied = np.zeros((32, 32), dtype=bool)
            occupied[pixels[:, 0], pixels[:, 1]] = True
            if frame.shape != (32, 32) or not np.isfinite(frame).all() or (frame < 0).any() or frame[~occupied].any():
                raise ValueError("Invalid firmware recording frame")
