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

"""Export Newton state and tactile data for the shared audited renderer."""

import hashlib
import json
import subprocess
import tempfile
from pathlib import Path

import numpy as np

from scripts.docs.motion_geometry import (
    apply_body_poses,
    capture_geometry,
    verify_geometry,
)


class NewtonCapture:
    """Capture environment zero using public Newton link poses, without replaying physics."""

    def __init__(self, stage, body_paths, labels, mode, destination, provenance, requested_steps):
        self.stage = stage
        self.paths = list(body_paths)
        self.labels = labels
        self.mode = mode
        self.destination = Path(destination)
        self.provenance = provenance
        self.requested_steps = requested_steps
        self.meshes, self.geometry = capture_geometry(stage, self.paths)
        self.samples = []

    def append(self, step, poses, sensors):
        frames, taxels = [], []
        for sensor in sensors:
            frame = sensor.get_tactile_image()[0].cpu().numpy().copy()
            forces = sensor.data.normal_forces[0].cpu().numpy().copy()
            pixels = sensor.taxel2pixel.cpu().numpy()
            if not np.array_equal(frame[pixels[:, 0], pixels[:, 1]], forces):
                raise ValueError("Captured firmware mapping differs from taxel forces")
            frames.append(frame)
            taxels.append(forces)
        pixels = sensors[0].taxel2pixel.cpu().numpy().copy()
        if any(not np.array_equal(pixels, sensor.taxel2pixel.cpu().numpy()) for sensor in sensors):
            raise ValueError("Captured sensors must share firmware mapping")
        self.samples.append((step, np.asarray(poses).copy(), np.stack(frames), np.stack(taxels), pixels))

    def save(self, completed_steps):
        from scripts.docs.render_examples import validate_motion

        if type(completed_steps) is not int or completed_steps != self.requested_steps or len(self.samples) < 2:
            raise RuntimeError("Incomplete Newton capture")
        steps, poses, frames, taxels, pixels = zip(*self.samples)
        times = (np.asarray(steps) + 1) / 240
        phases = []
        for step in steps:
            t = step / 240 % 2.5
            phases.append(
                ("unloaded" if t < 0.25 else "approach" if t < 1 else "hold" if t < 1.5 else "retract")
                if self.mode == "press"
                else f"Finger {2 + (step // 360) % 4} contact"
            )
        manifest = {
            "backend": "newton",
            "example": "stationary-press" if self.mode == "press" else "hand",
            "physics_version": self.provenance["packages"]["newton"],
            "physics_dt_s": 1 / 240,
            "requested_steps": self.requested_steps,
            "completed_steps": completed_steps,
            "steps": list(steps),
            "capture_stride": 8,
            "frame_count": len(steps),
            "body_paths": self.paths,
            "labels": self.labels,
            "phases": phases,
            "meshes": self.meshes,
            "body_scales": self.geometry["body_scales"].tolist(),
            "playback_fps": 30,
            "provenance": self.provenance,
            "source_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip(),
            "geometry_script_sha256": hashlib.sha256(
                (Path(__file__).resolve().parents[2] / "scripts/docs/motion_geometry.py").read_bytes()
            ).hexdigest(),
        }
        arrays = {
            "poses": np.stack(poses),
            "frames": np.stack(frames),
            "taxel_forces_n": np.stack(taxels),
            "taxel2pixel": pixels[0],
            "simulation_time_s": times,
        }
        validate_motion(manifest, arrays)
        apply_body_poses(self.stage, self.paths, arrays["poses"][0], self.geometry["body_scales"])
        _, error = verify_geometry(self.stage, self.paths, arrays["poses"][0], self.meshes, self.geometry)
        manifest["maximum_exported_vertex_error_m"] = error
        # Export only after all numerical and geometry checks pass.
        self.destination.parent.mkdir(parents=True, exist_ok=True)
        if self.destination.exists():
            raise FileExistsError(self.destination)
        with tempfile.TemporaryDirectory(prefix="newton-capture-", dir=self.destination.parent) as temporary:
            staging = Path(temporary) / "capture"
            staging.mkdir()
            if not self.stage.GetRootLayer().Export(str(staging / "scene.usda")):
                raise RuntimeError("USD export failed")
            from pxr import Usd

            reloaded = Usd.Stage.Open(str(staging / "scene.usda"))
            verify_geometry(reloaded, self.paths, arrays["poses"][0], self.meshes, self.geometry)
            np.savez_compressed(staging / "geometry.npz", **self.geometry)
            np.savez_compressed(staging / "motion.npz", **arrays)
            manifest["files"] = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in staging.iterdir()}
            (staging / "capture.json").write_text(json.dumps(manifest, indent=2) + "\n")
            staging.rename(self.destination)
