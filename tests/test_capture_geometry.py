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

"""Reject loaded samples outside the independently captured collision surface."""

import hashlib
import importlib.util
import json
import sys
from pathlib import Path

import numpy as np
import pytest
import trimesh

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts/docs"))
spec = importlib.util.spec_from_file_location("capture_checker", ROOT / "scripts/docs/check_capture_geometry.py")
checker = importlib.util.module_from_spec(spec)
spec.loader.exec_module(checker)


def capture(tmp_path, outside=False, inverted=False):
    mesh = trimesh.creation.box(extents=[0.02, 0.004, 0.006])
    if inverted:
        mesh = trimesh.Trimesh(vertices=mesh.vertices, faces=mesh.faces[:, ::-1], process=False)
    poses = np.zeros((4, 2, 7))
    poses[:, :, 6] = 1
    # Rotated and translated counter body: checks must use its physical frame.
    poses[:, 1, :3] = [0.2, -0.1, 0.3]
    poses[:, 1, 5:] = np.sqrt(0.5)
    pixels = np.array([(r, c) for r in range(32) for c in range(32)])
    frames = np.zeros((4, 1, 32, 32))
    frames[:, 0, 0, 0] = 1
    # Local x=8 mm becomes world y=8 mm after a 90 degree z rotation.
    # Omitting the inverse rotation puts it outside the narrow local y extent.
    points = np.tile([0.2, -0.092, 0.3], (4, 1, 1024, 1))
    if outside:
        points[:, 0, 0, 1] += 0.008
    np.savez(
        tmp_path / "motion.npz",
        poses=poses,
        frames=frames,
        taxel2pixel=pixels,
        simulation_time_s=np.arange(4) / 60,
        taxel_points_w=points,
    )
    np.savez(
        tmp_path / "geometry.npz",
        points_0=mesh.vertices,
        counts_0=np.full(len(mesh.faces), 3),
        indices_0=mesh.faces.ravel(),
    )
    manifest = dict(
        example="flat-pad",
        frame_count=4,
        body_paths=["pad", "gear"],
        labels=["Pad"],
        phases=["falling", "contact", "contact", "contact"],
        physics_dt_s=1 / 120,
        capture_stride=2,
        meshes=[dict(path="/World/Asset/gear/collisions", body_index=1)],
        files={p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in tmp_path.iterdir()},
    )
    (tmp_path / "capture.json").write_text(json.dumps(manifest))
    return tmp_path


def test_counter_frame_loaded_sample_passes(tmp_path):
    report = checker.check_capture(capture(tmp_path))
    assert report["loaded_taxel_samples"] == 4
    assert report["outside_mesh_max_distance_m"] == 0


def test_force_outside_mesh_is_rejected(tmp_path):
    with pytest.raises(ValueError, match="outside collision"):
        checker.check_capture(capture(tmp_path, outside=True))


def test_corrupt_capture_is_rejected(tmp_path):
    path = capture(tmp_path)
    (path / "motion.npz").write_bytes(b"corrupted")
    with pytest.raises(ValueError, match="hash mismatch"):
        checker.check_capture(path)


def test_inward_surface_cannot_hide_an_outside_sample(tmp_path):
    with pytest.raises(ValueError, match="outward oriented"):
        checker.check_capture(capture(tmp_path, outside=True, inverted=True))
