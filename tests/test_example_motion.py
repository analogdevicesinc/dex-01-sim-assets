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

"""Prevent documentation clips from hiding timing, pose, or force errors."""

import importlib.util
import json
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
path = ROOT / "scripts/docs/render_examples.py"
spec = importlib.util.spec_from_file_location("_example_renderer", path)
renderer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(renderer)


def grid_capture():
    pixels = np.array([(r, c) for r in range(32) for c in range(32)])
    poses = np.zeros((4, 2, 7))
    poses[:, :, 6] = 1
    manifest = {
        "example": "flat-pad",
        "frame_count": 4,
        "body_paths": ["pad", "gear"],
        "labels": ["Pad"],
        "phases": ["falling", "impact", "settling", "settling"],
        "physics_dt_s": 1 / 120,
        "capture_stride": 2,
    }
    arrays = {
        "poses": poses,
        "frames": np.zeros((4, 1, 32, 32)),
        "taxel2pixel": pixels,
        "simulation_time_s": np.arange(4) / 60,
    }
    return manifest, arrays


def test_exact_grid_sequence_passes():
    manifest, arrays = grid_capture()
    renderer.validate_motion(manifest, arrays)


@pytest.mark.parametrize(
    "error", ["time_gap", "negative_force", "invalid_rotation", "duplicate_pixel", "missing_frame"]
)
def test_invalid_grid_sequence_is_rejected(error):
    manifest, arrays = grid_capture()
    if error == "time_gap":
        arrays["simulation_time_s"][2] += 0.1
    elif error == "negative_force":
        arrays["frames"][1, 0, 5, 6] = -1
    elif error == "invalid_rotation":
        arrays["poses"][1, 1, 6] = 0
    elif error == "duplicate_pixel":
        arrays["taxel2pixel"][2] = arrays["taxel2pixel"][1]
    else:
        arrays["frames"] = arrays["frames"][:-1]
    with pytest.raises(ValueError):
        renderer.validate_motion(manifest, arrays)


def test_unused_dex01_pixels_cannot_hide_force():
    manifest, arrays = grid_capture()
    with np.load(ROOT / "source/dex01_sim_asset/dex01_sim_asset/taxel_patterns/dex01_pattern.npz") as p:
        pixels = p["taxel2pixel"]
    manifest.update(labels=[f"Finger {i}" for i in range(1, 6)])
    arrays["frames"] = np.zeros((4, 5, 32, 32))
    arrays["taxel2pixel"] = pixels
    occupied = np.zeros((32, 32), dtype=bool)
    occupied[pixels[:, 0], pixels[:, 1]] = True
    row, col = np.argwhere(~occupied)[0]
    arrays["frames"][1, 1, row, col] = 1
    with pytest.raises(ValueError, match="unoccupied"):
        renderer.validate_motion(manifest, arrays)


def test_stationary_press_rejects_moving_wrist():
    manifest, arrays = grid_capture()
    with np.load(ROOT / "source/dex01_sim_asset/dex01_sim_asset/taxel_patterns/dex01_pattern.npz") as p:
        pixels = p["taxel2pixel"]
    manifest.update(
        example="stationary-press",
        labels=[f"Finger {i}" for i in range(1, 6)],
        body_paths=["/World/envs/env_0/Robot/rl_dg_mount", "/World/envs/env_0/Asset/factory_gear_medium"],
        phases=["unloaded", "approach", "hold", "retract"],
    )
    arrays["frames"] = np.zeros((4, 5, 32, 32))
    arrays["taxel2pixel"] = pixels
    arrays["poses"][2, 0, 0] = 0.001
    with pytest.raises(ValueError, match="wrist"):
        renderer.validate_motion(manifest, arrays)


def test_old_capture_without_geometry_is_rejected(tmp_path, monkeypatch):
    (tmp_path / "capture.json").write_text(json.dumps({"files": {}}))
    monkeypatch.setattr(
        sys, "argv", ["render_examples.py", "--capture", str(tmp_path), "--output", str(tmp_path / "out")]
    )
    with pytest.raises(RuntimeError, match="recapture"):
        renderer.main()


@pytest.fixture
def geometry_tools():
    pytest.importorskip("pxr.Usd")
    spec = importlib.util.spec_from_file_location("_motion_geometry", ROOT / "scripts/docs/motion_geometry.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize("scale", [0.15, 0.19671368598937988])
def test_replay_preserves_scaled_offset_mesh_after_export(tmp_path, geometry_tools, scale):
    from pxr import Gf, Usd, UsdGeom

    stage = Usd.Stage.CreateInMemory()
    wrapper = UsdGeom.Xform.Define(stage, "/Asset")
    wrapper.AddTranslateOp().Set(Gf.Vec3d(0.1, 0.2, 0.3))
    wrapper.AddRotateZOp().Set(30)
    wrapper.AddScaleOp().Set(Gf.Vec3d(scale))
    body = UsdGeom.Xform.Define(stage, "/Asset/gear")
    body.AddRotateXOp().Set(20)
    mesh = UsdGeom.Mesh.Define(stage, "/Asset/gear/visual")
    points = np.array([[0, 0, 0], [0.04, 0, 0], [0, 0.04, 0.025]])
    mesh.CreatePointsAttr(points.tolist())
    mesh.CreateFaceVertexCountsAttr([3])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2])
    mesh.AddTranslateOp().Set(Gf.Vec3d(0.02, 0, 0.005))
    paths = ["/Asset/gear"]
    meshes, arrays = geometry_tools.capture_geometry(stage, paths)
    expected_local = (points + [0.02, 0, 0.005]) * scale
    assert np.allclose(arrays["points_0"], expected_local, atol=1e-9)

    # Exercise both export and playback, including a second move after reload.
    first = np.array([[0.4, 0.5, 0.6, 0, 0, 0, 1]])
    geometry_tools.apply_body_poses(stage, paths, first, arrays["body_scales"])
    geometry_tools.verify_geometry(stage, paths, first, meshes, arrays)
    stage.GetRootLayer().Export(str(tmp_path / "scene.usda"))
    stage = Usd.Stage.Open(str(tmp_path / "scene.usda"))
    second = np.array([[0.7, 0.8, 0.9, 0, 0, np.sqrt(0.5), np.sqrt(0.5)]])
    geometry_tools.apply_body_poses(stage, paths, second, arrays["body_scales"])
    geometry_tools.verify_geometry(stage, paths, second, meshes, arrays)
    mesh = UsdGeom.Mesh(stage.GetPrimAtPath(meshes[0]["path"]))
    world = mesh.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    actual = np.array([world.Transform(Gf.Vec3d(*p)) for p in points])
    expected = expected_local[:, [1, 0, 2]] * [-1, 1, 1] + second[0, :3]
    assert np.allclose(actual, expected, atol=1e-9)

    # Reproducing the old bug must fail even though position/orientation match.
    geometry_tools.apply_body_poses(stage, paths, second, np.ones((1, 3)))
    with pytest.raises(RuntimeError, match="scale"):
        geometry_tools.verify_geometry(stage, paths, second, meshes, arrays)
    geometry_tools.apply_body_poses(stage, paths, second, arrays["body_scales"])
    # Body checks alone also miss changes to a descendant's visible surface.
    mesh.GetPointsAttr().Set((points + [0, 0, 0.002]).tolist())
    with pytest.raises(RuntimeError, match="surface"):
        geometry_tools.verify_geometry(stage, paths, second, meshes, arrays)
