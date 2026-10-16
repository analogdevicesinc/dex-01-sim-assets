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

"""Test real USD export/reload and reject corrupted Newton recording evidence."""

import hashlib
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import torch
from pxr import Gf, Usd, UsdGeom

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "newton/scripts"))
from capture import NewtonCapture
from dex01_newton_recording import validate_recording

from scripts.docs.motion_geometry import apply_body_poses, verify_geometry
from scripts.docs.render_examples import validate_motion


@pytest.fixture
def sensor():
    pixels = np.load(ROOT / "common/source/dex01_sim_common/dex01_sim_common/taxel_patterns/dex01_contact.npz")[
        "taxel2pixel"
    ]
    forces = torch.zeros(1, 738)
    forces[0, 0] = 1
    frame = torch.zeros(1, 32, 32)
    frame[0, pixels[0, 0], pixels[0, 1]] = 1
    return SimpleNamespace(
        taxel2pixel=torch.tensor(pixels), data=SimpleNamespace(normal_forces=forces), get_tactile_image=lambda: frame
    )


@pytest.fixture
def capture(tmp_path, sensor):
    stage = Usd.Stage.CreateInMemory()
    wrapper = UsdGeom.Xform.Define(stage, "/World")
    wrapper.AddScaleOp().Set(Gf.Vec3d(0.2))
    paths = ["/World/Robot/rl_dg_mount", "/World/Probe"]
    for path in paths:
        UsdGeom.Xform.Define(stage, path)
        mesh = UsdGeom.Mesh.Define(stage, path + "/collision")
        mesh.CreatePointsAttr([[0, 0, 0], [0.01, 0, 0], [0, 0.01, 0]])
        mesh.CreateFaceVertexCountsAttr([3])
        mesh.CreateFaceVertexIndicesAttr([0, 1, 2])
        mesh.AddTranslateOp().Set(Gf.Vec3d(0.002, 0, 0))
    capture = NewtonCapture(
        stage,
        paths,
        [f"Finger {i}" for i in range(1, 6)],
        "hand",
        tmp_path / "capture",
        {"packages": {"newton": "1.5.2"}},
        1440,
    )
    poses = np.array([[0, 0, 0, 0, 0, 0, 1], [0, 0, 0.1, 0, 0, 0, 1]])
    for step in range(0, 1440, 8):
        capture.append(step, poses, [sensor] * 5)
    return capture


def test_newton_capture_export_reload_preserves_scale_offsets_and_forces(capture):
    capture.save(1440)
    path = capture.destination
    manifest = json.loads((path / "capture.json").read_text())
    assert manifest["backend"] == "newton"
    for name, sha in manifest["files"].items():
        assert hashlib.sha256((path / name).read_bytes()).hexdigest() == sha
    with np.load(path / "motion.npz") as arrays:
        validate_motion(manifest, arrays)
        poses = arrays["poses"][-1].copy()
    with np.load(path / "geometry.npz") as arrays:
        geometry = dict(arrays)
    stage = Usd.Stage.Open(str(path / "scene.usda"))
    poses[:, 0] += 0.02
    apply_body_poses(stage, manifest["body_paths"], poses, geometry["body_scales"])
    verify_geometry(stage, manifest["body_paths"], poses, manifest["meshes"], geometry)
    stage.GetPrimAtPath("/World/Probe/collision").GetAttribute("points").Set([[0, 0, 0], [0.02, 0, 0], [0, 0.01, 0]])
    with pytest.raises(RuntimeError, match="surface"):
        verify_geometry(stage, manifest["body_paths"], poses, manifest["meshes"], geometry)


@pytest.mark.parametrize("corruption", ["incomplete", "step", "time", "taxel", "quaternion", "negative"])
def test_newton_capture_rejects_bad_evidence_before_export(capture, corruption):
    if corruption == "incomplete":
        with pytest.raises(RuntimeError, match="Incomplete"):
            capture.save(0)
    else:
        step, poses, frames, taxels, pixels = capture.samples[1]
        if corruption == "step":
            capture.samples[1] = (9, poses, frames, taxels, pixels)
        elif corruption == "taxel":
            taxels[0, 0] += 1
        elif corruption == "quaternion":
            poses[0, 6] = 0
        elif corruption == "negative":
            frames[0, 0, 0] = -1
        else:
            capture.save(1440)
            manifest = json.loads((capture.destination / "capture.json").read_text())
            with np.load(capture.destination / "motion.npz") as arrays:
                data = dict(arrays)
            data["simulation_time_s"] += 0.1
            with pytest.raises(ValueError, match="time differs"):
                validate_motion(manifest, data)
            return
        with pytest.raises(ValueError):
            capture.save(1440)
    assert not capture.destination.exists()


@pytest.mark.parametrize("corruption", ["count", "gap", "negative", "unused", "nan", "rgb"])
def test_recording_rejects_desynchronized_or_invalid_frames(sensor, corruption):
    rgb = [np.zeros((16, 16, 3), dtype=np.uint8) for _ in range(3)]
    frames = [[sensor.get_tactile_image()[0].numpy().copy()] for _ in range(3)]
    steps = [64, 72, 80]
    if corruption == "count":
        frames.pop()
    elif corruption == "gap":
        steps[-1] += 8
    elif corruption == "negative":
        frames[0][0][0, 0] = -1
    elif corruption == "unused":
        occupied = np.zeros((32, 32), dtype=bool)
        occupied[tuple(sensor.taxel2pixel.numpy().T)] = True
        frames[0][0][tuple(np.argwhere(~occupied)[0])] = 1
    elif corruption == "nan":
        frames[0][0][0, 0] = np.nan
    else:
        rgb[0] = rgb[0].astype(float)
    with pytest.raises(ValueError):
        validate_recording(rgb, frames, [sensor], steps)


def test_newton_capture_rejects_mapping_mismatch(capture, sensor):
    sensor.data.normal_forces[0, 0] = 2
    with pytest.raises(ValueError, match="mapping"):
        capture.append(24, np.zeros((2, 7)), [sensor] * 5)


def test_video_writer_records_fixed_scale_and_matching_arrays(tmp_path, sensor, monkeypatch):
    import importlib.util

    import imageio_ffmpeg

    spec = importlib.util.spec_from_file_location(
        "dex01_sim_common.vis_utils.viewport",
        ROOT / "common/source/dex01_sim_common/dex01_sim_common/vis_utils/viewport.py",
    )
    viewport = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(viewport)
    monkeypatch.setitem(sys.modules, "dex01_sim_common.vis_utils.viewport", viewport)
    from dex01_newton_recording import save_recording

    calls = []
    original = viewport.force_frame_rgba

    def color(frame, pixels, vmax):
        calls.append(vmax)
        return original(frame, pixels, vmax)

    monkeypatch.setattr(viewport, "force_frame_rgba", color)
    rgb = [np.indices((32, 32))[0].astype(np.uint8)[..., None].repeat(3, axis=-1) for _ in range(3)]
    frames = [[sensor.get_tactile_image()[0].numpy().copy() * scale] for scale in (1, 2, 3)]
    path = tmp_path / "recording.mp4"
    report = path.with_suffix(".json")
    report.write_text('{"backend":"newton","observations":["preserve simulation report"]}')
    save_recording(path, rgb, frames, [sensor], [64, 72, 80], {"source_commit": "test"})
    assert json.loads(report.read_text())["observations"] == ["preserve simulation report"]
    manifest = json.loads(path.with_suffix(".recording.json").read_text())
    assert calls == [3, 3, 3]
    assert manifest["force_scale_n"] == 3
    assert manifest["simulation_time_s"] == [65 / 240, 73 / 240, 81 / 240]
    assert hashlib.sha256(path.read_bytes()).hexdigest() == manifest["video_sha256"]
    assert hashlib.sha256(path.with_suffix(".recording.npz").read_bytes()).hexdigest() == manifest["arrays_sha256"]
    with np.load(path.with_suffix(".recording.npz")) as arrays:
        np.testing.assert_array_equal(arrays["frames"], frames)
    reader = imageio_ffmpeg.read_frames(str(path))
    metadata = next(reader)
    assert metadata["fps"] == 30
    assert len(list(reader)) == 3


def test_existing_capture_is_preserved(capture):
    capture.destination.mkdir()
    sentinel = capture.destination / "keep"
    sentinel.write_text("existing")
    with pytest.raises(FileExistsError):
        capture.save(1440)
    assert sentinel.read_text() == "existing"


def test_renderer_rejects_corrupt_newton_motion_before_worker(capture, tmp_path, monkeypatch):
    from scripts.docs import render_examples

    capture.save(1440)
    manifest_path = capture.destination / "capture.json"
    manifest = json.loads(manifest_path.read_text())
    motion_path = capture.destination / "motion.npz"
    with np.load(motion_path) as arrays:
        corrupt = dict(arrays)
    corrupt["simulation_time_s"] += 0.1
    np.savez_compressed(motion_path, **corrupt)
    manifest["files"]["motion.npz"] = hashlib.sha256(motion_path.read_bytes()).hexdigest()
    manifest_path.write_text(json.dumps(manifest))
    monkeypatch.setattr(
        sys, "argv", ["render_examples.py", "--capture", str(capture.destination), "--output", str(tmp_path / "render")]
    )

    def no_launch(*args, **kwargs):
        pytest.fail("Renderer launched before validating capture")

    monkeypatch.setattr(render_examples.subprocess, "run", no_launch)
    with pytest.raises(ValueError, match="time differs"):
        render_examples.main()


@pytest.mark.parametrize(
    "flags",
    [[], ["--mode", "sensor", "--report", "unused"], ["--mode", "press", "--report", "unused", "--max-steps", "100"]],
)
def test_cli_rejects_capture_without_complete_report_before_imports(tmp_path, flags):
    import subprocess

    result = subprocess.run(
        [
            sys.executable,
            "-S",
            str(ROOT / "newton/scripts/dex01_newton.py"),
            "--capture",
            str(tmp_path / "capture"),
            *flags,
        ],
        capture_output=True,
        text=True,
    )
    assert result.returncode == 2
    assert "--capture requires" in result.stderr
    assert not (tmp_path / "capture").exists()


@pytest.mark.parametrize("collision", ["video", "sidecar", "benchmark", "capture"])
def test_cli_rejects_output_collisions_before_invalidating_report(tmp_path, collision):
    import subprocess

    video = tmp_path / "demo.mp4"
    report = tmp_path / (
        "demo.mp4" if collision == "video" else "demo.recording.json" if collision == "sidecar" else "report.json"
    )
    original = '{"observations":["existing evidence"]}'
    report.write_text(original)
    args = ["--record", str(video), "--report", str(report)]
    if collision == "benchmark":
        args += ["--benchmark", str(report)]
    elif collision == "capture":
        capture = tmp_path / "capture"
        report.unlink()
        capture.mkdir()
        report = capture / "report.json"
        report.write_text(original)
        args = ["--record", str(video), "--report", str(report), "--capture", str(capture)]
    result = subprocess.run(
        [sys.executable, "-S", str(ROOT / "newton/scripts/dex01_newton.py"), *args], capture_output=True, text=True
    )
    assert result.returncode == 2
    assert report.read_text() == original
    assert not video.exists() or video == report


@pytest.mark.parametrize("corruption", ["head", "tail", "two_frames", "phases", "press_release", "requested"])
def test_capture_requires_complete_expected_sample_schedule(capture, corruption):
    capture.save(1440)
    manifest = json.loads((capture.destination / "capture.json").read_text())
    with np.load(capture.destination / "motion.npz") as archive:
        arrays = dict(archive)
    if corruption == "requested":
        manifest["completed_steps"] -= 1
    elif corruption == "phases":
        manifest["phases"][0] = "wrong phase"
    else:
        keep = slice(1, None) if corruption == "head" else slice(None, -1) if corruption == "tail" else slice(None, 2)
        if corruption == "press_release":
            keep = slice(0, 50)
            manifest.update(example="stationary-press", requested_steps=1800, completed_steps=1800)
            manifest["steps"] = list(range(1200, 1800, 8))
            manifest["phases"] = [
                "unloaded" if step < 1260 else "approach" if step < 1440 else "hold" if step < 1560 else "retract"
                for step in manifest["steps"]
            ]
            arrays["simulation_time_s"] = (np.asarray(manifest["steps"]) + 1) / 240
        for name in ("poses", "frames", "taxel_forces_n", "simulation_time_s"):
            arrays[name] = arrays[name][keep]
        manifest["steps"] = manifest["steps"][keep]
        manifest["phases"] = manifest["phases"][keep]
        manifest["frame_count"] = len(manifest["steps"])
    with pytest.raises(ValueError, match="Newton capture"):
        validate_motion(manifest, arrays)
