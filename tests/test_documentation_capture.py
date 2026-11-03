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

"""Reject misleading documentation captures without importing Isaac Sim."""

import hashlib
import importlib.util
import json
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


def load_script(name):
    path = ROOT / "scripts/docs" / name
    spec = importlib.util.spec_from_file_location("_docs_" + path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_robotiq_published_media_matches_verified_capture():
    """The page must ship its actual clip/stills and matching provenance."""
    from PIL import Image

    root = ROOT / "docs/_static/videos"
    evidence = json.loads((ROOT / "docs/media/examples.json").read_text())["robotiq"]
    assert evidence["frame_count"] == 180
    assert evidence["duration_s"] == 6.0
    assert evidence["contact_closeup_camera"]
    assert not evidence["physical_timeline_advanced_by_renderer"]
    assert not evidence["force_frames_interpolated"]
    assert evidence["maximum_rendered_vertex_error_m"] < 1e-6
    assert evidence["capture"]["grasp_report_verified"]
    assert evidence["grasp_report_summary"]["maximum_hold_force_difference_n"] < 0.1
    assert evidence["grasp_report_summary"]["maximum_momentum_residual_n_s"] < 1e-6
    for name, sha in evidence["files"].items():
        path = root / name
        assert path.is_file() and path.stat().st_size < 2_000_000
        assert hashlib.sha256(path.read_bytes()).hexdigest() == sha
    for phase in ["before", "contact", "after"]:
        assert Image.open(root / f"robotiq-{phase}.webp").size == (960, 800)
    captions = (root / "robotiq.vtt").read_text()
    assert captions.startswith("WEBVTT") and "Hold" in captions and "Released" in captions
    page = (ROOT / "docs/examples/robotiq.md").read_text()
    for name in [
        "robotiq.mp4",
        "robotiq.webm",
        "robotiq.vtt",
        "robotiq-before.webp",
        "robotiq-contact.webp",
        "robotiq-after.webp",
    ]:
        assert name in page


@pytest.mark.parametrize("failure_mode", ["missing", "exception", "corrupt", "misaligned", "process"])
def test_capture_rejects_failed_worker_even_if_kit_exits_zero(tmp_path, monkeypatch, failure_mode):
    exporter = load_script("export_scene.py")

    def worker(command, check):
        staging = Path(command[command.index("--output") + 1])
        if failure_mode == "missing":
            return subprocess.CompletedProcess(command, 0)
        if failure_mode == "exception":
            (staging / "failure.txt").write_text("Failed mesh alignment")
            return subprocess.CompletedProcess(command, 0)
        payloads = {"scene.usda": b"captured scene", "tactile.npz": b"captured frame"}
        for name, data in payloads.items():
            (staging / name).write_bytes(data)
        manifest = {
            "sensor_visual_count": 5,
            "taxel_surface_max_error_m": [2e-6 if failure_mode == "misaligned" else 1e-8] * 5,
            "scene_sha256": hashlib.sha256(payloads["scene.usda"]).hexdigest(),
            "tactile_sha256": hashlib.sha256(payloads["tactile.npz"]).hexdigest(),
        }
        (staging / "manifest.json").write_text(json.dumps(manifest))
        if failure_mode == "corrupt":
            (staging / "scene.usda").write_bytes(b"wrong scene")
        return subprocess.CompletedProcess(command, 1 if failure_mode == "process" else 0)

    monkeypatch.setattr(exporter.subprocess, "run", worker)
    destination = tmp_path / "capture"
    with pytest.raises(RuntimeError):
        exporter.capture_checked(destination, 240)
    assert not destination.exists()


def test_capture_copies_only_verified_artifacts(tmp_path, monkeypatch):
    exporter = load_script("export_scene.py")

    def worker(command, check):
        staging = Path(command[command.index("--output") + 1])
        data = b"checked output"
        for name in ("scene.usda", "tactile.npz"):
            (staging / name).write_bytes(data)
        sha = hashlib.sha256(data).hexdigest()
        (staging / "manifest.json").write_text(
            json.dumps({
                "sensor_visual_count": 5,
                "taxel_surface_max_error_m": [1e-8] * 5,
                "scene_sha256": sha,
                "tactile_sha256": sha,
            })
        )
        return subprocess.CompletedProcess(command, 0)

    monkeypatch.setattr(exporter.subprocess, "run", worker)
    destination = tmp_path / "capture"
    exporter.capture_checked(destination, 240)
    assert {p.name for p in destination.iterdir()} == {"scene.usda", "tactile.npz", "manifest.json"}


def test_renderer_rejects_changed_capture_before_launch(tmp_path, monkeypatch):
    renderer = load_script("render_scene.py")
    (tmp_path / "scene.usda").write_text("changed")
    (tmp_path / "manifest.json").write_text(json.dumps({"scene_sha256": "0" * 64}))
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "render_scene.py",
            "--state",
            str(tmp_path),
            "--output",
            str(tmp_path / "render"),
        ],
    )
    with pytest.raises(RuntimeError, match="hash differs"):
        renderer.main()


@pytest.mark.parametrize("failure_mode", ["missing", "timeline", "body", "corrupt", "process", "none"])
def test_camera_supervisor_requires_verified_images(tmp_path, monkeypatch, failure_mode):
    from PIL import Image

    renderer = load_script("render_scene.py")
    capture = {"tactile_sha256": "physical-frame", "scene_sha256": "physical-scene"}

    def worker(command, check):
        staging = Path(command[command.index("--output") + 1])
        if failure_mode == "missing":
            return subprocess.CompletedProcess(command, 0)
        names = ("hand-camera.png", "loaded-contact.png", "loaded-contact.webp", "dex01-demo.png", "dex01-demo.webp")
        for name in names:
            Image.new("RGB", (16, 9), "navy").save(staging / name)
        result = {
            **capture,
            "timeline_advanced": failure_mode == "timeline",
            "fingertip_transforms_unchanged": True,
            "rigid_body_transforms_unchanged": failure_mode != "body",
            "rendered_sensor_visuals": 5,
            "camera_resolution": [16, 9],
            "rendered_files": {name: hashlib.sha256((staging / name).read_bytes()).hexdigest() for name in names},
        }
        (staging / "render-manifest.json").write_text(json.dumps(result))
        if failure_mode == "corrupt":
            (staging / "hand-camera.png").write_bytes(b"wrong image")
        return subprocess.CompletedProcess(command, 1 if failure_mode == "process" else 0)

    monkeypatch.setattr(renderer.subprocess, "run", worker)
    output = tmp_path / "rendered"
    if failure_mode == "none":
        renderer.render_checked(tmp_path / "state", output, capture)
        assert {p.name for p in output.iterdir()} == {
            "hand-camera.png",
            "loaded-contact.png",
            "loaded-contact.webp",
            "dex01-demo.png",
            "dex01-demo.webp",
            "render-manifest.json",
        }
    else:
        with pytest.raises(RuntimeError):
            renderer.render_checked(tmp_path / "state", output, capture)
        assert not output.exists()
