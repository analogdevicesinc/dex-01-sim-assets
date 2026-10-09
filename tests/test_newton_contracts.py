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

"""Backend-neutral contract checks; live Newton validation is required separately."""

import importlib.util
from pathlib import Path

import numpy as np
import pytest
import torch

ROOT = Path(__file__).resolve().parents[1]


def load(name):
    path = ROOT / "source/dex01_sim_asset/dex01_sim_asset" / name
    spec = importlib.util.spec_from_file_location("_contract", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_normal_load_conservation_and_no_patch():
    m = load("measurement.py")
    depths = torch.tensor([[-0.001, -0.003], [0.0, 0.0]])
    normals = torch.tensor([[[0.0, 0.0, 1.0], [0.0, 0.0, 1.0]]] * 2)
    force = torch.tensor([[0.0, 0.0, -8.0], [0.0, 0.0, -8.0]])
    loads, n = m.distribute_normal_load(depths, normals, force)
    assert torch.equal(loads, torch.tensor([[2.0, 6.0], [0.0, 0.0]]))
    assert torch.equal(n[0], torch.tensor([0.0, 0.0, 1.0]))
    assert torch.equal(n[1], torch.zeros(3))
    assert torch.equal(
        m.tactile_image(loads, torch.tensor([[0, 1], [1, 0]])),
        torch.tensor([[[0.0, 2.0], [6.0, 0.0]], [[0.0, 0.0], [0.0, 0.0]]]),
    )


def test_curved_normal_projection_rejects_outward_force():
    m = load("measurement.py")
    normals = torch.tensor([[[1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]])
    depths = torch.tensor([[-0.001, -0.001]])
    loads, n = m.distribute_normal_load(depths, normals, torch.tensor([[-2.0, 0.0, -2.0]]))
    assert loads.sum() == pytest.approx(2 * np.sqrt(2))
    loads, _ = m.distribute_normal_load(depths, normals, torch.tensor([[2.0, 0.0, 2.0]]))
    assert loads.sum() == 0


def test_report_checker_rejects_missing_ticks_and_fake_geometry():
    verification = load("newton/verification.py")
    package = ROOT / "source/dex01_sim_asset/dex01_sim_asset/taxel_patterns/dex01_contact_v10.npz"
    pixels = np.load(package)["taxel2pixel"]
    report = {
        "backend": "newton",
        "mode": "hand",
        "indenter": "flat",
        "physics_dt_s": 1 / 240,
        "taxels_per_sensor": [738] * 5,
        "taxel2pixel": pixels.tolist(),
        "observations": [{
            "step": 1,
            "elapsed_wall_s": 0.1,
            "forces_n": [0] * 5,
            "frame_n": np.zeros((32, 32)).tolist(),
            "taxel_forces_n": [0] * 738,
            "target_finger": 2,
            "normal_force_w_n": [0, 0, 0],
            "penetration_normal_w": [0, 0, 0],
        }],
    }
    with pytest.raises(ValueError, match="every physics tick"):
        verification.check_report(report)
    report["observations"][0]["step"] = 0
    report["taxel2pixel"][0] = [40, 0]
    with pytest.raises(ValueError, match="firmware mapping"):
        verification.check_report(report)


@pytest.mark.parametrize("failure", ["shutdown failed", "setup failed"])
def test_failed_report_is_never_accepted(failure):
    with pytest.raises(ValueError, match="Demo failed"):
        load("newton/verification.py").check_report({"failure": failure})


@pytest.mark.parametrize("fixed_base", [False, "false", 1, None])
def test_fixed_base_is_strict_boolean(fixed_base):
    with pytest.raises(ValueError, match="wrist is not fixed"):
        load("newton/verification.py").check_press({"fixed_base": fixed_base}, [], np.zeros((0, 5)))


@pytest.mark.parametrize(
    "exception", [RuntimeError("shutdown failed"), AssertionError(), KeyboardInterrupt(), SystemExit(2)]
)
def test_current_measurements_survive_failure_but_success_is_invalidated(tmp_path, exception):
    import json

    lifecycle = load("newton/lifecycle.py")
    report = tmp_path / "nested/report.json"

    def run():
        assert report.parent.is_dir() and not report.exists()
        report.write_text(json.dumps({"backend": "newton", "observations": [{"step": 12}], "failure": None}))
        raise exception

    with pytest.raises(type(exception)):
        lifecycle.run_with_report(run, report, "press")
    result = json.loads(report.read_text())
    assert result["observations"] == [{"step": 12}]
    assert result["failure"]


def test_cli_startup_failure_invalidates_old_report(tmp_path):
    import json
    import subprocess
    import sys

    path = tmp_path / "report.json"
    path.write_text('{"failure":null,"observations":["old success"]}')
    result = subprocess.run(
        [sys.executable, "-S", str(ROOT / "scripts/dex01_newton.py"), "--report", str(path)],
        capture_output=True,
        text=True,
    )
    assert result.returncode != 0
    report = json.loads(path.read_text())
    assert report["failure"] and report["observations"] == []


@pytest.mark.parametrize("flag", ["-h", "--help"])
def test_newton_help_preserves_report_and_never_launches(tmp_path, flag):
    import subprocess
    import sys

    path = tmp_path / "report.json"
    original = '{"observations":["keep"]}'
    path.write_text(original)
    result = subprocess.run(
        [sys.executable, "-S", str(ROOT / "scripts/dex01_newton.py"), flag, "--report", str(path)],
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0 and "usage:" in result.stdout
    assert path.read_text() == original


def test_newton_ctrl_c_restores_handler_and_allows_persisting(tmp_path):
    import signal

    lifecycle = load("newton/lifecycle.py")
    previous = signal.getsignal(signal.SIGINT)
    with lifecycle.graceful_stop() as stop:
        signal.raise_signal(signal.SIGINT)
        assert stop.is_set()
        (tmp_path / "report").write_text("current measurements")
    assert signal.getsignal(signal.SIGINT) == previous
    with pytest.raises(RuntimeError), lifecycle.graceful_stop():
        raise RuntimeError("failed")
    assert signal.getsignal(signal.SIGINT) == previous


@pytest.fixture
def valid_press_report():
    pixels = np.load(ROOT / "source/dex01_sim_asset/dex01_sim_asset/taxel_patterns/dex01_contact_v10.npz")[
        "taxel2pixel"
    ]
    loaded = np.zeros((32, 32))
    loaded[tuple(pixels[0])] = 2.0
    loaded_taxels = [2.0] + [0.0] * 737
    zero_frame = np.zeros((32, 32)).tolist()
    entries = []
    for step in range(1800):
        t = step / 240 % 2.5
        fraction = (
            0 if t < 0.25 else min(1, (t - 0.25) / 0.75) if t < 1 else 1 if t < 1.5 else max(0, 1 - (t - 1.5) / 0.65)
        )
        force = 2.0 if 1.0 <= t < 1.5 else 0.0
        entries.append({
            "step": step,
            "elapsed_wall_s": (step + 1) / 240,
            "forces_n": [0, force, 0, 0, 0],
            "target_finger": 2,
            "taxel_forces_n": loaded_taxels if force else [0.0] * 738,
            "frame_n": loaded.tolist() if force else zero_frame,
            "normal_force_w_n": [0, 0, -force],
            "penetration_normal_w": [0, 0, 1] if force else [0, 0, 0],
            "probe_pose": [0, 0, 0.1, 0, 0, 0, 1],
            "probe_velocity_w": [0] * 6,
            "wrist_pose": [0, 0, 0, 0, 0, 0, 1],
            "joints_rad": [fraction * 0.45, fraction * 0.35, fraction * 0.15, 0],
            "tip_position_m": [0, 0, 0.07 * fraction],
            "min_depth_m": -0.0001 if force else 0,
        })
    return {
        "backend": "newton",
        "mode": "press",
        "indenter": "gear",
        "failure": None,
        "physics_dt_s": 1 / 240,
        "taxels_per_sensor": [738] * 5,
        "taxel2pixel": pixels.tolist(),
        "fixed_base": True,
        "joint_ids": [0, 1, 2],
        "observations": entries,
    }


def test_newton_checker_accepts_complete_three_cycles(valid_press_report):
    assert load("newton/verification.py").check_report(valid_press_report)


@pytest.mark.parametrize(
    "corruption,error",
    [
        ("last_release", "Failed release"),
        ("cycle_wrist", "Wrist moved"),
        ("cycle_object", "indenter moved"),
        ("uncommanded", "Uncommanded"),
        ("duplicate", "every physics tick"),
        ("missing", "every physics tick"),
        ("unordered", "wall-clock"),
        ("wall_nan", "wall-clock"),
        ("negative_frame", "Firmware mapping"),
        ("tail_nan", "Invalid state"),
    ],
)
def test_newton_report_cannot_hide_bad_cycles_or_tail(valid_press_report, corruption, error):
    r = valid_press_report
    entries = r["observations"]
    if corruption == "last_release":
        e = entries[-1]
        e.update(
            forces_n=[0, 2, 0, 0, 0],
            normal_force_w_n=[0, 0, -2],
            penetration_normal_w=[0, 0, 1],
            frame_n=entries[250]["frame_n"],
            taxel_forces_n=entries[250]["taxel_forces_n"],
        )
    elif corruption in ("cycle_wrist", "cycle_object"):
        field = "wrist_pose" if corruption == "cycle_wrist" else "probe_pose"
        for e in entries[600:]:
            e[field] = [0.01, 0, 0, 0, 0, 0, 1]
    elif corruption == "uncommanded":
        entries[700]["joints_rad"][-1] = 0.1
    elif corruption == "duplicate":
        entries[700]["step"] = 699
    elif corruption == "missing":
        del entries[700]
    elif corruption == "unordered":
        entries[700], entries[701] = entries[701], entries[700]
    elif corruption == "wall_nan":
        entries[700]["elapsed_wall_s"] = float("nan")
    elif corruption == "negative_frame":
        entries[700]["frame_n"] = (-np.ones((32, 32))).tolist()
    else:
        tail = dict(entries[0], step=1800, elapsed_wall_s=8.0, tip_position_m=[float("nan"), 0, 0])
        entries.append(tail)
    with pytest.raises(ValueError, match=error):
        load("newton/verification.py").check_report(r)
