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

"""Check firmware footprint preservation and pressing/release report failures."""

import importlib.util
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]


def load_module(path):
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_viewport_preserves_occupied_cells_and_contact_footprint():
    module = load_module(ROOT / "source/dex01_sim_asset/dex01_sim_asset/vis_utils/viewport.py")
    with np.load(ROOT / "source/dex01_sim_asset/dex01_sim_asset/taxel_patterns/dex01_pattern.npz") as pattern:
        pixels = pattern["taxel2pixel"]
    mask = np.zeros((32, 32), dtype=bool)
    mask[pixels[:, 0], pixels[:, 1]] = True
    rows, cols = np.indices((32, 32))
    # A ring footprint checks holes, boundaries and row/column orientation.
    radius = np.hypot(rows - 21, cols - 12)
    contact = mask & (radius > 3) & (radius < 8) & (cols < 17)
    frame = contact.astype(float) * 0.05
    rgba = module.force_frame_rgba(frame, pixels, 0.05)
    assert rgba.shape == (32, 32, 4) and rgba.dtype == np.uint8
    assert np.array_equal((rgba[..., :3] == (255, 30, 0)).all(-1), contact)
    assert np.array_equal((rgba[..., :3] == (65, 65, 65)).all(-1), ~mask)
    assert (rgba[..., 3] == 255).all()
    with pytest.raises(ValueError):
        module.force_frame_rgba(frame, pixels, 0)
    frame[0, 0] = np.nan
    with pytest.raises(ValueError):
        module.force_frame_rgba(frame, pixels, 0.05)


def articulation_fixture():
    module = load_module(ROOT / "scripts/dex01_articulated_press.py")
    observations = []
    for step in range(300):
        t = step / 30
        fraction, phase = module.press_motion(t)
        load = 2.0 if phase == "hold" else 0.0
        observations.append({
            "step": step,
            "phase": phase,
            "elapsed_wall_s": (step + 1) / 30,
            "joint_positions_rad": [fraction * 0.45, fraction * 0.35, fraction * 0.15],
            "all_joint_positions_rad": [fraction * 0.45, fraction * 0.35, fraction * 0.15, 0],
            "wrist_pose": [0, 0, 0, 1, 0, 0, 0],
            "object_pose": [0, 0, 0.1, 1, 0, 0, 0],
            "tip_position_m": [0, 0, fraction * 0.07],
            "forces_n": [0, load, 0, 0, 0],
            "min_depth_m": -0.0001 if load else 0,
            "contact_force_w_n": [0, 0, -load],
            "penetration_normal_w": [0, 0, 1],
            "frame_n": [[load]],
            "taxel_forces_n": [load],
        })
    return module, {
        "mode": "fixed-wrist-articulated-press",
        "all_joint_names": ["a", "b", "c", "uncommanded"],
        "actuated_joint_indices": [0, 1, 2],
        "fixed_base": True,
        "physics_dt_s": 1 / 120,
        "trajectory_time_scale": 0.25,
        "observation_stride": 1,
        "taxel2pixel": [[0, 0]],
        "observations": observations,
    }


def test_fixed_wrist_articulation_report_passes_complete_cycle():
    module, report = articulation_fixture()
    assert module.check_articulated_report(report) == pytest.approx(2.0)


@pytest.mark.parametrize(
    "field,value,error",
    [
        ("wrist_pose", [0.01, 0, 0, 1, 0, 0, 0], "Wrist translated"),
        ("wrist_pose", [0, 0, 0, np.sqrt(0.5), 0, np.sqrt(0.5), 0], "Wrist rotated"),
        ("object_pose", [0, 0, 0.2, 1, 0, 0, 0], "Object translated"),
        ("joint_positions_rad", [0, 0, 0], "did not articulate"),
        ("tip_position_m", [0, 0, 0], "did not move"),
        ("min_depth_m", 0, "penetration"),
        ("contact_force_w_n", [0, 0, 0], "Contact force"),
        ("frame_n", [[0]], "mapping"),
    ],
)
def test_articulation_audit_rejects_false_motion_and_contact(field, value, error):
    module, report = articulation_fixture()
    for e in report["observations"]:
        if e["phase"] == "hold":
            e[field] = value
    if field in {"joint_positions_rad", "tip_position_m"}:
        for e in report["observations"]:
            e[field] = value
            if field == "joint_positions_rad":
                e["all_joint_positions_rad"] = value + [0]
    with pytest.raises(ValueError, match=error):
        module.check_articulated_report(report)


def test_articulation_audit_rejects_late_release_and_slow_cycle():
    module, report = articulation_fixture()
    report["observations"][-1]["forces_n"][1] = 2
    report["observations"][-1]["frame_n"] = [[2]]
    report["observations"][-1]["taxel_forces_n"] = [2]
    report["observations"][-1]["contact_force_w_n"] = [0, 0, -2]
    with pytest.raises(ValueError, match="release"):
        module.check_articulated_report(report)
    module, report = articulation_fixture()
    for e in report["observations"]:
        e["elapsed_wall_s"] *= 3
    with pytest.raises(ValueError, match="twelve"):
        module.check_articulated_report(report)


def test_runtime_drives_joint_targets_without_pose_teleportation():
    import ast

    tree = ast.parse((ROOT / "scripts/dex01_articulated_press.py").read_text())
    loop = next(node for node in ast.walk(tree) if isinstance(node, ast.While))
    calls = {
        node.func.attr for node in ast.walk(loop) if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
    }
    assert "set_joint_position_target" in calls
    assert not calls & {"write_joint_state_to_sim", "write_root_pose_to_sim", "write_root_state_to_sim"}


def test_all_joint_audit_rejects_uncommanded_finger_motion():
    module, report = articulation_fixture()
    report.update(all_joint_names=["a", "b", "c", "uncommanded"], actuated_joint_indices=[0, 1, 2], fixed_base=True)
    for entry in report["observations"]:
        entry["all_joint_positions_rad"] = entry["joint_positions_rad"] + [0.1 if entry["phase"] == "hold" else 0]
    with pytest.raises(ValueError, match="Uncommanded"):
        module.check_articulated_report(report)


def multi_cycle_fixture():
    import copy

    module, report = articulation_fixture()
    observations = []
    for cycle in range(3):
        for entry in report["observations"][::3]:
            entry = copy.deepcopy(entry)
            entry["step"] += cycle * 300
            entry["elapsed_wall_s"] += cycle * 10
            observations.append(entry)
    report.update(observations=observations, observation_stride=3)
    return module, report


def test_stride_three_report_checks_final_completed_cycle():
    module, report = multi_cycle_fixture()
    assert report["observations"][-1]["step"] == 897
    assert module.check_articulated_report(report) == pytest.approx(2.0)
    # Failure exists only in the last observation of the third cycle.
    entry = report["observations"][-1]
    entry.update(forces_n=[0, 2, 0, 0, 0], frame_n=[[2]], taxel_forces_n=[2], contact_force_w_n=[0, 0, -2])
    with pytest.raises(ValueError, match="release"):
        module.check_articulated_report(report)


@pytest.mark.parametrize(
    "field,value,error",
    [
        ("wrist_pose", [0.01, 0, 0, 1, 0, 0, 0], "Wrist translated"),
        ("object_pose", [0, 0, 0.2, 1, 0, 0, 0], "Object translated"),
        ("wrist_pose", [0, 0, 0, np.sqrt(0.5), 0, np.sqrt(0.5), 0], "Wrist rotated"),
        ("object_pose", [0, 0, 0.1, np.sqrt(0.5), 0, np.sqrt(0.5), 0], "Object rotated"),
        ("uncommanded", 0.1, "Uncommanded"),
    ],
)
def test_report_checks_invariance_across_cycle_boundaries(field, value, error):
    module, report = multi_cycle_fixture()
    report.update(all_joint_names=["a", "b", "c", "uncommanded"], actuated_joint_indices=[0, 1, 2], fixed_base=True)
    for entry in report["observations"]:
        entry["all_joint_positions_rad"] = entry["joint_positions_rad"] + [0]
        if entry["step"] >= 300:
            if field == "uncommanded":
                entry["all_joint_positions_rad"][-1] = value
            else:
                entry[field] = value
    with pytest.raises(ValueError, match=error):
        module.check_articulated_report(report)


def test_local_plot_receives_current_sensor_frames():
    from types import SimpleNamespace

    import torch

    module = load_module(ROOT / "scripts/dex01_articulated_press.py")
    current = torch.tensor([[[0.0, 1.5], [2.0, 0.0]]])
    sensors = [SimpleNamespace(get_tactile_image=lambda: current)]
    displayed = []
    viewer = SimpleNamespace(update=lambda images: displayed.append(images))
    module.update_local_viewer(sensors, viewer)
    assert np.array_equal(displayed[-1][0], current[0].numpy())
    current[0, 0, 1] = 3.0
    module.update_local_viewer(sensors, viewer)
    assert displayed[-1][0][0, 1] == 3.0
    module.update_local_viewer(sensors, None)
    assert len(displayed) == 2


def test_setup_failure_replaces_stale_success_report(tmp_path):
    import json

    module = load_module(ROOT / "scripts/dex01_articulated_press.py")
    path = tmp_path / "report.json"
    path.write_text(json.dumps({"failure": None, "observations": ["stale success"]}))

    def fail_setup():
        assert not path.exists()
        raise RuntimeError("setup failed")

    with pytest.raises(RuntimeError, match="setup failed"):
        module.run_with_report(fail_setup, path)
    assert json.loads(path.read_text()) == {
        "mode": "fixed-wrist-articulated-press",
        "failure": "setup failed",
        "observations": [],
    }
    with pytest.raises(ValueError, match="Demo failed"):
        module.check_articulated_report(json.loads(path.read_text()))


def test_current_run_detailed_failure_report_is_preserved(tmp_path):
    module = load_module(ROOT / "scripts/dex01_articulated_press.py")
    path = tmp_path / "report.json"
    details = '{"failure": "contact failed", "observations": [{"step": 42}]}'

    def fail_contact():
        path.write_text(details)
        raise RuntimeError("contact failed")

    with pytest.raises(RuntimeError, match="contact failed"):
        module.run_with_report(fail_contact, path)
    assert path.read_text() == details


@pytest.mark.parametrize("scale", [0.1, 0.2, 0.251, 0.3, 0.333, 0.5, 0.75, 1.0])
def test_non_aligned_cycle_preserves_hold_and_retract_boundaries(scale):
    import copy

    module, report = articulation_fixture()
    observations = []
    for step in range(0, int(30 * scale / report["physics_dt_s"]) + 1, 3):
        trajectory_time = step * report["physics_dt_s"] / scale
        fraction, phase = module.press_motion(trajectory_time)
        entry = copy.deepcopy(report["observations"][0])
        load = 2.0 if phase == "hold" else 0.0
        entry.update(
            step=step,
            phase=phase,
            elapsed_wall_s=trajectory_time + 0.01,
            joint_positions_rad=[fraction * 0.45, fraction * 0.35, fraction * 0.15],
            all_joint_positions_rad=[fraction * 0.45, fraction * 0.35, fraction * 0.15, 0],
            tip_position_m=[0, 0, fraction * 0.07],
            forces_n=[0, load, 0, 0, 0],
            min_depth_m=-0.0001 if load else 0,
            contact_force_w_n=[0, 0, -load],
            frame_n=[[load]],
            taxel_forces_n=[load],
        )
        observations.append(entry)
    report.update(observations=observations, trajectory_time_scale=scale, observation_stride=3)
    if scale == 0.251:
        assert next(e for e in observations if e["step"] == 483)["phase"] == "retract"
    if scale == 0.2:
        assert next(e for e in observations if e["step"] == 384)["phase"] == "retract"
    assert module.check_articulated_report(report) == pytest.approx(2.0)

    entry = next(e for e in observations if 15 < e["step"] * report["physics_dt_s"] / scale < 15.5)
    assert entry["phase"] == "hold"
    entry.update(
        forces_n=[0, 0, 0, 0, 0], min_depth_m=0, contact_force_w_n=[0, 0, 0], frame_n=[[0]], taxel_forces_n=[0]
    )
    with pytest.raises(ValueError, match="sustained"):
        module.check_articulated_report(report)


def test_multi_cycle_report_preserves_first_cycle_startup_time():
    module, report = multi_cycle_fixture()
    for entry in report["observations"]:
        entry["elapsed_wall_s"] += 1
    with pytest.raises(ValueError, match="Motion begins too late"):
        module.check_articulated_report(report)


@pytest.mark.parametrize(
    "field,value,error",
    [
        ("forces_n", [0, np.nan, 0, 0, 0], "Invalid forces_n"),
        ("joint_positions_rad", [0, np.nan, 0], "Invalid joint_positions_rad"),
        ("frame_n", [[1]], "Firmware mapping"),
        ("min_depth_m", 0.001, "positive penetration"),
        ("elapsed_wall_s", -1, "wall-clock"),
    ],
)
def test_incomplete_tail_cannot_hide_invalid_observations(field, value, error):
    import copy

    module, report = multi_cycle_fixture()
    tail = copy.deepcopy(report["observations"][0])
    tail.update(step=900, elapsed_wall_s=30.01)
    tail[field] = value
    report["observations"].append(tail)
    with pytest.raises(ValueError, match=error):
        module.check_articulated_report(report)


@pytest.mark.parametrize("corruption", ["duplicate", "missing", "out_of_order", "non_finite_wall"])
def test_observation_timeline_integrity(corruption):
    module, report = multi_cycle_fixture()
    if corruption == "duplicate":
        report["observations"][40]["step"] = report["observations"][39]["step"]
    elif corruption == "missing":
        del report["observations"][40]
    elif corruption == "out_of_order":
        report["observations"][40], report["observations"][41] = report["observations"][41], report["observations"][40]
    else:
        report["observations"][40]["elapsed_wall_s"] = np.nan
    with pytest.raises(ValueError):
        module.check_articulated_report(report)


@pytest.mark.parametrize(
    "script,args",
    [
        ("dex01_stationary_press.py", []),
        ("dex01_on_tesollo_dg5f.py", ["--stationary-press"]),
    ],
)
def test_cli_startup_failure_invalidates_previous_report(tmp_path, script, args):
    import json
    import subprocess
    import sys

    path = tmp_path / "report.json"
    path.write_text(json.dumps({"failure": None, "observations": ["stale success"]}))
    # Disable site packages to reproduce a real failure before AppLauncher.
    result = subprocess.run(
        [sys.executable, "-S", str(ROOT / "scripts" / script), *args, "--verification-report", str(path)],
        capture_output=True,
        text=True,
    )
    assert result.returncode != 0
    assert "isaaclab" in result.stderr
    failure = json.loads(path.read_text())
    assert "isaaclab" in failure["failure"]
    assert failure["observations"] == []


def test_boundary_normalization_does_not_move_real_nearby_times():
    module = load_module(ROOT / "scripts/dex01_articulated_press.py")
    for boundary in (1, 4, 6, 9, 10):
        assert module.cycle_time(boundary - 1e-12) == boundary % 10
        assert module.cycle_time(boundary - 1e-6) == pytest.approx(boundary - 1e-6)
        assert module.cycle_time(boundary + 1e-6) == pytest.approx((boundary + 1e-6) % 10)


@pytest.mark.parametrize("field", ["all_joint_names", "actuated_joint_indices", "fixed_base"])
def test_required_articulation_metadata(field):
    module, report = articulation_fixture()
    del report[field]
    with pytest.raises(ValueError, match=f"Missing {field}"):
        module.check_articulated_report(report)


@pytest.mark.parametrize("index", [0, 150, 299])
def test_required_joint_data_in_every_observation(index):
    module, report = articulation_fixture()
    del report["observations"][index]["all_joint_positions_rad"]
    report["fixed_base"] = False
    with pytest.raises(ValueError, match="Missing all_joint_positions_rad"):
        module.check_articulated_report(report)


def test_report_rejects_floating_base():
    module, report = articulation_fixture()
    report["fixed_base"] = False
    with pytest.raises(ValueError, match="fixed-base"):
        module.check_articulated_report(report)


def test_success_report_is_invalidated_if_shutdown_fails(tmp_path):
    import json

    module, report = articulation_fixture()
    path = tmp_path / "report.json"

    def run_then_fail_shutdown():
        path.write_text(json.dumps(report))
        raise RuntimeError("shutdown failed")

    with pytest.raises(RuntimeError, match="shutdown failed"):
        module.run_with_report(run_then_fail_shutdown, path)
    result = json.loads(path.read_text())
    assert result["failure"] == "shutdown failed"
    assert result["observations"] == report["observations"]
    with pytest.raises(ValueError, match="Demo failed"):
        module.check_articulated_report(result)


def test_report_parent_exists_before_run_and_preserves_measurements(tmp_path):
    import json

    module, report = articulation_fixture()
    path = tmp_path / "new" / "results" / "press.json"

    def run():
        assert path.parent.is_dir()
        path.write_text(json.dumps(report))

    module.run_with_report(run, path)
    result = json.loads(path.read_text())
    assert result["observations"] == report["observations"]
    assert module.check_articulated_report(result) == pytest.approx(2)


@pytest.mark.parametrize("fixed_base", [False, "false", 1, None])
def test_fixed_base_requires_true_boolean(fixed_base):
    module, report = articulation_fixture()
    report["fixed_base"] = fixed_base
    with pytest.raises(ValueError, match="fixed-base"):
        module.check_articulated_report(report)


@pytest.mark.parametrize("exception", [RuntimeError(), AssertionError()])
def test_empty_exception_message_cannot_leave_success_report(tmp_path, exception):
    import json

    module, report = articulation_fixture()
    path = tmp_path / "report.json"

    def fail():
        path.write_text(json.dumps(report))
        raise exception

    with pytest.raises(type(exception)):
        module.run_with_report(fail, path)
    result = json.loads(path.read_text())
    assert result["failure"] == type(exception).__name__
    assert result["observations"] == report["observations"]
    with pytest.raises(ValueError, match="Demo failed"):
        module.check_articulated_report(result)


@pytest.mark.parametrize(
    "script,args",
    [
        ("dex01_stationary_press.py", []),
        ("dex01_on_tesollo_dg5f.py", ["--stationary-press"]),
    ],
)
@pytest.mark.parametrize("help_flag", ["--help", "-h"])
def test_cli_help_preserves_existing_report(tmp_path, script, args, help_flag):
    import os
    import subprocess
    import sys

    # CI deliberately has no Isaac Lab installation. Only its argparse hook
    # is needed here; help exits before AppLauncher is constructed.
    package = tmp_path / "isaaclab"
    package.mkdir()
    (package / "__init__.py").write_text("")
    (package / "app.py").write_text(
        "class AppLauncher:\n"
        "    @staticmethod\n"
        "    def add_app_launcher_args(parser): pass\n"
        "    def __init__(self, *args): raise AssertionError('help launched simulator')\n"
    )
    env = dict(os.environ, PYTHONPATH=str(tmp_path))
    path = tmp_path / "report.json"
    original = '{"failure": null, "observations": ["keep"]}'
    path.write_text(original)
    result = subprocess.run(
        [sys.executable, str(ROOT / "scripts" / script), *args, help_flag, "--verification-report", str(path)],
        capture_output=True,
        text=True,
        env=env,
    )
    assert result.returncode == 0, result.stderr
    assert "usage:" in result.stdout
    assert path.read_text() == original


def test_ctrl_c_requests_graceful_stop_without_sdk_handler():
    import signal

    module = load_module(ROOT / "scripts/dex01_articulated_press.py")
    previous = signal.getsignal(signal.SIGINT)
    with module.graceful_stop() as stop:
        assert not stop.is_set()
        signal.raise_signal(signal.SIGINT)
        assert stop.is_set()
        # The caller can still persist results before its simulator is closed.
    assert signal.getsignal(signal.SIGINT) == previous


def test_graceful_stop_restores_handler_on_error():
    import signal

    module = load_module(ROOT / "scripts/dex01_articulated_press.py")
    previous = signal.getsignal(signal.SIGINT)
    with pytest.raises(RuntimeError):
        with module.graceful_stop():
            raise RuntimeError("test")
    assert signal.getsignal(signal.SIGINT) == previous


@pytest.mark.parametrize("exception", [SystemExit(2), KeyboardInterrupt()])
def test_abnormal_exit_cannot_leave_success_report(tmp_path, exception):
    import json

    module, report = articulation_fixture()
    path = tmp_path / "report.json"

    def fail():
        path.write_text(json.dumps(report))
        raise exception

    with pytest.raises(type(exception)):
        module.run_with_report(fail, path)
    result = json.loads(path.read_text())
    assert result["failure"]
    assert result["observations"] == report["observations"]
    with pytest.raises(ValueError, match="Demo failed"):
        module.check_articulated_report(result)
