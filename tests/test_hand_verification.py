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

"""Reject invalid reports before any settling filter can hide a bad observation."""

import ast
import copy
import json
import subprocess
import sys
from pathlib import Path
from types import ModuleType

import pytest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/verify_dex01_sim.py"


def checker_module():
    tree = ast.parse(SCRIPT.read_text())
    first_parser = next(
        i
        for i, node in enumerate(tree.body)
        if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == "parser" for t in node.targets)
    )
    module = ModuleType("_hand_checker")
    module.__file__ = str(SCRIPT)
    exec(compile(ast.Module(body=tree.body[:first_parser], type_ignores=[]), str(SCRIPT), "exec"), module.__dict__)
    return module


@pytest.fixture
def report():
    observations = []
    for step in range(800):
        finger = step // 200 + 2
        forces = [0.0] * 5
        contact = [[0.0] * 3 for _ in range(5)]
        depths = [0.0] * 5
        forces[finger - 1] = 0.981
        contact[finger - 1][2] = -0.981
        depths[finger - 1] = -0.0001
        observations.append(
            dict(
                step=step,
                target_finger=finger,
                forces_n=forces,
                min_forces_n=[0.0] * 5,
                depth_min_m=depths,
                contact_forces_w_n=contact,
                gear_velocity_w=[0.0] * 6,
                gear_position_m=[0.0] * 3,
                sensor_positions_m=[[0.0] * 3 for _ in range(5)],
            )
        )
    return dict(
        steps_per_finger=200,
        physics_dt_s=1 / 240,
        indenter="flat",
        taxels_per_finger=[738] * 5,
        observations=observations,
        thumb=dict(probe_taxel_force_n=1.0, probe_centroid_error_m=0.0001, gear_filtered_force_n=0.0),
    )


def test_complete_report_passes(tmp_path, report):
    path = tmp_path / "report.json"
    path.write_text(json.dumps(report))
    checker_module().check_hand_report(path)


@pytest.mark.parametrize(
    "corruption",
    [
        "negative_transient",
        "nan_transient",
        "other_finger",
        "missing_step",
        "wrong_target",
        "failed_run",
        "negative_thumb",
        "zero_dt",
    ],
)
def test_bad_observation_cannot_hide_outside_settling_window(tmp_path, report, corruption):
    data = copy.deepcopy(report)
    first = data["observations"][0]
    if corruption == "negative_transient":
        first["forces_n"][1] = -1.0
    elif corruption == "nan_transient":
        first["contact_forces_w_n"][1][0] = float("nan")
    elif corruption == "other_finger":
        first["forces_n"][2] = 1.0
    elif corruption == "missing_step":
        first["step"] = 1
    elif corruption == "wrong_target":
        first["target_finger"] = 4
    elif corruption == "failed_run":
        data["failure"] = "shutdown failed"
    elif corruption == "negative_thumb":
        data["thumb"]["probe_centroid_error_m"] = -1.0
    else:
        data["physics_dt_s"] = 0.0
    path = tmp_path / "report.json"
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError):
        checker_module().check_hand_report(path)


def test_checks_remain_active_with_python_optimization(tmp_path, report):
    report["thumb"]["probe_taxel_force_n"] = 0.0
    path = tmp_path / "bad.json"
    path.write_text(json.dumps(report))
    result = subprocess.run([sys.executable, "-O", str(SCRIPT), "--check-hand-report", str(path)], capture_output=True)
    assert result.returncode != 0
    assert b"ValueError" in result.stderr


def test_zero_exit_without_report_is_failure(tmp_path, monkeypatch):
    module = checker_module()
    path = tmp_path / "report.json"
    path.write_text('{"status": "passed"}')
    monkeypatch.setattr(module.subprocess, "run", lambda *a, **k: subprocess.CompletedProcess(a, 0))
    with pytest.raises(ValueError, match="no report"):
        module.run_checked([], path)
    assert not path.exists()
