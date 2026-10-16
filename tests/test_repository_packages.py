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

"""Build real wheels and enforce backend isolation and shared asset identity."""

import ast
import hashlib
import os
import subprocess
import sys
import zipfile
from email.parser import Parser
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
PACKAGES = {
    "common": ("common/source/dex01_sim_common", "dex01_sim_common"),
    "physx": ("physx/source/dex01_sim_asset", "dex01_sim_asset"),
    "newton": ("newton/source/dex01_sim_newton", "dex01_sim_newton"),
}


@pytest.fixture(scope="module")
def wheels(tmp_path_factory):
    destination = tmp_path_factory.mktemp("package-wheels")
    env = {**os.environ, "PIP_DISABLE_PIP_VERSION_CHECK": "1"}
    for source, _ in PACKAGES.values():
        result = subprocess.run(
            [
                sys.executable,
                "-m",
                "build",
                "--wheel",
                "--no-isolation",
                "--outdir",
                str(destination),
                str(ROOT / source),
            ],
            env=env,
            capture_output=True,
            text=True,
            timeout=120,
        )
        assert result.returncode == 0, result.stdout + result.stderr
    return {name: next(destination.glob(namespace + "-*.whl")) for name, (_, namespace) in PACKAGES.items()}


@pytest.mark.parametrize("environment", PACKAGES)
def test_wheel_contains_only_its_package_and_canonical_licenses(wheels, environment):
    _, namespace = PACKAGES[environment]
    with zipfile.ZipFile(wheels[environment]) as wheel:
        modules = [name for name in wheel.namelist() if name.endswith(".py")]
        assert modules and all(name.startswith(namespace + "/") for name in modules)
        metadata_file = next(name for name in wheel.namelist() if name.endswith("/METADATA"))
        metadata = Parser().parsestr(wheel.read(metadata_file).decode())
        dependencies = metadata.get_all("Requires-Dist", [])
        if environment != "common":
            assert dependencies == ["dex01-sim-common==0.1.0a1"]
            assert not any(name.endswith(".npz") for name in wheel.namelist())
        for filename in ("LICENSE", "NOTICE", "LICENSE_Tesollo"):
            archived = next(name for name in wheel.namelist() if name.endswith("/licenses/" + filename))
            assert wheel.read(archived) == (ROOT / filename).read_bytes()


def test_common_wheel_contains_identical_reference_taxel_files(wheels):
    import json

    reference = json.loads((ROOT / "assets/sensors/dex01/reference.json").read_text())
    with zipfile.ZipFile(wheels["common"]) as wheel:
        for field in ("pattern", "contact_pattern"):
            file = Path(reference[field + "_path"]).name
            data = wheel.read("dex01_sim_common/taxel_patterns/" + file)
            assert hashlib.sha256(data).hexdigest() == reference[field + "_sha256"]


@pytest.mark.parametrize("environment", PACKAGES)
def test_built_package_can_load_its_namespace_without_initializing_simulation(wheels, environment, tmp_path):
    _, namespace = PACKAGES[environment]
    with zipfile.ZipFile(wheels[environment]) as wheel:
        wheel.extractall(tmp_path)
    code = (
        "import importlib, sys; module = importlib.import_module("
        + repr(namespace)
        + "); "
        "assert 'isaaclab' not in sys.modules and 'isaacsim' not in sys.modules and 'newton' not in sys.modules; "
        "assert module.__file__.startswith("
        + repr(str(tmp_path))
        + ")"
    )
    result = subprocess.run(
        [sys.executable, "-S", "-c", code],
        cwd=tmp_path,
        env={**os.environ, "PYTHONPATH": str(tmp_path)},
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr


@pytest.mark.parametrize("environment", PACKAGES)
def test_backend_source_imports_do_not_cross_package_boundaries(environment):
    source, _ = PACKAGES[environment]
    forbidden = {
        "physx": {"dex01_sim_newton"},
        "newton": {"dex01_sim_asset"},
        "common": {"dex01_sim_asset", "dex01_sim_newton"},
    }[environment]
    for path in (ROOT / source).rglob("*.py"):
        if any(part in ("build", "dist") for part in path.parts):
            continue
        for node in ast.walk(ast.parse(path.read_text())):
            imported = (
                [node.module]
                if isinstance(node, ast.ImportFrom) and node.module
                else [item.name for item in node.names] if isinstance(node, ast.Import) else []
            )
            assert not any(name.split(".", 1)[0] in forbidden for name in imported), path
