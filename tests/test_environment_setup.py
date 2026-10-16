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

"""Run installer preflights with recorded commands and reject cross-environment writes."""

import os
import shlex
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def commands(tmp_path):
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    log = tmp_path / "commands.log"
    for name in ("git", "uv", "nvidia-smi"):
        path = bin_dir / name
        path.write_text(
            "#!/usr/bin/env bash\nprintf '%s %s\\n' '"
            + name
            + '\' "$*" >> "$TEST_COMMAND_LOG"\n'
            'if [[ "$1" == -C && "$3" == rev-parse ]]; then echo ae37b028ea415c91ea2bc32609efcd759ed2b974; fi\n'
        )
        path.chmod(0o755)
    return bin_dir, log


def run_setup(environment, path, commands, python_result=0):
    bin_dir, log = commands
    env_dir = path / ".venv" if environment == "newton" else path
    (env_dir / "bin").mkdir(parents=True)
    python = env_dir / "bin/python"
    python.write_text(
        '#!/usr/bin/env bash\nprintf \'python %s\\n\' "$*" >> "$TEST_COMMAND_LOG"\nexit ' + str(python_result) + "\n"
    )
    python.chmod(0o755)
    flag = "--isaaclab" if environment == "newton" else "--venv"
    result = subprocess.run(
        ["bash", str(ROOT / environment / "setup_env.sh"), flag, str(path)],
        env={**os.environ, "PATH": str(bin_dir) + ":" + os.environ["PATH"], "TEST_COMMAND_LOG": str(log)},
        capture_output=True,
        text=True,
        timeout=20,
    )
    return result, log.read_text() if log.exists() else ""


@pytest.mark.parametrize("environment", ["physx", "newton"])
def test_installer_rejects_wrong_python_before_dependency_mutations(tmp_path, commands, environment):
    result, log = run_setup(environment, tmp_path / "install", commands, python_result=1)
    assert result.returncode != 0
    assert "requires Python 3." in result.stderr
    assert "uv " not in log


def test_newton_setup_preserves_lockfile_and_installs_only_common_and_newton(tmp_path, commands):
    result, log = run_setup("newton", tmp_path / "lab", commands)
    assert result.returncode == 0, result.stderr
    assert "uv sync --frozen" in log
    assert "--no-deps" in log
    assert str(ROOT / "common/source/dex01_sim_common") in log
    assert str(ROOT / "newton/source/dex01_sim_newton") in log
    assert "physx/source" not in log


@pytest.mark.parametrize("environment", ["physx", "newton"])
def test_installer_help_is_read_only_from_any_directory(tmp_path, environment):
    result = subprocess.run(
        ["bash", str(ROOT / environment / "setup_env.sh"), "--help"], cwd=tmp_path, capture_output=True, text=True
    )
    assert result.returncode == 0
    assert "Usage:" in result.stdout
    assert not list(tmp_path.iterdir())


@pytest.mark.parametrize("environment,foreign_package", [("physx", "dex01-sim-newton"), ("newton", "dex01-sim-asset")])
def test_installer_rejects_foreign_backend_metadata_without_running_uv(
    tmp_path, commands, environment, foreign_package
):
    bin_dir, log = commands
    path = tmp_path / "foreign-environment"
    env_dir = path / ".venv" if environment == "newton" else path
    (env_dir / "bin").mkdir(parents=True)
    python = env_dir / "bin/python"
    python.write_text(
        '#!/usr/bin/env bash\nif [[ "$1" == -c ]]; then exit 0; fi\nexec ' + shlex.quote(sys.executable) + ' "$@"\n'
    )
    python.chmod(0o755)
    metadata = tmp_path / "site" / (foreign_package.replace("-", "_") + "-0.1.0a1.dist-info")
    metadata.mkdir(parents=True)
    (metadata / "METADATA").write_text("Metadata-Version: 2.1\nName: " + foreign_package + "\nVersion: 0.1.0a1\n")
    flag = "--isaaclab" if environment == "newton" else "--venv"
    result = subprocess.run(
        ["bash", str(ROOT / environment / "setup_env.sh"), flag, str(path)],
        env={
            **os.environ,
            "PATH": str(bin_dir) + ":" + os.environ["PATH"],
            "TEST_COMMAND_LOG": str(log),
            "PYTHONPATH": str(metadata.parent),
        },
        capture_output=True,
        text=True,
        timeout=20,
    )
    assert result.returncode != 0
    assert "environment containing the" in result.stderr
    assert "uv " not in (log.read_text() if log.exists() else "")
    assert (metadata / "METADATA").read_text().endswith("Version: 0.1.0a1\n")


def real_uv(tmp_path):
    executable = shutil.which("uv")
    assert executable, "uv is required to verify environment isolation"
    return executable, {
        **os.environ,
        "UV_CACHE_DIR": str(tmp_path / "uv-cache"),
        "UV_OFFLINE": "1",
        "UV_NO_MANAGED_PYTHON": "1",
        "UV_PYTHON_DOWNLOADS": "never",
    }


def test_newton_sync_ignores_foreign_project_environment_with_real_uv(tmp_path):
    uv, env = real_uv(tmp_path)
    repo = tmp_path / "repo"
    (repo / "newton").mkdir(parents=True)
    shutil.copy2(ROOT / "newton/setup_env.sh", repo / "newton/setup_env.sh")
    lab = tmp_path / "lab"
    lab.mkdir()
    lab.joinpath("pyproject.toml").write_text('[project]\nname="test-lab"\nversion="0.0.0"\nrequires-python=">=3.11"\n')
    result = subprocess.run([uv, "lock", "--offline"], cwd=lab, env=env, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    foreign = tmp_path / "foreign"
    result = subprocess.run(
        [uv, "venv", "--python", sys.executable, str(foreign)], env=env, capture_output=True, text=True
    )
    assert result.returncode == 0, result.stderr
    sentinel = foreign / "keep.txt"
    sentinel.write_text("foreign environment must survive")
    config = (foreign / "pyvenv.cfg").read_bytes()
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    git = bin_dir / "git"
    git.write_text(
        '#!/usr/bin/env bash\nif [[ "$3" == rev-parse ]]; then echo ae37b028ea415c91ea2bc32609efcd759ed2b974; fi\n'
    )
    git.chmod(0o755)
    log = tmp_path / "uv.log"
    wrapper = bin_dir / "uv"
    wrapper.write_text(
        '#!/usr/bin/env bash\nprintf "%s:%s\\n" "$1" "$UV_PROJECT_ENVIRONMENT" >> "$TEST_UV_LOG"\n'
        'if [[ "$1" == sync ]]; then UV_PYTHON='
        + shlex.quote(sys.executable)
        + " "
        + shlex.quote(uv)
        + ' "$@"; else exit 0; fi\n'
    )
    wrapper.chmod(0o755)
    result = subprocess.run(
        ["bash", str(repo / "newton/setup_env.sh"), "--isaaclab", str(lab)],
        env={
            **env,
            "PATH": str(bin_dir) + ":" + os.environ["PATH"],
            "UV_PROJECT_ENVIRONMENT": str(foreign),
            "TEST_UV_LOG": str(log),
        },
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert result.returncode == 0, result.stderr
    assert log.read_text().splitlines()[0] == "sync:" + str(lab / ".venv")
    assert (lab / ".venv/bin/python").exists()
    assert sentinel.read_text() == "foreign environment must survive"
    assert (foreign / "pyvenv.cfg").read_bytes() == config


def test_physx_pip_ignores_foreign_uv_python_with_real_uv(tmp_path):
    import zipfile

    uv, env = real_uv(tmp_path)
    intended, foreign = tmp_path / "intended", tmp_path / "foreign"
    for directory in (intended, foreign):
        result = subprocess.run(
            [uv, "venv", "--python", sys.executable, str(directory)], env=env, capture_output=True, text=True
        )
        assert result.returncode == 0, result.stderr
    # Use the actual installer function, installing an offline fixture wheel.
    script = (ROOT / "physx/setup_env.sh").read_text()
    function = script[script.index("install() {") : script.index("\n", script.index("install() {"))]
    fixture = tmp_path / "setup_probe-0.0.0-py3-none-any.whl"
    with zipfile.ZipFile(fixture, "w") as wheel:
        wheel.writestr("setup_probe/__init__.py", "VALUE = 17\n")
        wheel.writestr(
            "setup_probe-0.0.0.dist-info/METADATA", "Metadata-Version: 2.1\nName: setup-probe\nVersion: 0.0.0\n"
        )
        wheel.writestr(
            "setup_probe-0.0.0.dist-info/WHEEL",
            "Wheel-Version: 1.0\nGenerator: test\nRoot-Is-Purelib: true\nTag: py3-none-any\n",
        )
        wheel.writestr("setup_probe-0.0.0.dist-info/RECORD", "")
    result = subprocess.run(
        ["bash", "-c", function + '\ninstall --no-deps "$1"', "test", str(fixture)],
        env={**env, "VENV_DIR": str(intended), "VIRTUAL_ENV": str(intended), "UV_PYTHON": str(foreign / "bin/python")},
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr
    assert next((intended / "lib").rglob("setup_probe/__init__.py")).read_text() == "VALUE = 17\n"
    assert not list((foreign / "lib").rglob("setup_probe"))
