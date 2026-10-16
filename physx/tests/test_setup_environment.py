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

"""Do not modify an existing environment for a different Isaac Sim release."""

import os
import shlex
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize("version,allowed", [("6.0.1.0", False), ("5.0.0", False), ("5.1.0", True), ("5.1.0.0", True)])
def test_reused_environment_version_is_checked_before_installing(tmp_path, version, allowed):
    bin_dir = tmp_path / "venv/bin"
    bin_dir.mkdir(parents=True)
    marker = tmp_path / "installer-was-called"
    python = bin_dir / "python"
    python.write_text(
        '#!/bin/sh\ncase "$*" in\n*sys.version_info*) exit 0;;\n*importlib.metadata*) echo '
        + shlex.quote(version)
        + ";;\n*) exit 99;;\nesac\n"
    )
    python.chmod(0o755)
    uv = bin_dir / "uv"
    uv.write_text("#!/bin/sh\ntouch " + shlex.quote(str(marker)) + "\nexit 73\n")
    uv.chmod(0o755)
    (bin_dir / "activate").write_text("export PATH=" + shlex.quote(str(bin_dir)) + ':"$PATH"\n')
    env = dict(os.environ, PATH=str(bin_dir) + os.pathsep + os.environ["PATH"])
    result = subprocess.run(
        ["bash", str(ROOT / "setup_env.sh"), "--venv", str(bin_dir.parent)], env=env, capture_output=True, text=True
    )
    assert marker.exists() is allowed
    if allowed:
        assert result.returncode == 73
    else:
        assert result.returncode != 0
        assert "existing environment was preserved" in result.stderr
