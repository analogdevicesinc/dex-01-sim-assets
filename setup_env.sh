#!/usr/bin/env bash
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
#
# Creates an isolated Python environment with Isaac Sim, Isaac Lab and this package.
# Safe to re-run: each step is skipped if it is already done.

set -euo pipefail

ISAACSIM_VERSION="5.1.0"
TORCH_VERSION="2.7.0"
TORCHVISION_VERSION="0.22.0"
USD_CORE_VERSION="26.8"
NVIDIA_INDEX_URL="https://pypi.nvidia.com"
TORCH_INDEX_URL="https://download.pytorch.org/whl/cu128"
ISAACLAB_BRANCH="main"
ISAACLAB_REVISION="b0542fe2d45bf91c4e1d9ef6952b9c709c80b4e8"
PYTHON_VERSION="3.11"

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PARENT_DIR="$(cd "${REPO_ROOT}/.." && pwd)"
VENV_DIR="${PARENT_DIR}/env_isaaclab"
ISAACLAB_DIR="${PARENT_DIR}/IsaacLab"

usage() {
    cat <<EOF
Usage: $(basename "$0") [options]

  --venv PATH       Virtual environment to create or reuse (default: ${VENV_DIR})
  --isaaclab PATH   Isaac Lab checkout to create or reuse (default: ${ISAACLAB_DIR})
  -h, --help        Show this message

Everything is installed with uv, which also fetches the Python ${PYTHON_VERSION} interpreter that
Isaac Sim ${ISAACSIM_VERSION} needs, so the host's own Python version does not matter. Install uv
from https://docs.astral.sh/uv/getting-started/installation/.

The Isaac Sim install is tens of gigabytes, so point --venv at a disk with room.
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --venv)
            [[ $# -ge 2 && -n "$2" ]] || { usage >&2; exit 1; }
            VENV_DIR="$2"; shift 2 ;;
        --isaaclab)
            [[ $# -ge 2 && -n "$2" ]] || { usage >&2; exit 1; }
            ISAACLAB_DIR="$2"; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown option: $1" >&2; usage >&2; exit 1 ;;
    esac
done

log() { printf '\n[setup] %s\n' "$1"; }
die() { printf '\n[setup] error: %s\n' "$1" >&2; exit 1; }

# --- preflight ---------------------------------------------------------------

[[ "$(uname -s)" == "Linux" ]] || die "Isaac Sim is only supported on Linux and Windows; this is $(uname -s)."
command -v git >/dev/null || die "git not found."
command -v uv >/dev/null || die "uv not found. Install it with
       'curl -LsSf https://astral.sh/uv/install.sh | sh', then re-run."
command -v nvidia-smi >/dev/null || echo "[setup] warning: nvidia-smi not found. Isaac Sim needs an NVIDIA GPU to run."

# --- virtual environment -----------------------------------------------------

if [[ -d "${VENV_DIR}" ]]; then
    log "Reusing the environment at ${VENV_DIR}"
else
    log "Creating a virtual environment at ${VENV_DIR}"
    uv python install "${PYTHON_VERSION}"  # no-op when that interpreter is already available
    # --seed puts pip in the environment, for tooling that shells out to 'python -m pip'.
    uv venv --python "${PYTHON_VERSION}" --seed "${VENV_DIR}"
fi

set +u  # the activate script reads unset variables
# shellcheck disable=SC1091
source "${VENV_DIR}/bin/activate"
set -u

install() { uv pip install "$@"; }

# --- native build tools ------------------------------------------------------

# egl-probe (via Robomimic) shells out to cmake and make. A pyenv cmake shim
# may exist on PATH without a working executable, fooling upstream's preflight.
# Install CMake in this environment so it takes precedence over host shims.
# CMake 4 removes compatibility with egl-probe's CMake 2.8 project declaration.
log "Installing CMake for native dependencies"
install "cmake>=3.18,<4"
cmake --version
for build_tool in gcc g++ make; do
    "${build_tool}" --version >/dev/null 2>&1 || die "${build_tool} is missing or unusable. Install build-essential and re-run."
done

# --- Isaac Sim ---------------------------------------------------------------

# Metadata avoids prompting for the EULA during an installation check.
if python -c "from importlib.metadata import version; version('isaacsim')" 2>/dev/null; then
    log "Isaac Sim is already installed"
else
    log "Installing Isaac Sim ${ISAACSIM_VERSION} (this downloads tens of gigabytes)"
    install "isaacsim[all,extscache]==${ISAACSIM_VERSION}" --extra-index-url "${NVIDIA_INDEX_URL}"
fi

# Guarded so that pointing this script at an environment that already runs the demos does not
# replace a working PyTorch build.
if python -c "import sys, torch; sys.exit(0 if torch.__version__.split('+')[0] == '${TORCH_VERSION}' else 1)" 2>/dev/null; then
    log "PyTorch ${TORCH_VERSION} is already installed"
else
    log "Installing PyTorch ${TORCH_VERSION}"
    install -U "torch==${TORCH_VERSION}" "torchvision==${TORCHVISION_VERSION}" --index-url "${TORCH_INDEX_URL}"
fi

# --- Isaac Lab ---------------------------------------------------------------

if [[ -d "${ISAACLAB_DIR}" ]]; then
    log "Reusing the Isaac Lab checkout at ${ISAACLAB_DIR}"
else
    log "Cloning Isaac Lab (${ISAACLAB_BRANCH}) into ${ISAACLAB_DIR}"
    git clone https://github.com/isaac-sim/IsaacLab.git --branch "${ISAACLAB_BRANCH}" "${ISAACLAB_DIR}"
    git -C "${ISAACLAB_DIR}" checkout --detach "${ISAACLAB_REVISION}"
fi

ACTUAL_ISAACLAB_REVISION="$(git -C "${ISAACLAB_DIR}" rev-parse HEAD)"
if [[ "${ACTUAL_ISAACLAB_REVISION}" != "${ISAACLAB_REVISION}" ]]; then
    log "Existing Isaac Lab revision ${ACTUAL_ISAACLAB_REVISION} differs from tested ${ISAACLAB_REVISION}; preserving it"
fi

log "Installing the Isaac Lab extensions"
(cd "${ISAACLAB_DIR}" && ./isaaclab.sh --install)

# --- USD ---------------------------------------------------------------------

# Nothing declares usd-core as a dependency, so a fresh environment does not get one and the
# viewport renders black. Installed after the Isaac Lab extensions so that nothing in that step
# resolves a different build over this pin.
if python -c "import sys, importlib.metadata as md; sys.exit(0 if md.version('usd-core') == '${USD_CORE_VERSION}' else 1)" 2>/dev/null; then
    log "usd-core ${USD_CORE_VERSION} is already installed"
else
    log "Installing usd-core ${USD_CORE_VERSION}"
    install "usd-core==${USD_CORE_VERSION}"
fi

# --- this repository ---------------------------------------------------------

if command -v git-lfs >/dev/null; then
    log "Fetching the git-lfs assets"
    (cd "${REPO_ROOT}" && git lfs pull)
else
    echo "[setup] warning: git-lfs not found. The Tesollo assets will be pointer files and the hand demo will fail."
fi

log "Installing dex01_sim_asset"
install -e "${REPO_ROOT}/source/dex01_sim_asset"

# --- verify ------------------------------------------------------------------

log "Verifying the install"
# Checked by metadata rather than by importing. Importing isaaclab outside a running Isaac Sim
# app is not how upstream verifies an install, and it fails for reasons that do not mean the
# environment is broken. Running a demo is the real verification.
python - <<'PY'
from importlib.metadata import PackageNotFoundError, version

missing = []
for package in ("isaacsim", "isaaclab", "usd-core", "dex01_sim_asset"):
    try:
        print(f"  {package} {version(package)}")
    except PackageNotFoundError:
        missing.append(package)
if missing:
    raise SystemExit(f"[setup] error: not installed: {', '.join(missing)}")
PY

cat <<EOF

[setup] Done. To use this environment:

    source ${VENV_DIR}/bin/activate
    cd ${REPO_ROOT}
    python scripts/simple_example.py --headless

The first windowed or streamed run pulls extensions from the registry, prompts you to accept
the NVIDIA Omniverse EULA, and compiles shaders. Expect several minutes and a black viewport
before the scene appears.

EOF
