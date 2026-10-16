#!/usr/bin/env bash
# Copyright (c) 2026 Analog Devices, Inc. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0

# Preserve Isaac Lab's workspace lockfile and dependency overrides.
set -euo pipefail

NEWTON_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEX01_ROOT="$(cd "${NEWTON_ROOT}/.." && pwd)"
LAB_REVISION="ae37b028ea415c91ea2bc32609efcd759ed2b974"
LAB_DIR="${NEWTON_ROOT}/.isaaclab"
WITH_KIT=0
WITH_VIDEO=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --isaaclab) LAB_DIR="${2:?--isaaclab requires a path}"; shift 2 ;;
        --kit) WITH_KIT=1; shift ;;
        --video) WITH_KIT=1; WITH_VIDEO=1; shift ;;
        -h|--help)
            echo "Usage: $0 [--isaaclab PATH] [--kit] [--video]"
            echo "Create a pinned, isolated Newton environment; --video adds Kit and recording."
            exit 0 ;;
        *) echo "Unknown option: $1" >&2; exit 2 ;;
    esac
done

command -v uv >/dev/null
command -v git >/dev/null
if [[ ! -d "${LAB_DIR}" ]]; then
    mkdir -p "$(dirname "${LAB_DIR}")"
    git clone --depth 1 --branch v3.0.0-EA https://github.com/isaac-sim/IsaacLab.git "${LAB_DIR}"
fi
LAB_DIR="$(cd "${LAB_DIR}" && pwd)"
[[ "$(git -C "${LAB_DIR}" rev-parse HEAD)" == "${LAB_REVISION}" ]] || {
    echo "Refusing to reuse an Isaac Lab checkout other than ${LAB_REVISION}" >&2
    exit 1
}
[[ -z "$(git -C "${LAB_DIR}" status --porcelain --untracked-files=no)" ]] || {
    echo "Refusing to install from a modified Isaac Lab checkout" >&2
    exit 1
}
if [[ -d "${LAB_DIR}/.venv" ]]; then
    [[ -x "${LAB_DIR}/.venv/bin/python" ]] || {
        echo "Refusing to modify an environment without a Python interpreter" >&2
        exit 1
    }
    "${LAB_DIR}/.venv/bin/python" -c 'import sys; sys.exit(sys.version_info[:2] != (3, 12))' || {
        echo "Newton requires Python 3.12; the existing environment was preserved" >&2
        exit 1
    }
    "${LAB_DIR}/.venv/bin/python" - <<'PY'
import importlib.metadata

try:
    importlib.metadata.version("dex01-sim-asset")
except importlib.metadata.PackageNotFoundError:
    pass
else:
    raise SystemExit("Refusing to modify an environment containing the PhysX backend; choose a separate Isaac Lab checkout.")
PY
fi
extras=()
if [[ "${WITH_KIT}" == 1 ]]; then extras=(--extra isaacsim); fi
if [[ "${WITH_VIDEO}" == 1 ]]; then extras+=(--extra video); fi
(cd "${LAB_DIR}" && UV_PROJECT_ENVIRONMENT="${LAB_DIR}/.venv" UV_PYTHON=3.12 uv sync --frozen "${extras[@]}")
uv pip install --python "${LAB_DIR}/.venv/bin/python" --no-deps \
    -e "${DEX01_ROOT}/common/source/dex01_sim_common" \
    -e "${NEWTON_ROOT}/source/dex01_sim_newton"
echo "Pinned environment: ${LAB_DIR}/.venv"
echo "Activate ${LAB_DIR}/.venv/bin/activate and run scripts from ${NEWTON_ROOT}."
echo "Rerun this installer to change optional extras without changing the frozen simulation pins."
