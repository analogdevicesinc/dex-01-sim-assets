#!/usr/bin/env bash
# Copyright (c) 2026 Analog Devices, Inc. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0

# Preserve Isaac Lab's workspace lockfile and dependency overrides.
set -euo pipefail

DEX01_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LAB_REVISION="ae37b028ea415c91ea2bc32609efcd759ed2b974"
LAB_DIR="${DEX01_ROOT}/.newton/IsaacLab"
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
[[ "$(git -C "${LAB_DIR}" rev-parse HEAD)" == "${LAB_REVISION}" ]] || {
    echo "Refusing to reuse an Isaac Lab checkout other than ${LAB_REVISION}" >&2
    exit 1
}
[[ -z "$(git -C "${LAB_DIR}" status --porcelain --untracked-files=no)" ]] || {
    echo "Refusing to install from a modified Isaac Lab checkout" >&2
    exit 1
}
extras=()
if [[ "${WITH_KIT}" == 1 ]]; then extras=(--extra isaacsim); fi
if [[ "${WITH_VIDEO}" == 1 ]]; then extras+=(--extra video); fi
(cd "${LAB_DIR}" && uv sync --frozen "${extras[@]}")
echo "Pinned environment: ${LAB_DIR}/.venv"
echo "Run through 'uv run --frozen' from ${LAB_DIR}, preserving the selected extras."
