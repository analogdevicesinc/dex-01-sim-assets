#!/usr/bin/env python3
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

"""Collect reproducible host and pinned-stack evidence without importing a simulator."""

import argparse
import importlib.metadata
import json
import platform
import shutil
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

EXPECTED = {
    "isaacsim": "6.1.0.0",
    "newton": "1.5.2",
    "warp-lang": "1.16.0",
    "torch": "2.11.0",
    "torchvision": "0.26.0",
    "usd-exchange": "2.3.0",
}
LAB_REVISION = "ae37b028ea415c91ea2bc32609efcd759ed2b974"


def command(args):
    """Capture a diagnostic command, including its failure rather than suppressing it."""
    try:
        result = subprocess.run(args, capture_output=True, text=True, timeout=30)
        return {"command": args, "returncode": result.returncode, "stdout": result.stdout, "stderr": result.stderr}
    except (OSError, subprocess.TimeoutExpired) as error:
        return {"command": args, "error": str(error)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--isaaclab", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--kit", action="store_true", help="Require the full rendering/recording prerequisites.")
    args = parser.parse_args()
    packages = {}
    for package in (*EXPECTED, "isaaclab", "mujoco", "mujoco-warp"):
        try:
            packages[package] = importlib.metadata.version(package)
        except importlib.metadata.PackageNotFoundError:
            packages[package] = None
    revision = command(["git", "-C", str(args.isaaclab), "rev-parse", "HEAD"])
    failures = []
    if sys.version_info[:2] != (3, 12):
        failures.append("Requires Python 3.12")
    if revision.get("stdout", "").strip() != LAB_REVISION:
        failures.append("Isaac Lab checkout does not match the approved EA revision")
    for package, expected in EXPECTED.items():
        if package == "isaacsim" and not args.kit:
            continue
        actual = packages[package]
        if actual is None or actual.split("+")[0] != expected:
            failures.append(f"{package}: expected {expected}, found {actual}")
    for package in ("mujoco", "mujoco-warp"):
        if not (packages[package] or "").startswith("3.11."):
            failures.append(f"{package}: requires the locked 3.11.x series")
    gpu = command(["nvidia-smi", "--query-gpu=name,driver_version,memory.total", "--format=csv,noheader"])
    if gpu.get("returncode") != 0:
        failures.append("NVIDIA GPU/driver is unavailable")
    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is None:
        try:
            import imageio_ffmpeg

            ffmpeg = imageio_ffmpeg.get_ffmpeg_exe()
        except (ImportError, RuntimeError):
            pass
    if args.kit and not ffmpeg:
        failures.append("FFmpeg is required for recording")
    report = {
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "command": sys.argv,
        "python": sys.version,
        "platform": platform.platform(),
        "libc": platform.libc_ver(),
        "packages": packages,
        "isaaclab_revision": revision,
        "gpu": gpu,
        "memory": command(["free", "-b"]),
        "disk": command(["df", "-B1", str(args.isaaclab)]),
        "ffmpeg": command([ffmpeg, "-version"]) if ffmpeg else None,
        "failures": failures,
        "status": "failed" if failures else "prerequisites-present-runtime-unverified",
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    return bool(failures)


if __name__ == "__main__":
    raise SystemExit(main())
