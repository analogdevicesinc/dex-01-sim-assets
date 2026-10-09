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

"""DEX-01 Newton demos in the pinned Isaac Lab EA environment."""

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "source/dex01_sim_asset"))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("sensor", "hand", "press"), default="hand")
    parser.add_argument("--indenter", choices=("sphere", "flat", "gear"), default="gear")
    parser.add_argument("--viz", choices=("none", "kit"), default="none")
    parser.add_argument("--livestream", type=int, choices=(0, 1, 2), default=0)
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--num-envs", type=int, default=1)
    parser.add_argument("--max-steps", type=int, default=1800)
    parser.add_argument("--report", type=Path)
    parser.add_argument("--check-report", type=Path, help="Validate an existing report without launching simulation.")
    parser.add_argument("--record", type=Path, help="Write synchronized RGB/tactile video.")
    parser.add_argument("--debug-vis", action="store_true")
    parser.add_argument("--no-tactile-ui", action="store_true")
    parser.add_argument("--benchmark", type=Path, help="Write warm physics/sensing performance measurements.")
    parser.add_argument("--no-sensing", action="store_true", help="Physics-only benchmark; requires --benchmark.")
    args = parser.parse_args()
    if args.check_report:
        from dex01_newton_verification import check_report

        check_report(json.loads(args.check_report.read_text()))
        print("Newton report validation passed")
        return
    if args.num_envs < 1 or args.max_steps < 1:
        parser.error("Environment and step counts must be positive")
    if args.record and args.debug_vis:
        parser.error("Disable debug markers for recordings")
    if args.device == "cpu":
        parser.error("The validated Newton SDF workflow requires an NVIDIA CUDA device")
    if args.livestream and args.viz != "kit":
        parser.error("Streaming requires --viz kit")
    if args.debug_vis and args.viz == "none":
        parser.error("Debug markers require --viz kit")
    if args.no_sensing and (not args.benchmark or args.report or args.record or args.mode == "press"):
        parser.error("--no-sensing requires a physics-only benchmark without reports, recording or pressing")
    from dex01_newton_lifecycle import run_with_report

    run_with_report(lambda: launch(args), args.report, args.mode)


def launch(args):
    # Kit must start before anything imports pxr, avoiding incompatible USD bindings.
    launcher = None
    if args.viz == "kit" or args.record:
        # Load the pinned Torch CUDA runtime before Kit exposes bundled CUDA libs.
        import torch

        torch.cuda.init()
        from isaaclab.app import AppLauncher

        launcher = AppLauncher(
            visualizer=["kit"] if args.viz == "kit" else None,
            livestream=args.livestream,
            device=args.device,
            enable_cameras=bool(args.record),
        )
    try:
        from dex01_newton_demo import run
        from dex01_newton_lifecycle import graceful_stop

        with graceful_stop() as stop:
            run(args, ROOT, launcher, stop=stop)
    finally:
        if launcher is not None:
            launcher.app.close()


if __name__ == "__main__":
    main()
