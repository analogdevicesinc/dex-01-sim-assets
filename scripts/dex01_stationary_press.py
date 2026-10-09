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

"""WebRTC demo: articulate a fingertip against a stationary gear with the wrist fixed and show tactile forces.

python scripts/dex01_stationary_press.py --livestream=2
Uses the same hand, sensor configuration and GUI as the gravity-load example.
"""

import argparse
import json
import runpy
import sys
from pathlib import Path

if __name__ == "__main__":
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("-h", "--help", action="store_true")
    parser.add_argument("--check-report", type=Path)
    parser.add_argument("--verification-report", type=Path)
    args, _ = parser.parse_known_args()
    if args.help:
        sys.argv.insert(1, "--stationary-press")
        runpy.run_path(str(Path(__file__).with_name("dex01_on_tesollo_dg5f.py")), run_name="_dex01_hand_run")
        raise SystemExit(0)
    if args.check_report:
        from dex01_articulated_press import check_articulated_report

        force = check_articulated_report(json.loads(args.check_report.read_text()))
        print(f"Fixed wrist, articulated finger, stationary gear, sustained load ({force:.3f} N) and release passed")
        raise SystemExit(0)
    sys.argv.insert(1, "--stationary-press")
    from dex01_articulated_press import run_with_report

    run_with_report(
        lambda: runpy.run_path(str(Path(__file__).with_name("dex01_on_tesollo_dg5f.py")), run_name="__main__"),
        args.verification_report,
    )
