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

"""Report and interruption lifecycle independent of simulator imports."""

import json
import signal
import threading
from contextlib import contextmanager, suppress
from pathlib import Path


def run_with_report(run, report_path, mode):
    """Invalidate old evidence before startup and preserve current-run failures."""
    destination = Path(report_path) if report_path else None
    if destination:
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.unlink(missing_ok=True)
    try:
        return run()
    except (Exception, KeyboardInterrupt, SystemExit) as exc:
        if destination:
            current = None
            if destination.exists():
                with suppress(ValueError, OSError):
                    current = json.loads(destination.read_text())
            if not isinstance(current, dict):
                current = {"backend": "newton", "mode": mode, "observations": []}
            current["failure"] = current.get("failure") or str(exc) or type(exc).__name__
            destination.write_text(json.dumps(current, indent=2) + "\n")
        raise


@contextmanager
def graceful_stop():
    """Finish a physics tick and write its observations when Ctrl+C is received."""
    stop = threading.Event()
    previous = signal.getsignal(signal.SIGINT)
    signal.signal(signal.SIGINT, lambda signum, frame: stop.set())
    try:
        yield stop
    finally:
        signal.signal(signal.SIGINT, previous)


def validate_output_paths(report, recording, benchmark, capture):
    """Reject file collisions before invalidating previous report evidence."""
    files = [Path(path).resolve() for path in (report, recording, benchmark) if path is not None]
    if recording is not None:
        video = Path(recording)
        sidecars = [video.with_suffix(".recording.json"), video.with_suffix(".recording.npz")]
        outputs = [video, *sidecars]
        if any(path.exists() for path in outputs):
            raise ValueError("Recording outputs already exist; choose a new video path")
        files.extend(path.resolve() for path in sidecars)
    if len(set(files)) != len(files):
        raise ValueError("Report, video, recording sidecars and benchmark must use distinct output paths")
    if capture is not None:
        destination = Path(capture).resolve()
        if any(destination == path or destination in path.parents for path in files):
            raise ValueError("Capture directory must be separate from report, video and benchmark outputs")
