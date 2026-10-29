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

"""Reject native design inputs from repository additions and CI checkouts.

This format guard supplements the content/classification review in AGENTS.md;
it cannot determine whether arbitrary code, documents, or media are confidential.
"""

import argparse
import subprocess
from pathlib import Path

PRIVATE_INPUT_SUFFIXES = {".step", ".stp", ".dxf"}


def rejection_reason(path):
    path = Path(path)
    if PRIVATE_INPUT_SUFFIXES.intersection(suffix.lower() for suffix in path.suffixes):
        return "native design input; retain only approved derived simulation artifacts"
    if not path.is_file():
        return None
    with path.open("rb") as stream:
        prefix = stream.read(2048)
    if prefix.startswith(b"AutoCAD Binary DXF"):
        return "binary DXF content, regardless of filename"
    text = prefix.decode("utf-8-sig", errors="ignore").lstrip()
    if text.startswith("ISO-10303-21;"):
        return "STEP content, regardless of filename"
    lines = [line.strip() for line in text.splitlines()[:8]]
    if (
        len(lines) >= 4
        and lines[:3] == ["0", "SECTION", "2"]
        and lines[3]
        in {
            "HEADER",
            "CLASSES",
            "TABLES",
            "BLOCKS",
            "ENTITIES",
            "OBJECTS",
        }
    ):
        return "DXF content, regardless of filename"
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("files", nargs="*", help="Files to check; defaults to the Git index.")
    args = parser.parse_args()
    files = args.files
    if not files:
        files = subprocess.check_output(["git", "ls-files", "-z"]).decode().split("\0")
    rejected = []
    for name in sorted(set(filter(None, files))):
        reason = rejection_reason(name)
        if reason:
            rejected.append(name)
            print(f"Blocked {name}: {reason}")
    return bool(rejected)


if __name__ == "__main__":
    raise SystemExit(main())
