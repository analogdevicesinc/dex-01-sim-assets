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

"""Prevent accidental publication of native design inputs, including LFS pointers."""

import importlib.util
import sys
from pathlib import Path

import pytest

spec = importlib.util.spec_from_file_location(
    "public_asset_guard", Path(__file__).resolve().parents[1] / "scripts/check_public_assets.py"
)
guard = importlib.util.module_from_spec(spec)
spec.loader.exec_module(guard)


@pytest.mark.parametrize("name", ["part.step", "part.STEP", "part.stp", "drawing.DxF", "part.step.gz"])
def test_native_inputs_are_rejected_even_as_lfs_pointers(tmp_path, name):
    path = tmp_path / name
    path.write_text("version https://git-lfs.github.com/spec/v1\noid sha256:" + "0" * 64 + "\nsize 1\n")
    assert guard.rejection_reason(path)


@pytest.mark.parametrize(
    "content",
    [
        b"ISO-10303-21;\nHEADER;\n",
        b"\xef\xbb\xbf  ISO-10303-21;\n",
        b"  0\r\nSECTION\r\n  2\r\nHEADER\r\n",
        b"0\nSECTION\n2\nENTITIES\n",
        b"AutoCAD Binary DXF\r\n\x1a\x00",
    ],
)
def test_renamed_native_contents_are_rejected(tmp_path, content):
    path = tmp_path / "export.obj"
    path.write_bytes(content)
    assert guard.rejection_reason(path)


@pytest.mark.parametrize(
    "name, content",
    [
        ("finger.obj", b"# Approved exterior\no Finger\nv 0 0 0\n"),
        ("finger.usda", b'#usda 1.0\ndef Xform "Finger" {}\n'),
        ("grid.npz", b"PK\x03\x04"),
        ("guide.md", b"Keep confidential inputs outside the repository.\n"),
    ],
)
def test_approved_artifact_formats_are_not_blanket_blocked(tmp_path, name, content):
    path = tmp_path / name
    path.write_bytes(content)
    assert guard.rejection_reason(path) is None


def test_ci_checks_indexed_paths_when_no_filenames_are_passed(monkeypatch):
    monkeypatch.setattr(sys, "argv", ["check_public_assets.py"])
    monkeypatch.setattr(guard.subprocess, "check_output", lambda args: b"secret.STEP\0")
    assert guard.main()
