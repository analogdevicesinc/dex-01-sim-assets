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

"""Tests for the taxel layout generators.

These run without Isaac Sim: ``taxel_patterns.py`` is loaded straight from its path, because
importing it as part of the package would pull in ``isaaclab`` through the config module.
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import torch

PACKAGE_DIR = Path(__file__).resolve().parents[1] / "source" / "dex01_sim_asset" / "dex01_sim_asset"
PATTERN_NPZ = PACKAGE_DIR / "taxel_patterns" / "dex01_pattern.npz"


def _load_module():
    path = PACKAGE_DIR / "taxel_patterns" / "taxel_patterns.py"
    spec = importlib.util.spec_from_file_location("_taxel_patterns", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


taxel_patterns = _load_module()


def test_grid_pattern_shapes_and_extent():
    cfg = SimpleNamespace(size=(0.004, 0.002), resolution=(0.001, 0.001), ordering="xy")
    taxel_pos, taxel_axes, taxel2pixel = taxel_patterns.grid_pattern(cfg, "cpu")

    num_taxels = 5 * 3
    assert taxel_pos.shape == (num_taxels, 3)
    assert taxel_axes.shape == (num_taxels, 3, 3)
    assert taxel2pixel.shape == (num_taxels, 2)

    assert torch.allclose(taxel_pos[:, 2], torch.zeros(num_taxels))
    assert taxel_pos[:, 0].min().item() == pytest.approx(-0.002)
    assert taxel_pos[:, 0].max().item() == pytest.approx(0.002)
    assert taxel_pos[:, 1].min().item() == pytest.approx(-0.001)
    assert taxel_pos[:, 1].max().item() == pytest.approx(0.001)

    # every taxel frame is the identity, so the surface normal is +z
    assert torch.allclose(taxel_axes, torch.eye(3).expand(num_taxels, 3, 3))


@pytest.mark.parametrize("ordering", ["xy", "yx"])
def test_grid_pattern_orderings_cover_the_same_points(ordering):
    cfg = SimpleNamespace(size=(0.003, 0.002), resolution=(0.001, 0.001), ordering=ordering)
    taxel_pos, _, _ = taxel_patterns.grid_pattern(cfg, "cpu")
    assert taxel_pos.shape == (4 * 3, 3)
    assert torch.unique(taxel_pos, dim=0).shape[0] == 4 * 3


@pytest.mark.parametrize("ordering", ["xy", "yx"])
def test_grid_pixel_indices_preserve_physical_rows_and_columns(ordering):
    cfg = SimpleNamespace(size=(0.003, 0.002), resolution=(0.001, 0.001), ordering=ordering)
    positions, _, pixels = taxel_patterns.grid_pattern(cfg, "cpu")
    assert torch.allclose(positions[:, 0], -0.0015 + pixels[:, 1] * 0.001)
    assert torch.allclose(positions[:, 1], -0.001 + pixels[:, 0] * 0.001)


def test_grid_pattern_rejects_bad_ordering():
    cfg = SimpleNamespace(size=(0.002, 0.002), resolution=(0.001, 0.001), ordering="zz")
    with pytest.raises(ValueError, match="Ordering"):
        taxel_patterns.grid_pattern(cfg, "cpu")


@pytest.mark.parametrize("resolution", [(0.0, 0.001), (0.001, -0.001)])
def test_grid_pattern_rejects_non_positive_resolution(resolution):
    cfg = SimpleNamespace(size=(0.002, 0.002), resolution=resolution, ordering="xy")
    with pytest.raises(ValueError, match="Resolution"):
        taxel_patterns.grid_pattern(cfg, "cpu")


def test_shipped_dex01_pattern_loads():
    cfg = SimpleNamespace(filename=str(PATTERN_NPZ), resolution=(0.0005, 0.0005))
    taxel_pos, taxel_axes, taxel2pixel = taxel_patterns.custom_pattern(cfg, "cpu")

    num_taxels = taxel_pos.shape[0]
    assert num_taxels > 0
    assert taxel_axes.shape == (num_taxels, 3, 3)
    assert taxel2pixel.shape == (num_taxels, 2)
    assert taxel_pos.dtype == torch.float32

    # the third column of each frame is the surface normal and must be unit length
    normals = taxel_axes[:, :, 2]
    assert torch.allclose(normals.norm(dim=-1), torch.ones(num_taxels), atol=1e-5)

    # pixel indices address a real image
    assert taxel2pixel.min().item() >= 0


@pytest.mark.parametrize("dtype", [np.int32, np.int64, np.uint32])
def test_custom_integer_mapping_supports_tactile_image_scatter(tmp_path, dtype):
    path = tmp_path / "pattern.npz"
    poses = np.zeros((3, 3, 4))
    poses[:, :, :3] = np.eye(3)
    np.savez(path, poses=poses, taxel2pixel=np.array([[0, 1], [1, 0], [1, 2]], dtype=dtype))
    _, _, pixels = taxel_patterns.custom_pattern(SimpleNamespace(filename=str(path)), "cpu")

    assert pixels.dtype == torch.int64
    forces = torch.tensor([[1.0, 2.0, 3.0]])
    image = torch.zeros(1, 6)
    image.scatter_(1, (pixels[:, 0] * 3 + pixels[:, 1]).unsqueeze(0), forces)
    assert torch.equal(image.reshape(1, 2, 3), torch.tensor([[[0.0, 1.0, 0.0], [2.0, 0.0, 3.0]]]))


def test_custom_pattern_reports_a_missing_file():
    cfg = SimpleNamespace(filename="/nonexistent/pattern.npz", resolution=(0.0005, 0.0005))
    with pytest.raises(FileNotFoundError, match="pattern.npz"):
        taxel_patterns.custom_pattern(cfg, "cpu")


def test_custom_pattern_rejects_wrong_pose_shape(tmp_path):
    bad = tmp_path / "bad.npz"
    np.savez(bad, poses=np.zeros((4, 4, 4)), taxel2pixel=np.zeros((4, 2), dtype=np.int64))
    cfg = SimpleNamespace(filename=str(bad), resolution=(0.0005, 0.0005))
    with pytest.raises(ValueError, match=r"\(N, 3, 4\)"):
        taxel_patterns.custom_pattern(cfg, "cpu")
