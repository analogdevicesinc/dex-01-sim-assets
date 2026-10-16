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

from __future__ import annotations

import os
from typing import TYPE_CHECKING

import numpy as np
import torch

if TYPE_CHECKING:
    from . import taxel_patterns_cfg


def grid_pattern(
    cfg: taxel_patterns_cfg.GridPatternCfg, device: str
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    """A regular grid pattern for tactile sensor.

    The grid pattern is made from taxels that span a 2D grid in the sensor's
    local coordinates from ``(-length/2, -width/2)`` to ``(length/2, width/2)``, which is defined
    by the ``size = (length, width)`` and ``resolution`` parameters in the config.

    Args:
        cfg: The configuration instance for the pattern.
        device: The device to create the pattern on.

    Returns:
        A tuple of ``(taxel_pos, taxel_axes, taxel2pixel)``: taxel positions of shape ``(N, 3)`` in the
        sensor frame, per-taxel rotation matrices of shape ``(N, 3, 3)`` whose third column is the
        surface normal, and the taxel-to-pixel map of shape ``(N, 2)`` indexing the tactile image.

    Raises:
        ValueError: If the ordering is not "xy" or "yx".
        ValueError: If the resolution is less than or equal to 0.
    """
    # check valid arguments
    if cfg.ordering not in ["xy", "yx"]:
        raise ValueError(f"Ordering must be 'xy' or 'yx'. Received: '{cfg.ordering}'.")
    if cfg.resolution[0] <= 0 or cfg.resolution[1] <= 0:
        raise ValueError(f"Resolution must be greater than 0. Received: '{cfg.resolution}'.")

    # resolve mesh grid indexing (note: torch meshgrid is different from numpy meshgrid)
    # check: https://github.com/pytorch/pytorch/issues/15301
    indexing = cfg.ordering if cfg.ordering == "xy" else "ij"
    # define grid pattern
    x = torch.arange(start=-cfg.size[0] / 2, end=cfg.size[0] / 2 + 1.0e-9, step=cfg.resolution[0], device=device)
    y = torch.arange(start=-cfg.size[1] / 2, end=cfg.size[1] / 2 + 1.0e-9, step=cfg.resolution[1], device=device)
    grid_x, grid_y = torch.meshgrid(x, y, indexing=indexing)

    # store into taxels starts
    num_taxels = grid_x.numel()
    taxel_pos = torch.zeros(num_taxels, 3, device=device)
    taxel_pos[:, 0] = grid_x.flatten()
    taxel_pos[:, 1] = grid_y.flatten()

    # A flat grid shares one orientation with the sensor frame, so every taxel normal is +z.
    taxel_axes = torch.eye(3, device=device).expand(num_taxels, 3, 3).clone()

    pixel_rows, pixel_cols = torch.meshgrid(
        torch.arange(len(y), device=device), torch.arange(len(x), device=device), indexing="ij"
    )
    if cfg.ordering == "yx":
        pixel_rows, pixel_cols = pixel_rows.T, pixel_cols.T
    row = pixel_rows.flatten()
    col = pixel_cols.flatten()
    taxel2pixel = torch.stack((row, col), dim=1)

    return taxel_pos, taxel_axes, taxel2pixel


def custom_pattern(
    cfg: taxel_patterns_cfg.CustomPatternCfg, device: str
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    """A custom pattern for tactile sensor described by npz file.

    Args:
        cfg: The configuration instance for the pattern.
        device: The device to create the pattern on.

    Returns:
        A tuple of ``(taxel_pos, taxel_axes, taxel2pixel)``: taxel positions of shape ``(N, 3)`` in the
        sensor frame, per-taxel rotation matrices of shape ``(N, 3, 3)`` whose third column is the
        surface normal, and the taxel-to-pixel map of shape ``(N, 2)`` indexing the tactile image.
    """
    if not os.path.exists(cfg.filename):
        raise FileNotFoundError(f"Tactile pattern file not found: {cfg.filename}")

    with np.load(cfg.filename, allow_pickle=False) as taxel_pattern_data:
        taxel_poses = taxel_pattern_data["poses"]
        taxel2pixel = taxel_pattern_data["taxel2pixel"]
    if taxel_poses.ndim != 3 or taxel_poses.shape[1:] != (3, 4):
        raise ValueError(f"Expected shape (N, 3, 4) but got {taxel_poses.shape} in {cfg.filename}")

    if taxel2pixel.shape != (len(taxel_poses), 2) or not np.issubdtype(taxel2pixel.dtype, np.integer):
        raise ValueError("taxel2pixel must be an integer array of shape (N, 2)")
    if (taxel2pixel < 0).any() or len(np.unique(taxel2pixel, axis=0)) != len(taxel2pixel):
        raise ValueError("taxel2pixel must contain unique nonnegative frame cells")
    if not np.isfinite(taxel_poses).all():
        raise ValueError("Taxel poses must be finite")
    rotation = taxel_poses[:, :, :3]
    if not np.allclose(rotation.transpose(0, 2, 1) @ rotation, np.eye(3), atol=1e-5) or not np.allclose(
        np.linalg.det(rotation), 1.0, atol=1e-5
    ):
        raise ValueError("Taxel axes must be proper orthonormal rotations")

    taxel_pos = torch.from_numpy(taxel_poses[:, :3, 3]).float().to(device)
    taxel_axes = torch.from_numpy(taxel_poses[:, :3, :3]).float().to(device)
    taxel2pixel = torch.from_numpy(taxel2pixel.astype(np.int64)).to(device)

    return taxel_pos, taxel_axes, taxel2pixel
