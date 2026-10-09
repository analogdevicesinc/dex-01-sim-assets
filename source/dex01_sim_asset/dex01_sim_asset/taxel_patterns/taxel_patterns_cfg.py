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

"""Configurations for the taxel layouts available to the tactile sensor."""

from __future__ import annotations

import os
from collections.abc import Callable
from dataclasses import MISSING
from typing import Literal

import torch
from isaaclab.utils import configclass

from . import taxel_patterns


@configclass
class PatternBaseCfg:
    """Base configuration for a pattern."""

    func: Callable[[PatternBaseCfg, str], tuple[torch.Tensor, torch.Tensor, torch.Tensor]] = MISSING
    """Function to generate the pattern.

    The function takes the configuration and the device name as arguments, and returns
    ``(taxel_pos, taxel_axes, taxel2pixel)`` of shapes ``(N, 3)``, ``(N, 3, 3)`` and ``(N, 2)``.
    """


@configclass
class GridPatternCfg(PatternBaseCfg):
    """Configuration for the grid pattern for tactile sensing

    Defines a 2D grid of taxels in the coordinates of the sensor.

    .. attention::
        The points are ordered based on the :attr:`ordering` attribute.

    """

    func: Callable = taxel_patterns.grid_pattern

    resolution: tuple[float, float] = MISSING
    """Grid resolution (in meters) along each dimension"""

    size: tuple[float, float] = MISSING
    """Grid size (length, width) (in meters)."""

    ordering: Literal["xy", "yx"] = "xy"
    """Specifies the ordering of points in the generated grid. Defaults to ``"xy"``.

    Consider a grid pattern with points at :math:`(x, y)` where :math:`x` and :math:`y` are the grid indices.
    The ordering of the points can be specified as "xy" or "yx". This determines the inner and outer loop order
    when iterating over the grid points.

    * If "xy" is selected, the points are ordered with inner loop over "x" and outer loop over "y".
    * If "yx" is selected, the points are ordered with inner loop over "y" and outer loop over "x".

    For example, the grid pattern points with :math:`X = (0, 1, 2)` and :math:`Y = (3, 4)`:

    * "xy" ordering: :math:`[(0, 3), (1, 3), (2, 3), (0, 4), (1, 4), (2, 4)]`
    * "yx" ordering: :math:`[(0, 3), (0, 4), (1, 3), (1, 4), (2, 3), (2, 4)]`
    """


@configclass
class CustomPatternCfg(PatternBaseCfg):
    """Configuration for the custom pattern for tactile sensing defined by npz file"""

    func: Callable = taxel_patterns.custom_pattern

    filename: str = MISSING
    """Path to npz file describing poses of each taxel

    The file contains two entries: 'poses' and 'taxel2pixel'.  The 'poses' array has
    a shape of Nx3x4 and describes the poses of each taxel in the parent prim's reference
    frame, where the z-axis is the surface normal.  The 'taxel2pixel' array has a shape
    of Nx2 and describes the pixel location of each taxel in the resulting tactile image.
    """

    resolution: tuple[float, float] = MISSING
    """Grid resolution (in meters) along each dimension"""


@configclass
class Dex01PatternCfg(CustomPatternCfg):
    """DEX-01 fingertip layout: 738 taxels mapped into a 32x32 frame.

    Contact poses are registered to the V10 overmold exterior. Firmware pixel
    indices are unchanged; the original reference remains in dex01_pattern.npz.
    Use the matching mounting offset with the DG-5F sensor asset.
    """

    filename: str = os.path.join(os.path.dirname(__file__), "dex01_contact_v10.npz")

    resolution: tuple[float, float] = (0.00072, 0.00088)
