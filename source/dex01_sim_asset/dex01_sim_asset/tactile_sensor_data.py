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

from dataclasses import dataclass

import torch


@dataclass
class TactileSensorData:
    """Data container for the tactile sensor."""

    taxel_points_w: torch.Tensor | None = None
    """Position of the taxel points in world frame.

    Shape is (N, B, 3), where N is the number of sensors, and B
    is the number of taxels
    """
    taxel_normals_w: torch.Tensor | None = None
    """The normal vectors for each taxel in the world frame. Unit length

    Shape is (N, B, 3), where N is the number of sensors, B is the number of taxels
    """

    taxel_rot_w: torch.Tensor | None = None
    """Orientation of each taxel frame in the world frame, as a (w, x, y, z) quaternion.

    Shape is (N, B, 4), where N is the number of sensors, B is the number of taxels
    """

    normal_forces: torch.Tensor | None = None
    """The normal force magnitudes in the world frame.

    Shape is (N, B), where N is the number of sensors, B is the number of taxels
    """

    depths: torch.Tensor | None = None
    """Penetration of each taxel into the counter object, in metres. Negative when in contact,
    zero otherwise.

    Shape is (N, B), where N is the number of sensors, B is the number of taxels
    """

    depth_dots: torch.Tensor | None = None
    """Rate of change of :attr:`depths`, in metres per second.

    Shape is (N, B), where N is the number of sensors, B is the number of taxels
    """
