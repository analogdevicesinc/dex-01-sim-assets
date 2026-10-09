# Copyright (c) 2026 Analog Devices, Inc. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0 AND BSD-3-Clause
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

# ADI modifications are licensed under Apache-2.0.
# Portions adapted from Isaac Lab retain their BSD-3-Clause terms:
# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
# The full license text is reproduced in NOTICE.

from dataclasses import MISSING

from isaaclab.markers import VisualizationMarkersCfg
from isaaclab.sensors import SensorBaseCfg
from isaaclab.utils import configclass

from .marker_cfg import TACTILE_SENSOR_MARKER_CFG
from .tactile_sensor import TactileSensor
from .taxel_patterns.taxel_patterns_cfg import PatternBaseCfg


@configclass
class TactileSensorCfg(SensorBaseCfg):
    """Configuration for the tactile sensor."""

    @configclass
    class OffsetCfg:
        """The offset pose of the sensor's frame from the sensor's parent frame."""

        pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
        """Translation w.r.t. the parent frame. Defaults to (0.0, 0.0, 0.0)."""
        rot: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
        """Quaternion rotation (w, x, y, z) w.r.t. the parent frame. Defaults to (1.0, 0.0, 0.0, 0.0)."""

    class_type: type = TactileSensor

    mesh_prim_path: str = MISSING
    """Prim path expression (with '.*' env wildcard) for the counter object.

    May point at a rigid body or an articulation link. The prim must carry a collision mesh whose
    approximation is ``sdf``, since penetration is measured against the PhysX signed distance field.
    """

    offset: OffsetCfg = OffsetCfg()
    """The offset pose of the sensor's frame from the sensor's parent frame. Defaults to identity."""

    pattern_cfg: PatternBaseCfg = MISSING
    """The pattern that defines the local taxel layout in terms of positions and normals."""

    visualizer_cfg: VisualizationMarkersCfg = TACTILE_SENSOR_MARKER_CFG.replace(prim_path="/Visuals/TactileSensor")
    """The configuration object for the visualization markers. Defaults to TACTILE_SENSOR_MARKER_CFG.

    Note:
        This attribute is only used when debug visualization is enabled.
    """
