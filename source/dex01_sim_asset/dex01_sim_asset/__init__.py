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

"""Penetration-based tactile sensor for Isaac Lab."""

from importlib import import_module

__all__ = [
    "TactileSensor",
    "TactileSensorCfg",
    "TactileSensorData",
    "taxel_patterns",
    "vis_utils",
]


def __getattr__(name):
    # Newton must be importable without loading the Isaac Sim 5.1/PhysX sensor.
    modules = {
        "TactileSensor": ".tactile_sensor",
        "TactileSensorCfg": ".tactile_sensor_cfg",
        "TactileSensorData": ".tactile_sensor_data",
        "taxel_patterns": ".taxel_patterns",
        "vis_utils": ".vis_utils",
    }
    if name not in modules:
        raise AttributeError(name)
    module = import_module(modules[name], __name__)
    value = module if name in ("taxel_patterns", "vis_utils") else getattr(module, name)
    globals()[name] = value
    return value
