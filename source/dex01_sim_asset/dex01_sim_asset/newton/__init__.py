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

"""Newton integration; importing this namespace does not launch Kit."""

from importlib import import_module

__all__ = ["TactileSensor", "TactileSensorCfg"]


def __getattr__(name):
    if name not in __all__:
        raise AttributeError(name)
    value = getattr(import_module(".tactile_sensor", __name__), name)
    globals()[name] = value
    return value
