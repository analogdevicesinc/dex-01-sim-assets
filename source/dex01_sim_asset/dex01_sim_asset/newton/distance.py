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

"""GPU mesh distance queries using Newton's public interface."""

import warp as wp
from newton.geometry import sdf_mesh


@wp.kernel
def query_distances(
    mesh: wp.uint64,
    points: wp.array(dtype=wp.vec3, ndim=2),
    velocity: wp.array(dtype=wp.vec3, ndim=2),
    active: wp.array(dtype=wp.bool),
    scale: float,
    epsilon: float,
    distances: wp.array(dtype=float, ndim=2),
    rates: wp.array(dtype=float, ndim=2),
):
    env, taxel = wp.tid()
    if not active[env]:
        distances[env, taxel] = 0.0
        rates[env, taxel] = 0.0
        return
    point = points[env, taxel]
    motion = epsilon * velocity[env, taxel]
    distance = sdf_mesh(mesh, point, 0.1) * scale
    distances[env, taxel] = wp.min(distance, 0.0)
    rate = (sdf_mesh(mesh, point + motion, 0.1) - sdf_mesh(mesh, point - motion, 0.1)) * scale / (2.0 * epsilon)
    rates[env, taxel] = wp.where(distance < 0.0, rate, 0.0)
