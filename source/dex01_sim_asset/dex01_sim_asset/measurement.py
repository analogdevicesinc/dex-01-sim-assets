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

"""Backend-independent normal-force redistribution and firmware mapping."""

import torch


def distribute_normal_load(depths, normals_w, normal_force_w):
    """Distribute filtered normal contact force [N] over negative depths [m].

    Inputs have shapes [E,T], [E,T,3] and [E,3]. Returns taxel loads [E,T]
    and the penetration-weighted outward unit normal [E,3]. Empty patches
    return zero loads; tangential forces must be removed by the backend.
    """
    depth_sum = depths.sum(-1, keepdim=True)
    mean_normal = (normals_w * depths.unsqueeze(-1)).sum(1) / depth_sum.clamp(max=-1e-8)
    mean_normal = mean_normal / mean_normal.norm(dim=-1, keepdim=True).clamp(min=1e-8)
    load = -(normal_force_w * mean_normal).sum(-1, keepdim=True)
    forces = torch.where(depths < 0, load.clamp(min=0) * depths / depth_sum.clamp(max=-1e-8), 0.0)
    return forces, mean_normal


def tactile_image(forces, pixels):
    """Scatter loads [N] into firmware cells with no interpolation or reordering."""
    height = int(pixels[:, 0].max().item()) + 1
    width = int(pixels[:, 1].max().item()) + 1
    image = forces.new_zeros((len(forces), height * width))
    indices = (pixels[:, 0] * width + pixels[:, 1]).expand(len(forces), -1)
    image.scatter_(1, indices, forces)
    return image.view(len(forces), height, width)
