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

"""Signed distance fields of MuJoCo collision geoms, evaluated in each geom's local frame.

The Isaac Lab sensor queries a PhysX SDF of the counter object. MuJoCo has no point SDF query, so
the counter object is described by its collision geoms: primitives use exact analytic distances,
and meshes use the convex hull MuJoCo collides with (exact inside, a lower bound outside). A body
built from several geoms is their union, i.e. the minimum distance. Negative means inside.
"""

from __future__ import annotations

import mujoco
import numpy as np
from scipy.spatial import ConvexHull

_hull_cache: dict[tuple[int, int], np.ndarray] = {}


def _box(p, half):
    q = np.abs(p) - half
    outside = np.linalg.norm(np.maximum(q, 0.0), axis=-1)
    return outside + np.minimum(q.max(-1), 0.0)


def _cylinder(p, radius, half_height):
    d = np.stack([np.linalg.norm(p[:, :2], axis=-1) - radius, np.abs(p[:, 2]) - half_height], axis=-1)
    return np.minimum(d.max(-1), 0.0) + np.linalg.norm(np.maximum(d, 0.0), axis=-1)


def _capsule(p, radius, half_height):
    q = p.copy()
    q[:, 2] -= np.clip(q[:, 2], -half_height, half_height)
    return np.linalg.norm(q, axis=-1) - radius


def _mesh_hull(model, geom_id):
    key = (id(model), geom_id)
    if key not in _hull_cache:
        mesh = model.geom_dataid[geom_id]
        start, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
        _hull_cache[key] = ConvexHull(model.mesh_vert[start : start + count]).equations  # n.x + d <= 0 inside
    return _hull_cache[key]


def geom_sdf(model: mujoco.MjModel, data: mujoco.MjData, geom_id: int, points_w: np.ndarray) -> np.ndarray:
    """Signed distance (m) from world points to one geom."""
    local = (points_w - data.geom_xpos[geom_id]) @ data.geom_xmat[geom_id].reshape(3, 3)
    size = model.geom_size[geom_id]
    kind = model.geom_type[geom_id]
    if kind == mujoco.mjtGeom.mjGEOM_BOX:
        return _box(local, size)
    if kind == mujoco.mjtGeom.mjGEOM_SPHERE:
        return np.linalg.norm(local, axis=-1) - size[0]
    if kind == mujoco.mjtGeom.mjGEOM_CYLINDER:
        return _cylinder(local, size[0], size[1])
    if kind == mujoco.mjtGeom.mjGEOM_CAPSULE:
        return _capsule(local, size[0], size[1])
    if kind == mujoco.mjtGeom.mjGEOM_MESH:
        planes = _mesh_hull(model, geom_id)
        return (local @ planes[:, :3].T + planes[:, 3]).max(-1)
    raise NotImplementedError(f"No SDF for geom type {mujoco.mjtGeom(kind).name}")


def _boxes(data, geom_ids, sizes, points_w):
    """Union SDF of several boxes in one vectorized pass."""
    rotations = data.geom_xmat[geom_ids].reshape(-1, 3, 3)
    local = np.einsum("gij,gpi->gpj", rotations, points_w[None] - data.geom_xpos[geom_ids][:, None])
    q = np.abs(local) - sizes[:, None]
    distance = np.linalg.norm(np.maximum(q, 0.0), axis=-1) + np.minimum(q.max(-1), 0.0)
    return distance.min(0)


def body_sdf(model, data, geom_ids, points_w):
    """Union SDF of a counter body's collision geoms."""
    if len(points_w) == 0:
        return np.zeros(0)
    geom_ids = np.asarray(geom_ids)
    is_box = model.geom_type[geom_ids] == mujoco.mjtGeom.mjGEOM_BOX
    parts = [geom_sdf(model, data, g, points_w) for g in geom_ids[~is_box]]
    if is_box.any():
        parts.append(_boxes(data, geom_ids[is_box], model.geom_size[geom_ids[is_box]], points_w))
    return np.min(parts, axis=0)
