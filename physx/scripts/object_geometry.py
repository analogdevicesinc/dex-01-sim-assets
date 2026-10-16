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

"""Locate an indenter's lower face without assuming its rigid-body origin is centered."""

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics


def collision_triangles(stage, body_path):
    """Return triangulated collision geometry in physical body coordinates (metres)."""
    body = stage.GetPrimAtPath(body_path)
    cache = UsdGeom.XformCache()
    transform = Gf.Transform(cache.GetLocalToWorldTransform(body))
    rigid = Gf.Matrix4d(1).SetRotate(transform.GetRotation())
    rigid.SetTranslateOnly(transform.GetTranslation())
    triangles = []
    for prim in Usd.PrimRange(body, Usd.TraverseInstanceProxies()):
        if prim.IsA(UsdGeom.Mesh) and prim.HasAPI(UsdPhysics.CollisionAPI):
            relative = np.asarray(cache.GetLocalToWorldTransform(prim) * rigid.GetInverse())
            mesh = UsdGeom.Mesh(prim)
            vertices = np.asarray(mesh.GetPointsAttr().Get())
            counts = np.asarray(mesh.GetFaceVertexCountsAttr().Get())
            if not (counts == 3).all():
                raise ValueError(f"Expected triangulated collision geometry under {body_path}")
            faces = np.asarray(mesh.GetFaceVertexIndicesAttr().Get()).reshape(-1, 3)
            points = vertices @ relative[:3, :3] + relative[3, :3]
            triangles.append(points[faces])
    if not triangles:
        raise ValueError(f"No collision mesh under {body_path}")
    result = np.concatenate(triangles)
    if not len(result) or not np.isfinite(result).all():
        raise ValueError(f"Invalid collision geometry under {body_path}")
    return result


def lower_face_center(stage, body_path):
    """Return the collision bounds' lower-face center in physical body coordinates (metres)."""
    vertices = collision_triangles(stage, body_path).reshape(-1, 3)
    low, high = vertices.min(0), vertices.max(0)
    return np.array([(low[0] + high[0]) / 2, (low[1] + high[1]) / 2, low[2]])


def projected_face_mask(points_xy, triangles_xy):
    """Test the real triangulated underside, retaining holes and tooth outlines."""
    a, b, c = np.moveaxis(np.asarray(triangles_xy), 1, 0)
    points = np.asarray(points_xy)[:, None, :]

    def cross(u, v):
        return u[..., 0] * v[..., 1] - u[..., 1] * v[..., 0]

    area = cross(b - a, c - a)
    sides = np.stack([cross(b - a, points - a), cross(c - b, points - b), cross(a - c, points - c)])
    return (((sides >= -1e-12).all(0) | (sides <= 1e-12).all(0)) & (np.abs(area) > 1e-12)).any(1)
