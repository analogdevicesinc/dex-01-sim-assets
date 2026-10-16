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

"""Preserve physical dimensions when replaying scale-free physics body poses."""

from itertools import product

import numpy as np
from pxr import Gf, Usd, UsdGeom


def pose_matrix(pose):
    matrix = Gf.Matrix4d(1).SetRotate(Gf.Quatd(float(pose[6]), Gf.Vec3d(*map(float, pose[3:6]))))
    matrix.SetTranslateOnly(Gf.Vec3d(*map(float, pose[:3])))
    return matrix


def transform_points(points, matrix):
    matrix = np.asarray(matrix)
    return points @ matrix[:3, :3] + matrix[3, :3]


def geometry_points(prim):
    """Include primitive dimensions, not just meshes, in the geometry audit."""
    if prim.IsA(UsdGeom.Mesh):
        points = np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get())
    elif prim.IsA(UsdGeom.Cube):
        points = np.array(list(product((-1, 1), repeat=3))) * UsdGeom.Cube(prim).GetSizeAttr().Get() / 2
    else:
        raise ValueError(f"Unsupported captured geometry: {prim.GetPath()} ({prim.GetTypeName()})")
    if points.ndim != 2 or points.shape[1] != 3 or not len(points) or not np.isfinite(points).all():
        raise ValueError(f"Invalid captured geometry points: {prim.GetPath()}")
    return points


def capture_geometry(stage, body_paths):
    """Record scale and every mesh vertex in physical body coordinates before editing USD."""
    cache = UsdGeom.XformCache()
    arrays = {"body_scales": []}
    meshes = []
    for body_index, path in enumerate(body_paths):
        body = stage.GetPrimAtPath(path)
        world = cache.GetLocalToWorldTransform(body)
        scale = np.asarray(Gf.Transform(world).GetScale())
        rotation = np.asarray(world)[:3, :3] / scale[:, None]
        if (
            not np.isfinite(scale).all()
            or (scale <= 0).any()
            or not np.allclose(rotation @ rotation.T, np.eye(3), atol=1e-6)
            or np.linalg.det(rotation) <= 0
        ):
            raise ValueError(f"Unsupported reflected or sheared body transform: {path}")
        arrays["body_scales"].append(scale)
        physical_scale = Gf.Matrix4d(1).SetScale(Gf.Vec3d(*map(float, scale)))
        for prim in Usd.PrimRange(body, Usd.TraverseInstanceProxies()):
            if not prim.IsA(UsdGeom.Gprim):
                continue
            mesh_path = str(prim.GetPath())
            owner = max((p for p in body_paths if mesh_path == p or mesh_path.startswith(p + "/")), key=len)
            if owner != path:
                continue
            index = len(meshes)
            # Relative child transforms retain mesh offsets; body scale includes
            # every ancestor, including an Isaac Lab spawn wrapper.
            relative = cache.GetLocalToWorldTransform(prim) * world.GetInverse() * physical_scale
            arrays[f"points_{index}"] = transform_points(geometry_points(prim), relative)
            if prim.IsA(UsdGeom.Mesh):
                mesh = UsdGeom.Mesh(prim)
                arrays[f"counts_{index}"] = np.asarray(mesh.GetFaceVertexCountsAttr().Get())
                arrays[f"indices_{index}"] = np.asarray(mesh.GetFaceVertexIndicesAttr().Get())
            meshes.append({"path": mesh_path, "body_index": body_index, "kind": prim.GetTypeName()})
    arrays["body_scales"] = np.asarray(arrays["body_scales"])
    if not meshes:
        raise ValueError("No mesh geometry captured")
    return meshes, arrays


def apply_body_poses(stage, body_paths, poses, scales):
    """Apply world poses without cancelling scale inherited from a parent prim."""
    for index in sorted(range(len(body_paths)), key=lambda i: body_paths[i].count("/")):
        body = stage.GetPrimAtPath(body_paths[index])
        if not body.IsValid():
            raise RuntimeError(f"Captured body missing: {body_paths[index]}")
        world = Gf.Matrix4d(1).SetScale(Gf.Vec3d(*map(float, scales[index]))) * pose_matrix(poses[index])
        parent = UsdGeom.Xformable(body.GetParent()).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        xf = UsdGeom.Xformable(body)
        ops = xf.GetOrderedXformOps()
        if len(ops) != 1 or ops[0].GetOpType() != UsdGeom.XformOp.TypeTransform:
            xf.ClearXformOpOrder()
            op = xf.AddTransformOp(opSuffix="docs")
        else:
            op = ops[0]
        xf.SetResetXformStack(False)
        op.Set(world * parent.GetInverse())


def verify_geometry(stage, body_paths, poses, meshes, arrays):
    """Compare rendered vertices with the independent pre-export geometry, in metres."""
    cache = UsdGeom.XformCache()
    pose_error, vertex_error = 0.0, 0.0
    for path, pose, scale in zip(body_paths, poses, arrays["body_scales"]):
        expected = Gf.Matrix4d(1).SetScale(Gf.Vec3d(*map(float, scale))) * pose_matrix(pose)
        actual = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(path))
        error = float(np.max(np.abs(np.asarray(actual) - np.asarray(expected))))
        pose_error = max(pose_error, error)
        if error > 1e-6:
            raise RuntimeError(f"Body pose or scale differs from capture: {path}")
    for index, entry in enumerate(meshes):
        prim = stage.GetPrimAtPath(entry["path"])
        if not prim.IsValid() or prim.GetTypeName() != entry.get("kind", "Mesh"):
            raise RuntimeError(f"Captured geometry missing or changed type: {entry['path']}")
        if prim.IsA(UsdGeom.Mesh):
            mesh = UsdGeom.Mesh(prim)
            if not np.array_equal(
                mesh.GetFaceVertexCountsAttr().Get(), arrays[f"counts_{index}"]
            ) or not np.array_equal(mesh.GetFaceVertexIndicesAttr().Get(), arrays[f"indices_{index}"]):
                raise RuntimeError(f"Mesh topology differs from capture: {entry['path']}")
        actual = transform_points(geometry_points(prim), cache.GetLocalToWorldTransform(prim))
        expected = transform_points(arrays[f"points_{index}"], pose_matrix(poses[entry["body_index"]]))
        if actual.shape != expected.shape or not np.isfinite(actual).all():
            raise RuntimeError(f"Invalid rendered vertices: {entry['path']}")
        error = float(np.max(np.linalg.norm(actual - expected, axis=1)))
        vertex_error = max(vertex_error, error)
        if error > 1e-6:
            raise RuntimeError(f"Mesh surface differs from capture: {entry['path']} ({error:.6g} m)")
    return pose_error, vertex_error
