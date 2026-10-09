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

"""Author Newton collider overlays without changing released PhysX assets."""

import hashlib
import tempfile
import urllib.request
from pathlib import Path

import newton_usd_schemas  # noqa: F401 -- register schemas before USD parses the prototype
import numpy as np
import trimesh
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade


def cached_gear():
    """Download the demo's NVIDIA gear to a local cache; never redistribute it."""
    cache = Path(tempfile.gettempdir()) / "dex01-newton-assets"
    cache.mkdir(exist_ok=True)
    target = cache / "factory_gear_medium.usd"
    if not target.exists():
        url = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/Isaac/IsaacLab/Factory/factory_gear_medium.usd"
        with urllib.request.urlopen(url, timeout=60) as response:
            data = response.read()
        temporary = target.with_suffix(".download")
        temporary.write_bytes(data)
        temporary.replace(target)
    return target


def indenter_mesh(kind, press=False):
    """Return a centered collider [m], preserving the NVIDIA gear silhouette."""
    if kind == "sphere":
        return trimesh.creation.icosphere(subdivisions=3, radius=0.003)
    if kind == "flat":
        return trimesh.creation.box(extents=(0.006, 0.006, 0.006))
    stage = Usd.Stage.Open(str(cached_gear()))
    mesh = UsdGeom.Mesh(stage.GetPrimAtPath("/factory_gear_medium/factory_gear_medium/collisions"))
    points = np.array(mesh.GetPointsAttr().Get())
    faces = np.array(mesh.GetFaceVertexIndicesAttr().Get()).reshape(-1, 3)
    points -= (points.min(0) + points.max(0)) / 2
    points *= (0.012 if press else 0.006) / np.ptp(points, axis=0)[:2].max()
    return trimesh.Trimesh(points, faces, process=True)


def write_indenter(mesh):
    """Create a generated rigid mesh with explicit Newton SDF properties."""
    from isaaclab.sim.schemas import apply_mesh_collision_properties
    from isaaclab_newton.sim.schemas import NewtonSDFCollisionCfg

    stage = Usd.Stage.CreateInMemory()
    root = UsdGeom.Xform.Define(stage, "/Indenter").GetPrim()
    stage.SetDefaultPrim(root)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdPhysics.RigidBodyAPI.Apply(root)
    UsdPhysics.MassAPI.Apply(root).CreateMassAttr(0.1)
    collider = UsdGeom.Mesh.Define(stage, "/Indenter/collision")
    collider.CreatePointsAttr([Gf.Vec3f(*p) for p in mesh.vertices])
    collider.CreateFaceVertexCountsAttr([3] * len(mesh.faces))
    collider.CreateFaceVertexIndicesAttr(mesh.faces.ravel().tolist())
    collider.CreateSubdivisionSchemeAttr("none")
    UsdPhysics.CollisionAPI.Apply(collider.GetPrim())
    apply_mesh_collision_properties(
        str(collider.GetPath()),
        [
            NewtonSDFCollisionCfg(
                sdf_max_resolution=128, sdf_narrow_band_inner=-0.002, sdf_narrow_band_outer=0.002, sdf_padding=0.001
            )
        ],
        stage,
    )
    UsdPhysics.MeshCollisionAPI(collider.GetPrim()).CreateApproximationAttr("none")
    material = UsdShade.Material.Define(stage, "/Indenter/appearance")
    shader = UsdShade.Shader.Define(stage, "/Indenter/appearance/shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.9, 0.2, 0.05))
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(collider.GetPrim()).Bind(material)
    cache = Path(tempfile.gettempdir()) / "dex01-newton-assets"
    cache.mkdir(exist_ok=True)
    target = cache / (
        "indenter-" + hashlib.sha256(stage.GetRootLayer().ExportToString().encode()).hexdigest() + ".usda"
    )
    if not target.exists():
        stage.GetRootLayer().Export(str(target))
    return str(target)


def prepare_hand_asset(source):
    """Create a generated, flattened Newton overlay before scene cloning imports it."""
    source = Path(source)
    stage = Usd.Stage.Open(str(source))
    stage.GetDefaultPrim().GetVariantSets().GetVariantSet("fingertip").SetVariantSelection("dex01")
    flattened = Usd.Stage.Open(stage.Flatten())
    prepare_hand_colliders(flattened, str(flattened.GetDefaultPrim().GetPath()))
    author_sensor_sdfs(flattened)
    cache = Path(tempfile.gettempdir()) / "dex01-newton-assets"
    cache.mkdir(exist_ok=True)
    digest = hashlib.sha256(flattened.GetRootLayer().ExportToString().encode()).hexdigest()
    target = cache / f"hand-{digest}.usda"
    if not target.exists():
        flattened.GetRootLayer().Export(str(target))
    return str(target)


def author_sensor_sdfs(stage):
    """Cook fingertip SDF intent before Isaac Lab imports its scene prototype."""
    from isaaclab.sim.schemas import apply_mesh_collision_properties
    from isaaclab_newton.sim.schemas import NewtonSDFCollisionCfg

    for prim in stage.Traverse():
        if prim.GetTypeName() == "Mesh" and prim.GetName() == "collision":
            apply_mesh_collision_properties(
                str(prim.GetPath()),
                [
                    NewtonSDFCollisionCfg(
                        sdf_max_resolution=384,
                        sdf_narrow_band_inner=-0.002,
                        sdf_narrow_band_outer=0.002,
                        sdf_padding=0.001,
                    )
                ],
                stage,
            )
            UsdPhysics.MeshCollisionAPI(prim).CreateApproximationAttr("none")


def prepare_sensor_asset(source):
    stage = Usd.Stage.Open(str(source))
    stage = Usd.Stage.Open(stage.Flatten())
    author_sensor_sdfs(stage)
    cache = Path(tempfile.gettempdir()) / "dex01-newton-assets"
    cache.mkdir(exist_ok=True)
    target = cache / ("sensor-" + hashlib.sha256(stage.GetRootLayer().ExportToString().encode()).hexdigest() + ".usda")
    if not target.exists():
        stage.GetRootLayer().Export(str(target))
    return str(target)


def prepare_hand_colliders(stage, root_path):
    """Move PhysX mesh-merge collider intent onto actual descendant meshes.

    Newton cannot collide an Xform carrying PhysxMeshMergeCollisionAPI.
    Deinstance only collision containers, preserve their mesh transforms, and
    author the existing convex-hull intent on those meshes instead.
    """
    root = stage.GetPrimAtPath(root_path)
    wrappers = [
        p
        for p in Usd.PrimRange(root, Usd.TraverseInstanceProxies())
        if p.GetName() == "collisions" and p.HasAPI(UsdPhysics.CollisionAPI)
    ]
    for wrapper in wrappers:
        wrapper.SetInstanceable(False)
        meshes = [p for p in Usd.PrimRange(wrapper) if p.GetTypeName() == "Mesh"]
        if not meshes:
            raise ValueError(f"Collision wrapper has no resolved meshes: {wrapper.GetPath()}")
        for mesh in meshes:
            UsdPhysics.CollisionAPI.Apply(mesh)
            UsdPhysics.MeshCollisionAPI.Apply(mesh).CreateApproximationAttr("convexHull")
        wrapper.RemoveAPI(UsdPhysics.CollisionAPI)
        wrapper.RemoveAPI(UsdPhysics.MeshCollisionAPI)
    return len(wrappers)
