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

"""Build the physically scaled V10 DEX-01 USD from the released exterior OBJ.

Rebuilding this USD needs only the exterior OBJ and reference metadata, with
no CAD kernel, detailed component geometry or other checkout. Rendering and
collision share the same simplified exterior surface.
"""

import hashlib
import json
import shutil
import tempfile
from pathlib import Path

import numpy as np
import trimesh
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

ROOT = Path(__file__).resolve().parents[1]

# Compliant rigid-body approximation; these are validated simulation choices,
# not measured sensor mass, elastomer stiffness or damping.
SENSOR_MASS_KG = 0.020
CONTACT_STIFFNESS_N_M = 10000.0
CONTACT_DAMPING_N_S_M = 50.0
CONTACT_OFFSET_M = 0.0001  # 0.1 mm contact-generation margin; zero rest offset.
SENSOR_SDF_RESOLUTION = 384  # Numerical sampling resolution, not a taxel count.
FRICTION_COEFFICIENT = 1.0


def load_exterior(path):
    """Read the released triangulated OBJ, including corner normals/materials."""
    vertices, normals, triangles = [], [], []
    material = None
    for line in path.read_text().splitlines():
        fields = line.split()
        if not fields or fields[0].startswith("#"):
            continue
        if fields[0] == "v":
            vertices.append([float(value) for value in fields[1:]])
        elif fields[0] == "vn":
            normals.append([float(value) for value in fields[1:]])
        elif fields[0] == "usemtl":
            material = {"silver_frame": 0, "dark_overmold": 1}[fields[1]]
        elif fields[0] == "f":
            if len(fields) != 4 or material is None:
                raise ValueError("Exterior OBJ requires triangles with a known material")
            corners = [tuple(int(value) - 1 for value in corner.split("//")) for corner in fields[1:]]
            if any(len(corner) != 2 or min(corner) < 0 for corner in corners):
                raise ValueError("Exterior OBJ requires positive vertex and normal indices")
            triangles.append(([corner[0] for corner in corners], [corner[1] for corner in corners], material))
    vertices, normals = np.array(vertices), np.array(normals)
    # The OBJ groups faces by material. Corner-normal indices retain their
    # geometric order, so a stable ordering keeps USD rebuilds reproducible.
    triangles.sort(key=lambda triangle: tuple(triangle[1]))
    faces = np.array([triangle[0] for triangle in triangles])
    corner_normals = normals[np.array([triangle[1] for triangle in triangles])]
    regions = np.array([triangle[2] for triangle in triangles])
    if not np.isfinite(vertices).all() or not np.isfinite(corner_normals).all():
        raise ValueError("Exterior coordinates and normals must be finite")
    return vertices, faces, corner_normals, regions


def create_mesh(stage, path, vertices, faces, normals=None):
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in vertices])
    mesh.CreateFaceVertexCountsAttr([3] * len(faces))
    mesh.CreateFaceVertexIndicesAttr(faces.flatten().tolist())
    mesh.CreateExtentAttr([Gf.Vec3f(*vertices.min(0)), Gf.Vec3f(*vertices.max(0))])
    mesh.CreateSubdivisionSchemeAttr("none")
    if normals is not None:
        mesh.CreateNormalsAttr([Gf.Vec3f(*n) for n in normals])
        mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    return mesh


def appearance(stage, path, color, roughness, metallic=0.0):
    material = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path + "/PreviewSurface")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


def main():
    reference = json.loads((ROOT / "assets/sensors/dex01/reference.json").read_text())
    source = ROOT / reference["outer_surface_path"]
    if hashlib.sha256(source.read_bytes()).hexdigest() != reference["outer_surface_sha256"]:
        raise ValueError("Exterior OBJ hash mismatch")
    vertices, faces, normals, regions = load_exterior(source)
    envelope = trimesh.Trimesh(vertices, faces, process=False)
    if not envelope.is_watertight or not envelope.is_winding_consistent:
        raise ValueError("Exterior mesh must be watertight and consistently wound")
    destination = ROOT / "assets/robots/tesollo/dex01.usda"
    with tempfile.TemporaryDirectory(prefix="dex01-build-") as directory:
        temporary_asset = Path(directory) / "dex01.usda"
        stage = Usd.Stage.CreateNew(str(temporary_asset))
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        body = UsdGeom.Xform.Define(stage, "/Sensor").GetPrim()
        stage.SetDefaultPrim(body)
        body.SetCustomDataByKey("physicalScale", 1.0)
        body.SetCustomDataByKey("patternSha256", reference["contact_pattern_sha256"])
        body.SetCustomDataByKey("collisionModel", reference["collision_model"])
        UsdPhysics.RigidBodyAPI.Apply(body)
        UsdPhysics.MassAPI.Apply(body).CreateMassAttr(SENSOR_MASS_KG)
        metal = appearance(stage, "/Sensor/frame_appearance", (0.55, 0.57, 0.59), 0.4, 0.6)
        coating = appearance(stage, "/Sensor/overmold_appearance", (0.06, 0.07, 0.08), 0.75)
        visual = create_mesh(stage, "/Sensor/visual", vertices, faces)
        visual.CreateNormalsAttr([Gf.Vec3f(*n) for n in normals.reshape(-1, 3)])
        visual.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
        visual.GetPrim().SetCustomDataByKey("sourceObjSha256", reference["outer_surface_sha256"])
        for region, name, material in ((0, "silver_frame", metal), (1, "dark_overmold", coating)):
            subset = UsdGeom.Subset.CreateGeomSubset(
                visual,
                name,
                UsdGeom.Tokens.face,
                np.flatnonzero(regions == region).tolist(),
                "materialBind",
                UsdGeom.Tokens.partition,
            )
            UsdShade.MaterialBindingAPI.Apply(subset.GetPrim()).Bind(material)
        collision = create_mesh(stage, "/Sensor/collision", envelope.vertices, envelope.faces)
        collision.CreateVisibilityAttr(UsdGeom.Tokens.invisible)
        UsdPhysics.CollisionAPI.Apply(collision.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(collision.GetPrim()).CreateApproximationAttr("sdf")
        collision.GetPrim().AddAppliedSchema("PhysxCollisionAPI")
        collision.GetPrim().AddAppliedSchema("PhysxSDFMeshCollisionAPI")
        collision.GetPrim().CreateAttribute("physxSDFMeshCollision:sdfResolution", Sdf.ValueTypeNames.Int).Set(
            SENSOR_SDF_RESOLUTION
        )
        collision.GetPrim().CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float).Set(
            CONTACT_OFFSET_M
        )
        collision.GetPrim().CreateAttribute("physxCollision:restOffset", Sdf.ValueTypeNames.Float).Set(0.0)
        material = UsdShade.Material.Define(stage, "/Sensor/material")
        api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        api.CreateStaticFrictionAttr(FRICTION_COEFFICIENT)
        api.CreateDynamicFrictionAttr(FRICTION_COEFFICIENT)
        material.GetPrim().AddAppliedSchema("PhysxMaterialAPI")
        for key, value in (
            ("compliantContactStiffness", CONTACT_STIFFNESS_N_M),
            ("compliantContactDamping", CONTACT_DAMPING_N_S_M),
        ):
            material.GetPrim().CreateAttribute("physxMaterial:" + key, Sdf.ValueTypeNames.Float).Set(value)
        UsdShade.MaterialBindingAPI.Apply(collision.GetPrim()).Bind(material, materialPurpose="physics")
        stage.GetRootLayer().Save()
        temporary_asset.write_text(temporary_asset.read_text().rstrip() + "\n")
        with tempfile.NamedTemporaryFile(dir=destination.parent, suffix=".usda", delete=False) as output:
            output_path = Path(output.name)
        try:
            shutil.copyfile(temporary_asset, output_path)
            output_path.replace(destination)
        finally:
            output_path.unlink(missing_ok=True)
    print(f"Wrote {destination}: one exterior-only V10 OBJ at 1:1 scale, {len(envelope.faces)} collision faces")


if __name__ == "__main__":
    main()
