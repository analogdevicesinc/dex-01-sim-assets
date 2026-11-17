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

"""Build the single-mesh finger and compose it with the official Isaac Sim gripper."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import trimesh
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

ROOT = Path(__file__).resolve().parents[1]
ASSET = ROOT / "assets/robots/robotiq_2f85"
DEFAULT_ASSET_ROOT = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1"
GRIPPER_ASSET_PATH = "Isaac/Robots/Robotiq/2F-85/Robotiq_2F_85_edit.usd"
OFFICIAL_GRIPPER_URL = DEFAULT_ASSET_ROOT + "/" + GRIPPER_ASSET_PATH
LINK_ROOT = "Robotiq_2F_85"
STOCK_TIP = "visuals/Defeatured_2F_85_PAD_OPEN_fingertipsstep_01"
# The stock jaw's mounting plane and pin center in its authored link frame.
MOUNT_Y_M = 0.05740763
MOUNT_Z_M = 0.11042019
# The custom adapters meet before the stock 47-degree stop. At 35.9 degrees,
# the authored parallel-jaw kinematics retain at least 0.5 mm between exteriors.
MAX_CLOSURE_DEG = 35.9
# Simulation assumptions, not measured hardware mass or elastomer properties.
FINGER_MASS_KG = 0.030
ASSEMBLED_JAW_MASS_KG = 0.060
CONTACT_STIFFNESS_N_M = 10000.0
CONTACT_DAMPING_N_S_M = 50.0


def load_mesh(path):
    """Read the one-object OBJ without depending on an external MTL loader."""
    vertices, normals, faces, normal_indices, regions = [], [], [], [], []
    region = None
    for line in Path(path).read_text().splitlines():
        fields = line.split()
        if not fields:
            continue
        if fields[0] == "v":
            vertices.append([float(v) for v in fields[1:4]])
        elif fields[0] == "vn":
            normals.append([float(v) for v in fields[1:4]])
        elif fields[0] == "usemtl":
            region = {"finger_body": 0, "tactile_surface": 1}[fields[1]]
        elif fields[0] == "f":
            if len(fields) != 4 or region is None:
                raise ValueError("Finger OBJ must contain material-tagged triangles")
            corners = [[int(v) - 1 for v in value.split("//")] for value in fields[1:]]
            faces.append([c[0] for c in corners])
            normal_indices.extend(c[1] for c in corners)
            regions.append(region)
    return np.array(vertices), np.array(faces), np.array(normals)[normal_indices], np.array(regions)


def material(stage, path, color):
    result = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path + "/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.65)
    result.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return result


def build_finger(destination):
    metadata = json.loads((ASSET / "reference.json").read_text())
    obj = ASSET / "finger.obj"
    if hashlib.sha256(obj.read_bytes()).hexdigest() != metadata["exterior_sha256"]:
        raise ValueError("Finger exterior hash mismatch")
    vertices, faces, normals, regions = load_mesh(obj)
    mesh = trimesh.Trimesh(vertices, faces, process=False)
    if not mesh.is_watertight or not mesh.is_winding_consistent or len(mesh.split()) != 1:
        raise ValueError("Expected one closed finger exterior")
    stage = Usd.Stage.CreateNew(str(destination))
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    root = UsdGeom.Xform.Define(stage, "/Finger").GetPrim()
    stage.SetDefaultPrim(root)
    root.SetCustomDataByKey("exteriorSha256", metadata["exterior_sha256"])
    root.SetCustomDataByKey("patternSha256", metadata["pattern_sha256"])
    UsdPhysics.RigidBodyAPI.Apply(root)
    UsdPhysics.MassAPI.Apply(root).CreateMassAttr(FINGER_MASS_KG)
    geometry = UsdGeom.Mesh.Define(stage, "/Finger/geometry")
    geometry.CreatePointsAttr(vertices.tolist())
    geometry.CreateFaceVertexCountsAttr([3] * len(faces))
    geometry.CreateFaceVertexIndicesAttr(faces.ravel().tolist())
    geometry.CreateNormalsAttr(normals.tolist())
    geometry.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
    geometry.CreateSubdivisionSchemeAttr("none")
    geometry.CreateExtentAttr([vertices.min(0).tolist(), vertices.max(0).tolist()])
    for index, name, color in [(0, "body", (0.55, 0.59, 0.63)), (1, "sensing_surface", (0.973, 0.529, 0.004))]:
        mat = material(stage, "/Finger/materials/" + name, color)
        subset = UsdGeom.Subset.CreateGeomSubset(
            geometry,
            name,
            UsdGeom.Tokens.face,
            np.flatnonzero(regions == index).tolist(),
            "materialBind",
            UsdGeom.Tokens.partition,
        )
        UsdShade.MaterialBindingAPI.Apply(subset.GetPrim()).Bind(mat)
    # One mesh supplies both rendering and contact geometry.
    UsdPhysics.CollisionAPI.Apply(geometry.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(geometry.GetPrim()).CreateApproximationAttr("sdf")
    for schema in ["PhysxCollisionAPI", "PhysxSDFMeshCollisionAPI"]:
        geometry.GetPrim().AddAppliedSchema(schema)
    for key, value, kind in [
        ("physxSDFMeshCollision:sdfResolution", 384, Sdf.ValueTypeNames.Int),
        ("physxCollision:contactOffset", 0.0001, Sdf.ValueTypeNames.Float),
        ("physxCollision:restOffset", 0.0, Sdf.ValueTypeNames.Float),
    ]:
        geometry.GetPrim().CreateAttribute(key, kind).Set(value)
    physics = UsdShade.Material.Define(stage, "/Finger/materials/contact")
    api = UsdPhysics.MaterialAPI.Apply(physics.GetPrim())
    api.CreateStaticFrictionAttr(1.0)
    api.CreateDynamicFrictionAttr(1.0)
    physics.GetPrim().AddAppliedSchema("PhysxMaterialAPI")
    physics.GetPrim().CreateAttribute("physxMaterial:compliantContactStiffness", Sdf.ValueTypeNames.Float).Set(
        CONTACT_STIFFNESS_N_M
    )
    physics.GetPrim().CreateAttribute("physxMaterial:compliantContactDamping", Sdf.ValueTypeNames.Float).Set(
        CONTACT_DAMPING_N_S_M
    )
    UsdShade.MaterialBindingAPI.Apply(geometry.GetPrim()).Bind(physics, materialPurpose="physics")
    stage.GetRootLayer().Save()
    destination = Path(destination)
    destination.write_text(destination.read_text().rstrip() + "\n")


def build_gripper(destination, source=None, finger_path=None, *, asset_root=None):
    """Author an override layer; the official geometry and joints remain referenced."""
    if source is not None and asset_root is not None:
        raise ValueError("Choose an explicit gripper USD or an asset root, not both")
    stage = Usd.Stage.CreateNew(str(destination))
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    root = UsdGeom.Xform.Define(stage, "/Robotiq").GetPrim()
    stage.SetDefaultPrim(root)
    if source is None:
        # Standard USD variable expressions also allow an enclosing layer to
        # override the root without modifying this asset or its relative path.
        library = str(asset_root or DEFAULT_ASSET_ROOT)
        if "://" not in library:
            library = Path(library).expanduser().resolve().as_posix()
        stage.GetRootLayer().expressionVariables = {"ISAAC_ASSET_ROOT": library.rstrip("/")}
        source = '`"${ISAAC_ASSET_ROOT}/' + GRIPPER_ASSET_PATH + '"`'
    elif "://" not in str(source):
        source = Path(source).expanduser().resolve().as_posix()
    root.GetReferences().AddReference(str(source))
    root.SetCustomDataByKey("officialSource", OFFICIAL_GRIPPER_URL)
    root.GetVariantSets().GetVariantSet("Physics").SetVariantSelection("Physx_Mimic")
    for side, y, quat in [("left", -MOUNT_Y_M, Gf.Quatd(1, 0, 0, 0)), ("right", MOUNT_Y_M, Gf.Quatd(0, 0, 0, 1))]:
        link_path = f"/Robotiq/{LINK_ROOT}/{side}_inner_finger"
        # Make this instance editable, then remove just its stock fingertip.
        stage.OverridePrim(link_path + "/visuals").SetInstanceable(False)
        stage.OverridePrim(link_path + "/" + STOCK_TIP).SetActive(False)
        attachment = UsdGeom.Xform.Define(stage, link_path + "/dex01")
        attachment.GetPrim().GetReferences().AddReference(str(finger_path or "./finger.usda"))
        attachment.GetPrim().RemoveAPI(UsdPhysics.RigidBodyAPI)
        attachment.GetPrim().RemoveAPI(UsdPhysics.MassAPI)
        attachment.ClearXformOpOrder()
        attachment.AddTranslateOp().Set(Gf.Vec3d(0, y, MOUNT_Z_M))
        attachment.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(quat)
        # Recompute inertia from the changed geometry instead of retaining the stock tip's tensor.
        mass = UsdPhysics.MassAPI.Apply(stage.OverridePrim(link_path))
        mass.CreateMassAttr(ASSEMBLED_JAW_MASS_KG)
        mass.CreateCenterOfMassAttr(Gf.Vec3f(float("-inf")))
        mass.CreateDiagonalInertiaAttr(Gf.Vec3f(0))
        mass.CreatePrincipalAxesAttr(Gf.Quatf(1))
    joint = UsdPhysics.FixedJoint.Define(stage, f"/Robotiq/{LINK_ROOT}/fix_base")
    joint.CreateBody1Rel().SetTargets([f"/Robotiq/{LINK_ROOT}/base_link"])
    drive = UsdPhysics.RevoluteJoint.Define(stage, f"/Robotiq/{LINK_ROOT}/Joints/finger_joint")
    drive.CreateUpperLimitAttr(MAX_CLOSURE_DEG)
    stage.GetRootLayer().Save()
    destination = Path(destination)
    destination.write_text(destination.read_text().rstrip() + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sources = parser.add_mutually_exclusive_group()
    sources.add_argument("--gripper-usd", help="Explicit gripper USD URL or local file.")
    sources.add_argument("--asset-root", help="Isaac library root, containing the Isaac/Robots directory.")
    parser.add_argument("--output-dir", type=Path, default=ASSET)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    build_finger(args.output_dir / "finger.usda")
    build_gripper(args.output_dir / "robotiq_2f85_dex01.usda", args.gripper_usd, asset_root=args.asset_root)
    print(f"Built finger and integrated gripper in {args.output_dir}")


if __name__ == "__main__":
    main()
