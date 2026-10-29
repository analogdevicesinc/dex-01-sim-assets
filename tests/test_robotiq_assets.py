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

"""Check the unified finger, anatomical taxel indexing, and gripper composition."""

import hashlib
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import trimesh
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

ROOT = Path(__file__).resolve().parents[1]
ASSET = ROOT / "assets/robots/robotiq_2f85"
PACKAGE = ROOT / "source/dex01_sim_asset/dex01_sim_asset"


def load_module(path):
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


BUILDER = load_module(ROOT / "scripts/build_robotiq_asset.py")
PATTERNS = load_module(PACKAGE / "taxel_patterns/taxel_patterns.py")
DEMO = load_module(ROOT / "scripts/dex01_on_robotiq_2f85.py")


def test_finger_is_one_closed_mesh_and_artifact_identity():
    reference = json.loads((ASSET / "reference.json").read_text())
    for prefix in ["exterior", "pattern"]:
        assert (
            hashlib.sha256((ROOT / reference[prefix + "_path"]).read_bytes()).hexdigest()
            == reference[prefix + "_sha256"]
        )
    vertices, faces, _, regions = BUILDER.load_mesh(ASSET / "finger.obj")
    mesh = trimesh.Trimesh(vertices, faces, process=False)
    assert mesh.is_watertight and mesh.is_winding_consistent and mesh.volume > 0
    assert len(mesh.split()) == 1
    assert set(regions) == {0, 1}
    assert np.allclose(mesh.extents, [0.027, 0.0315904630783, 0.0718796209145], atol=1e-8)
    assert np.allclose(mesh.bounds[:, 0], [-0.0135, 0.0135], atol=1e-10)
    stage = Usd.Stage.Open(str(ASSET / "finger.usda"))
    assert UsdGeom.GetStageMetersPerUnit(stage) == 1
    meshes = [p for p in stage.Traverse() if p.IsA(UsdGeom.Mesh)]
    assert len(meshes) == 1
    visual = UsdGeom.Mesh(meshes[0])
    np.testing.assert_allclose(visual.GetPointsAttr().Get(), vertices, atol=1e-8)
    np.testing.assert_array_equal(np.array(visual.GetFaceVertexIndicesAttr().Get()).reshape(-1, 3), faces)
    assert meshes[0].HasAPI(UsdPhysics.CollisionAPI)
    assert UsdPhysics.MeshCollisionAPI(meshes[0]).GetApproximationAttr().Get() == "sdf"


def test_grid_matches_humanoid_directions_and_sensing_surface():
    path = PACKAGE / "taxel_patterns/dex01_rectangular.npz"
    with np.load(path, allow_pickle=False) as data:
        assert set(data.files) == {"positions_m", "normals"}
        positions, normals = data["positions_m"], data["normals"]
    assert positions.shape == normals.shape == (32, 32, 3)
    n = np.array([0, np.cos(np.deg2rad(5)), -np.sin(np.deg2rad(5))])
    tip = np.array([0, np.sin(np.deg2rad(5)), np.cos(np.deg2rad(5))])
    right = np.cross(tip, n)
    dr, dc = np.diff(positions, axis=0), np.diff(positions, axis=1)
    np.testing.assert_allclose(dr, np.broadcast_to(-tip * 0.00105, dr.shape), atol=1e-10)
    np.testing.assert_allclose(dc, np.broadcast_to(right * 0.00045, dc.shape), atol=1e-10)
    np.testing.assert_allclose(normals, np.broadcast_to(n, normals.shape), atol=1e-10)
    # Independent reference: mounted humanoid rows go toward its +X base;
    # front-face columns go toward +Y (viewer's right with the tip pointing up).
    with np.load(PACKAGE / "taxel_patterns/dex01_contact.npz") as data:
        poses, cells = data["poses"], data["taxel2pixel"]
    reference = json.loads((ROOT / "assets/sensors/dex01/reference.json").read_text())
    q = Gf.Quatd(*reference["mount_rotation_wxyz"])
    rot = np.array(Gf.Matrix3d(q)).T
    points = poses[:, :, 3] @ rot.T + reference["mount_translation_m"]
    lookup = {tuple(cell): p for cell, p in zip(cells, points)}
    assert np.all(np.diff([lookup[r, 16][0] for r in range(32)]) > 0)
    assert np.all(np.diff([lookup[25, c][1] for c in range(10, 22)]) > 0)
    vertices, faces, _, regions = BUILDER.load_mesh(ASSET / "finger.obj")
    mesh = trimesh.Trimesh(vertices, faces, process=False)
    _, distances, nearest = trimesh.proximity.closest_point(mesh, positions.reshape(-1, 3))
    assert distances.max() < 1e-8
    assert np.all(regions[nearest] == 1)
    loaded, axes, pixels = PATTERNS.rectangular_dex01_pattern(SimpleNamespace(filename=path), "cpu")
    np.testing.assert_allclose(loaded.reshape(32, 32, 3), positions, atol=1e-8)
    np.testing.assert_array_equal(pixels.reshape(32, 32, 2), np.stack(np.indices((32, 32)), axis=-1))
    np.testing.assert_allclose(np.linalg.det(axes.numpy()), 1.0, atol=1e-6)
    np.testing.assert_allclose(axes[:, :, 2].reshape(32, 32, 3), normals, atol=1e-6)


def test_rectangular_grid_rejects_bad_normals(tmp_path):
    path = tmp_path / "bad.npz"
    np.savez(path, positions_m=np.zeros((32, 32, 3)), normals=np.zeros((32, 32, 3)))
    with pytest.raises(ValueError, match="unit length"):
        PATTERNS.rectangular_dex01_pattern(SimpleNamespace(filename=path), "cpu")


def test_asset_root_can_be_relocated_or_overridden_by_a_parent_layer(tmp_path):
    for name in ["library_a", "library_b"]:
        asset = tmp_path / name / BUILDER.GRIPPER_ASSET_PATH
        asset.parent.mkdir(parents=True)
        source = Usd.Stage.CreateNew(str(asset))
        prim = UsdGeom.Xform.Define(source, "/Robotiq_2F_85").GetPrim()
        source.SetDefaultPrim(prim)
        prim.CreateAttribute("libraryMarker", Sdf.ValueTypeNames.String).Set(name)
        source.GetRootLayer().Save()
    output = tmp_path / "integrated.usda"
    BUILDER.build_gripper(output, finger_path=ASSET / "finger.usda", asset_root=tmp_path / "library_a")
    stage = Usd.Stage.Open(str(output))
    assert stage.GetDefaultPrim().GetAttribute("libraryMarker").Get() == "library_a"
    references = stage.GetRootLayer().GetPrimAtPath("/Robotiq").referenceList.prependedItems
    assert references[0].assetPath == '`"${ISAAC_ASSET_ROOT}/' + BUILDER.GRIPPER_ASSET_PATH + '"`'
    # The caller changes only the variable, leaving the reusable layer untouched.
    parent = Usd.Stage.CreateInMemory()
    parent.GetRootLayer().expressionVariables = {"ISAAC_ASSET_ROOT": str(tmp_path / "library_b")}
    prim = UsdGeom.Xform.Define(parent, "/Configured").GetPrim()
    prim.GetReferences().AddReference(str(output))
    assert prim.GetAttribute("libraryMarker").Get() == "library_b"
    assert stage.GetDefaultPrim().GetAttribute("libraryMarker").Get() == "library_a"


def test_ambiguous_asset_source_is_rejected(tmp_path):
    with pytest.raises(ValueError, match="not both"):
        BUILDER.build_gripper(tmp_path / "bad.usda", "explicit.usda", asset_root="library")


def test_closure_limit_keeps_opposing_finger_exteriors_apart():
    vertices, _, _, _ = BUILDER.load_mesh(ASSET / "finger.obj")
    # Official outer-knuckle pivot -> inner-finger pivot in the open pose:
    # (0, -0.03060, 0.05466) -> (0, -0.06776, 0.09809).
    angles = np.linspace(0.0, np.deg2rad(BUILDER.MAX_CLOSURE_DEG), 101)
    advance = 0.03716 * (1 - np.cos(angles)) + 0.04343 * np.sin(angles)
    minimum_gap = 2 * (BUILDER.MOUNT_Y_M - vertices[:, 1].max() - advance)
    assert minimum_gap.min() > 0.0005
    stage = Sdf.Layer.FindOrOpen(str(ASSET / "robotiq_2f85_dex01.usda"))
    attr = stage.GetAttributeAtPath("/Robotiq/Robotiq_2F_85/Joints/finger_joint.physics:upperLimit")
    assert attr.default == pytest.approx(BUILDER.MAX_CLOSURE_DEG)


def test_composition_replaces_only_tips_and_preserves_linkage(tmp_path):
    # A minimal instanceable reference exercises the same composition contract
    # without requiring network access or redistributing NVIDIA geometry in CI.
    original = tmp_path / "original.usda"
    source = Usd.Stage.CreateNew(str(original))
    root = UsdGeom.Xform.Define(source, "/Robotiq_2F_85").GetPrim()
    source.SetDefaultPrim(root)
    body_root = "/Robotiq_2F_85/Robotiq_2F_85"
    UsdPhysics.ArticulationRootAPI.Apply(UsdGeom.Xform.Define(source, body_root).GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(UsdGeom.Xform.Define(source, body_root + "/base_link").GetPrim())
    shared = UsdGeom.Xform.Define(source, "/Shared").GetPrim()
    UsdGeom.Cube.Define(source, "/Shared/linkage")
    UsdGeom.Cube.Define(source, "/Shared/Defeatured_2F_85_PAD_OPEN_fingertipsstep_01")
    for side in ["left", "right"]:
        link = UsdGeom.Xform.Define(source, body_root + f"/{side}_inner_finger").GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(link)
        visual = UsdGeom.Xform.Define(source, str(link.GetPath()) + "/visuals").GetPrim()
        visual.GetReferences().AddInternalReference(shared.GetPath())
        visual.SetInstanceable(True)
        joint = UsdPhysics.RevoluteJoint.Define(source, body_root + "/" + side + "_joint")
        joint.CreateBody0Rel().SetTargets([body_root + "/base_link"])
        joint.CreateBody1Rel().SetTargets([link.GetPath()])
    source.GetRootLayer().Save()
    output = tmp_path / "integrated.usda"
    BUILDER.build_gripper(output, original, ASSET / "finger.usda")
    result = Usd.Stage.Open(str(output))
    cache = UsdGeom.XformCache()
    centers, normals = [], []
    with np.load(PACKAGE / "taxel_patterns/dex01_rectangular.npz") as data:
        p = data["positions_m"].mean(axis=(0, 1))
        n = data["normals"][0, 0]
    for side in ["left", "right"]:
        link_path = "/Robotiq/Robotiq_2F_85/" + side + "_inner_finger"
        assert result.GetPrimAtPath(link_path).HasAPI(UsdPhysics.RigidBodyAPI)
        assert result.GetPrimAtPath(link_path + "/visuals/linkage").IsActive()
        assert not result.GetPrimAtPath(link_path + "/" + BUILDER.STOCK_TIP).IsActive()
        child = result.GetPrimAtPath(link_path + "/dex01")
        assert not child.HasAPI(UsdPhysics.RigidBodyAPI)
        assert result.GetPrimAtPath(str(child.GetPath()) + "/geometry").HasAPI(UsdPhysics.CollisionAPI)
        transform = cache.GetLocalToWorldTransform(child)
        centers.append(np.array(transform.Transform(Gf.Vec3d(*p))))
        normals.append(np.array(transform.TransformDir(Gf.Vec3d(*n))))
        joint = UsdPhysics.Joint(result.GetPrimAtPath("/Robotiq/Robotiq_2F_85/" + side + "_joint"))
        assert joint.GetBody1Rel().GetTargets() == [Sdf.Path(link_path)]
    np.testing.assert_allclose(centers[0], np.array(centers[1]) * [1, -1, 1], atol=1e-10)
    assert normals[0][1] > 0.99 and normals[1][1] < -0.99
    assert len([p for p in result.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]) == 3


@pytest.mark.parametrize(
    "fault", ["no_contact", "no_release", "moved_base", "unresolved_hold", "projection", "imbalance", "momentum"]
)
def test_report_checker_rejects_invalid_integration(tmp_path, fault):
    report = {
        "status": "complete",
        "taxel_grid_shapes": [[32, 32], [32, 32]],
        "maximum_image_sum_error_n": 0.0,
        "maximum_contact_projection_error_n": 0.0,
        "maximum_unresolved_hold_contact_n": 0.0,
        "maximum_base_displacement_m": 0.0,
        "maximum_base_rotation_rad": 0.0,
        "maximum_hold_force_difference_n": 0.0,
        "maximum_momentum_residual_n_s": 0.0,
        "physics_dt_s": 1.0 / 240.0,
        "block_mass_kg": 0.1,
        "observations": [
            {
                "phase": phase,
                "joint_positions_rad": [0.4 if phase == "hold" else 0.0],
                "forces_n": [[2.0, 2.0] if phase == "hold" else [0.0, 0.0]],
                "normal_forces_on_block_n": (
                    [[[0.0, 2.0, 0.0], [0.0, -2.0, 0.0]]] if phase == "hold" else [[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]]
                ),
                "friction_forces_on_block_n": [[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]],
                "block_velocity_m_s": [[0.0, 0.0, 0.0]],
                "previous_block_velocity_m_s": [[0.0, 0.0, 0.0]],
            }
            for phase in ["hold"] * 35 + ["released"] * 35
        ],
    }
    if fault == "no_contact":
        report["observations"][0]["forces_n"][0][1] = 0.0
    elif fault == "no_release":
        report["observations"][-1]["forces_n"][0][0] = 1.0
    elif fault == "moved_base":
        report["maximum_base_displacement_m"] = 0.01
    elif fault == "unresolved_hold":
        report["maximum_unresolved_hold_contact_n"] = 0.1
    elif fault == "imbalance":
        report["observations"][0]["forces_n"] = [[2.023, 1.469]]
    elif fault == "momentum":
        report["observations"][0]["normal_forces_on_block_n"][0][0][1] = 3.0
    else:
        report["maximum_contact_projection_error_n"] = 0.1
    path = tmp_path / "report.json"
    path.write_text(json.dumps(report))
    with pytest.raises(AssertionError):
        DEMO.check_report(path)
