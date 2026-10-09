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

"""Verify released exterior geometry, dimensions, taxel mapping and mounting."""

import hashlib
import importlib.util
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
REFERENCE = json.loads((ROOT / "assets/sensors/dex01/reference.json").read_text())
PATTERN = ROOT / "source/dex01_sim_asset/dex01_sim_asset/taxel_patterns/dex01_pattern.npz"


def exterior_mesh():
    """Load the public builder's OBJ data without running a simulator."""
    import trimesh

    path = ROOT / "scripts/build_dex01_asset.py"
    spec = importlib.util.spec_from_file_location("_dex01_asset_builder", path)
    builder = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(builder)
    points, faces, _, _ = builder.load_exterior(ROOT / REFERENCE["outer_surface_path"])
    return trimesh.Trimesh(points, faces, process=False)


def test_corrected_source_pattern_identity_and_frames():
    assert hashlib.sha256(PATTERN.read_bytes()).hexdigest() == REFERENCE["pattern_sha256"]
    with np.load(PATTERN, allow_pickle=False) as data:
        poses, pixels = data["poses"], data["taxel2pixel"]
    assert poses.shape == (738, 3, 4)
    assert pixels.shape == (738, 2)
    assert np.isfinite(poses).all()
    assert len(np.unique(pixels, axis=0)) == 738
    assert np.issubdtype(pixels.dtype, np.integer)
    assert pixels.min() == 0 and pixels.max() == 31
    rotations = poses[:, :, :3]
    assert np.allclose(rotations.transpose(0, 2, 1) @ rotations, np.eye(3), atol=1e-6)
    assert np.allclose(np.linalg.det(rotations), 1.0, atol=1e-6)


def test_v10_exterior_identity_and_physical_scale():
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.Open(str(ROOT / "assets/robots/tesollo/dex01.usda"))
    assert UsdGeom.GetStageMetersPerUnit(stage) == 1.0
    assert stage.GetDefaultPrim().GetCustomDataByKey("physicalScale") == 1.0
    mesh = exterior_mesh()
    points, faces = mesh.vertices, mesh.faces
    visual = UsdGeom.Mesh(stage.GetPrimAtPath("/Sensor/visual"))
    assert np.max(np.abs(np.array(visual.GetPointsAttr().Get()) - points)) < 2e-9
    assert np.array_equal(np.array(visual.GetFaceVertexIndicesAttr().Get()).reshape(-1, 3), faces)
    # Independent V10 engineering dimensions; guard against inherited display scaling.
    assert np.allclose(np.ptp(points, axis=0) * 1000, [32.7, 22.40631, 19.0], atol=0.015)
    rear = points[np.isclose(points[:, 0], 0.0065, atol=1e-7)]
    assert np.allclose(np.ptp(rear[:, 1:], axis=0) * 1000, [22.4, 19.0], atol=0.015)
    assert not stage.GetPrimAtPath("/Sensor/distal_tip")
    assert not stage.GetPrimAtPath("/Sensor/sensing_skin")


def test_single_exterior_obj_matches_runtime_geometry():
    import trimesh
    from pxr import Usd, UsdGeom
    from scipy.spatial import cKDTree

    path = ROOT / REFERENCE["outer_surface_path"]
    assert hashlib.sha256(path.read_bytes()).hexdigest() == REFERENCE["outer_surface_sha256"]
    mesh = trimesh.load(path, force="mesh", process=False)
    expected = exterior_mesh()
    # OBJ normal seams and material ordering split vertices/reorder faces.
    assert len(mesh.faces) == len(expected.faces)
    for a, b in ((mesh, expected), (expected, mesh)):
        distance = cKDTree(b.triangles_center).query(a.triangles_center)[0]
        assert distance.max() < 1e-10
    assert expected.is_watertight and expected.is_winding_consistent
    assert expected.euler_number == 2
    assert len(expected.split(only_watertight=False)) == 1
    assert expected.area_faces.min() > 1e-16
    stage = Usd.Stage.Open(str(ROOT / "assets/robots/tesollo/dex01.usda"))
    collision = UsdGeom.Mesh(stage.GetPrimAtPath("/Sensor/collision"))
    assert np.array_equal(np.array(collision.GetFaceVertexIndicesAttr().Get()).reshape(-1, 3), expected.faces)
    assert np.max(np.abs(np.array(collision.GetPointsAttr().Get()) - expected.vertices)) < 2e-9


def test_material_seam_preserves_overmold_side_and_contact_region():
    import trimesh
    from scipy.spatial.transform import Rotation

    path = ROOT / "scripts/build_dex01_asset.py"
    spec = importlib.util.spec_from_file_location("_dex01_material_builder", path)
    builder = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(builder)
    points, faces, normals, materials = builder.load_exterior(ROOT / REFERENCE["outer_surface_path"])
    mesh = trimesh.Trimesh(points * 1000, faces, process=False)
    assert np.allclose(np.linalg.norm(normals, axis=2), 1, atol=1e-6)
    # Source CAD seam: front of rear flange x=1.5 mm; bottom of cover z=-11.43 mm.
    # Silver triangles must not bridge diagonally across the overmold side panel.
    front = (points[faces, 0].max(1) < 0.00149) & (points[faces, 2].min(1) > -0.01142)
    assert front.any() and np.all(materials[front] == 1)
    w, x, y, z = REFERENCE["mount_rotation_wxyz"]
    rotation = Rotation.from_quat([x, y, z, w]).as_matrix()
    with np.load(ROOT / REFERENCE["contact_pattern_path"], allow_pickle=False) as data:
        contact = data["poses"][:, :, 3] @ rotation.T + REFERENCE["mount_translation_m"]
    _, _, nearest = trimesh.proximity.closest_point(mesh, contact * 1000)
    assert np.all(materials[nearest] == 1)


def test_taxel_mapping_correction_has_ordered_front_rows():
    with np.load(PATTERN) as data:
        poses, pixels = data["poses"], data["taxel2pixel"]
    lookup = {tuple(pixel): pose for pixel, pose in zip(pixels, poses)}
    # Neighboring columns on the flat front surface must share physical row positions.
    for row in range(20, 32):
        y = [lookup[row, col][1, 3] for col in (14, 15, 16, 17, 18, 19, 20, 21)]
        assert np.ptp(y) < 5e-6
    for col in (16, 17):
        y = [lookup[row, col][1, 3] for row in range(20, 32)]
        assert np.all(np.diff(y) < -0.0008)
    # Adjacent-row swaps must never reverse direction on the flat center strip.
    for col in range(10, 22):
        y = [lookup[row, col][1, 3] for row in range(20, 32)]
        assert np.all(np.diff(y) < -0.0008)


def test_interior_layout_preserves_distribution_and_outer_projection():
    from scipy.spatial.transform import Rotation

    with np.load(PATTERN, allow_pickle=False) as source:
        original, pixels = source["poses"], source["taxel2pixel"]
    w, x, y, z = REFERENCE["mount_rotation_wxyz"]
    mount = Rotation.from_quat([x, y, z, w]).as_matrix()
    # Interior sites are derived from the original data with one constant
    # translation; there is no redundant stored copy or per-taxel deformation.
    shift = np.array(REFERENCE["registration_shift_in_sensor_m"])
    points = original[:, :, 3] @ mount.T + REFERENCE["mount_translation_m"] + shift
    normals = original[:, :, 2] @ mount.T
    # The shipped exterior preserves the original sites as buried samples;
    # projection to its surface must follow their outward sensing normals.
    with np.load(ROOT / REFERENCE["contact_pattern_path"], allow_pickle=False) as data:
        contact = data["poses"][:, :, 3] @ mount.T + REFERENCE["mount_translation_m"]
        assert np.array_equal(data["taxel2pixel"], pixels)
    projected = contact - points
    distance = np.linalg.norm(projected, axis=1)
    assert distance.min() > 0.00142 and distance.max() < 0.00145
    assert np.max(np.linalg.norm(np.cross(projected, normals), axis=1)) < 6e-6


def test_collision_surface_and_dg5f_mount_clearance():
    # This extended asset check uses USD and mesh tooling available in Isaac Sim.
    import pytest

    Usd = pytest.importorskip("pxr.Usd")
    UsdGeom = pytest.importorskip("pxr.UsdGeom")
    Gf = pytest.importorskip("pxr.Gf")
    trimesh = pytest.importorskip("trimesh")
    pytest.importorskip("rtree")
    stage = Usd.Stage.Open(str(ROOT / "assets/robots/tesollo/dg5f_right.usda"))
    stage.GetDefaultPrim().GetVariantSet("fingertip").SetVariantSelection("dex01")
    cache = UsdGeom.XformCache()
    w, x, y, z = REFERENCE["mount_rotation_wxyz"]
    rotation = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    for finger in range(1, 6):
        path = f"/Root/rl_dg_{finger}_sensor"
        visual = UsdGeom.Mesh(stage.GetPrimAtPath(path + "/visual"))
        collision = UsdGeom.Mesh(stage.GetPrimAtPath(path + "/collision"))
        vertices = np.array(visual.GetPointsAttr().Get())
        hull = trimesh.Trimesh(
            np.array(collision.GetPointsAttr().Get()),
            np.array(collision.GetFaceVertexIndicesAttr().Get()).reshape(-1, 3),
            process=False,
        )
        assert hull.is_watertight
        # Registered samples lie on the same exterior used for rendering/physics.
        with np.load(ROOT / REFERENCE["contact_pattern_path"]) as data:
            contact = data["poses"][:, :, 3] @ rotation.T + REFERENCE["mount_translation_m"]
            contact_normals = data["poses"][:, :, 2] @ rotation.T
            assert np.array_equal(data["taxel2pixel"], np.load(PATTERN)["taxel2pixel"])
        hull_mm = trimesh.Trimesh(hull.vertices * 1000, hull.faces, process=False)
        _, error, nearest_face = trimesh.proximity.closest_point(hull_mm, contact * 1000)
        assert error.max() / 1000 < 5e-9
        assert np.allclose(np.linalg.norm(contact_normals, axis=1), 1, atol=1e-6)
        assert np.min((hull_mm.face_normals[nearest_face] * contact_normals).sum(1)) > 0.999
        body = stage.GetPrimAtPath(f"/Root/rl_dg_{finger}_4")
        assert not any("sensor_holder" in str(p.GetPath()) for p in Usd.PrimRange(body))
        joint = stage.GetPrimAtPath(path + "/attach_joint")
        matrix = Gf.Matrix4d(1)
        matrix.SetRotate(Gf.Rotation(Gf.Quatd(joint.GetAttribute("physics:localRot1").Get())))
        matrix.SetTranslateOnly(Gf.Vec3d(joint.GetAttribute("physics:localPos1").Get()))
        inv = cache.GetLocalToWorldTransform(body).GetInverse()
        bases = [p for p in Usd.PrimRange(body) if p.IsA(UsdGeom.Mesh) and "fingertip_base" in str(p.GetPath())]
        assert len(bases) == 1
        base = bases[0]
        xf = cache.GetLocalToWorldTransform(base)
        coords = np.array([
            matrix.Transform(inv.Transform(xf.Transform(Gf.Vec3d(p)))) for p in UsdGeom.Mesh(base).GetPointsAttr().Get()
        ])
        assert coords[:, 0].min() - vertices[:, 0].max() > 0.00029
