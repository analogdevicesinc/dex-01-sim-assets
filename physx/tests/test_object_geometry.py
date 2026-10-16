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

"""Check support placement for meshes whose body origins are offset."""

import importlib.util
from pathlib import Path

import numpy as np
import pytest


@pytest.mark.parametrize("scale", [0.15, 0.19671368598937988, 1.0])
def test_lower_face_center_includes_offsets_and_inherited_scale(scale):
    Usd = pytest.importorskip("pxr.Usd")
    from pxr import Gf, UsdGeom, UsdPhysics

    path = Path(__file__).resolve().parents[1] / "scripts/object_geometry.py"
    spec = importlib.util.spec_from_file_location("_object_geometry", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    stage = Usd.Stage.CreateInMemory()
    parent = UsdGeom.Xform.Define(stage, "/Asset")
    parent.AddTranslateOp().Set(Gf.Vec3d(0.3, -0.1, 0.5))
    parent.AddRotateXYZOp().Set(Gf.Vec3f(10, 30, 70))
    parent.AddScaleOp().Set(Gf.Vec3d(scale))
    UsdGeom.Xform.Define(stage, "/Asset/gear")
    mesh = UsdGeom.Mesh.Define(stage, "/Asset/gear/collision")
    # The source gear has an off-center origin in both x and z.
    mesh.CreatePointsAttr([(-0.0007, -0.021, 0.005), (0.0412, 0.021, 0.030), (-0.0007, 0.021, 0.005)])
    mesh.CreateFaceVertexCountsAttr([3])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2])
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    actual = module.lower_face_center(stage, "/Asset/gear")
    assert np.allclose(actual, np.array([0.02025, 0.0, 0.005]) * scale, atol=1e-9)


def test_projected_underside_retains_the_hole():
    pytest.importorskip("pxr.Usd")
    path = Path(__file__).resolve().parents[1] / "scripts/object_geometry.py"
    spec = importlib.util.spec_from_file_location("_object_geometry", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    triangles = []
    for left, bottom, right, top in [(-1, -1, 1, -0.5), (-1, 0.5, 1, 1), (-1, -0.5, -0.5, 0.5), (0.5, -0.5, 1, 0.5)]:
        a, b, c, d = [left, bottom], [right, bottom], [right, top], [left, top]
        triangles.extend([[a, b, c], [a, c, d]])
    triangles.append([[0, 0], [0, 0], [0, 0]])
    points = np.array([[0, 0], [0.75, 0], [0, 0.75], [-0.75, -0.75], [1.1, 0]])
    assert module.projected_face_mask(points, np.array(triangles)).tolist() == [False, True, True, True, False]
