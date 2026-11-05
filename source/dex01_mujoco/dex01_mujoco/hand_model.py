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

"""Convert the DG-5F USD articulation (with DEX-01 fingertips) into an MJCF body tree.

The USD asset is a flat PhysX articulation: every link is a sibling under ``/Root`` and the tree is
encoded by joints with ``localPos0/localRot0`` (frame in body0) and ``localPos1/localRot1`` (frame
in body1). MJCF needs a nested tree, so each child is placed at ``body0 * pose0 * pose1^-1`` and its
joint sits at ``pose1`` in the child frame. Mesh points are baked into their body frame and written
as STL files next to the generated XML.
"""

from __future__ import annotations

import hashlib
import json
import shutil
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[3]
HAND_USD = REPO_ROOT / "assets/robots/tesollo/dg5f_right.usda"
OUTPUT_DIR = REPO_ROOT / "assets/robots/tesollo/mjcf"
HAND_XML = OUTPUT_DIR / "dg5f_right_dex01.xml"
_STAMP = OUTPUT_DIR / "build_stamp.json"
_AXES = {"X": np.array([1.0, 0, 0]), "Y": np.array([0, 1.0, 0]), "Z": np.array([0, 0, 1.0])}


def _quat_to_mat(wxyz):
    w, x, y, z = wxyz
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _mat_to_quat(R):
    from scipy.spatial.transform import Rotation

    x, y, z, w = Rotation.from_matrix(R).as_quat()
    return np.array([w, x, y, z])


def _pose(pos, quat):
    T = np.eye(4)
    T[:3, :3] = _quat_to_mat([quat.GetReal(), *quat.GetImaginary()])
    T[:3, 3] = list(pos)
    return T


def _gf_to_np(m):
    # Gf matrices use row vectors (p' = p M); transpose to the column-vector convention.
    return np.array(m, dtype=float).T


DEFAULT_RGBA = (0.62, 0.63, 0.65, 1.0)  # meshes without a material, e.g. the distal fingertip_base


def _material_rgba(material):
    """Diffuse colour of an OmniPBR or UsdPreviewSurface material, or None."""
    if not material:
        return None
    from pxr import Usd

    for shader in Usd.PrimRange(material.GetPrim()):
        for name in ("inputs:diffuse_color_constant", "inputs:diffuseColor"):
            attr = shader.GetAttribute(name)
            if attr and attr.Get() is not None:
                return (*attr.Get(), 1.0)
    return None


def _fmt(values):
    return " ".join(f"{v:.9g}" for v in values)


def _source_hash():
    digest = hashlib.sha256(HAND_USD.read_bytes())
    digest.update(Path(__file__).read_bytes())
    return digest.hexdigest()


def _triangulate(mesh):
    """Fan-triangulate a UsdGeom.Mesh; returns points, triangles and each triangle's source polygon."""
    points = np.asarray(mesh.GetPointsAttr().Get(), dtype=float)
    counts = np.asarray(mesh.GetFaceVertexCountsAttr().Get())
    indices = np.asarray(mesh.GetFaceVertexIndicesAttr().Get())
    faces, polygon, offset = [], [], 0
    for face, count in enumerate(counts):
        for k in range(1, count - 1):
            faces.append((indices[offset], indices[offset + k], indices[offset + k + 1]))
            polygon.append(face)
        offset += count
    return points, np.asarray(faces), np.asarray(polygon)


def _material_parts(prim, polygon):
    """(triangle mask, rgba) per material subset, or one part for the whole mesh."""
    from pxr import UsdGeom, UsdShade

    parts = []
    for subset in UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(prim)):
        material = UsdShade.MaterialBindingAPI(subset.GetPrim()).ComputeBoundMaterial()[0]
        parts.append((np.isin(polygon, subset.GetIndicesAttr().Get()), _material_rgba(material)))
    if not parts:
        material = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
        parts = [(np.ones(len(polygon), bool), _material_rgba(material))]
    return parts


def _is_rendered(prim):
    from pxr import UsdGeom

    imageable = UsdGeom.Imageable(prim)
    return imageable.ComputeVisibility() != UsdGeom.Tokens.invisible and imageable.ComputePurpose() in (
        UsdGeom.Tokens.default_,
        UsdGeom.Tokens.render,
    )


class _HandConverter:
    """Reads the USD articulation and writes the MJCF body tree and its STL meshes."""

    def __init__(self, stage, mesh_dir):
        from pxr import UsdGeom

        self.stage = stage
        self.mesh_dir = mesh_dir
        self.xform_cache = UsdGeom.XformCache()
        self.assets = []
        self.bodies = self._read_bodies()
        self.children, parents = self._read_joints()
        self.roots = [path for path in self.bodies if path not in parents]
        if len(self.roots) != 1:
            raise RuntimeError(f"Expected one articulation root, found {self.roots}")

    def _read_bodies(self):
        from pxr import UsdPhysics

        bodies = {}
        for prim in self.stage.Traverse():
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                mass_attr = prim.GetAttribute("physics:mass")
                bodies[str(prim.GetPath())] = {
                    "prim": prim,
                    "name": prim.GetName(),
                    "mass": float(mass_attr.Get()) if mass_attr and mass_attr.Get() else 0.01,
                    "authored": _gf_to_np(self.xform_cache.GetLocalToWorldTransform(prim)),
                }
        return bodies

    def _read_joints(self):
        from pxr import UsdPhysics

        children = {path: [] for path in self.bodies}
        parents = {}
        for prim in self.stage.Traverse():
            if not prim.IsA(UsdPhysics.Joint):
                continue
            joint = UsdPhysics.Joint(prim)
            targets0, targets1 = joint.GetBody0Rel().GetTargets(), joint.GetBody1Rel().GetTargets()
            if not targets0 or not targets1:
                continue  # world anchor of the fixed wrist; the scene places the root body instead
            body0, body1 = str(targets0[0]), str(targets1[0])
            entry = {
                "name": prim.GetName(),
                "child": body1,
                "pose0": _pose(joint.GetLocalPos0Attr().Get(), joint.GetLocalRot0Attr().Get()),
                "pose1": _pose(joint.GetLocalPos1Attr().Get(), joint.GetLocalRot1Attr().Get()),
                "type": "fixed",
            }
            if prim.IsA(UsdPhysics.RevoluteJoint):
                revolute = UsdPhysics.RevoluteJoint(prim)
                entry["type"] = "hinge"
                entry["axis"] = _AXES[revolute.GetAxisAttr().Get()]
                entry["range"] = np.radians([revolute.GetLowerLimitAttr().Get(), revolute.GetUpperLimitAttr().Get()])
            children[body0].append(entry)
            parents[body1] = body0
        return children, parents

    def _write_mesh(self, name, points, faces):
        import trimesh

        trimesh.Trimesh(points, faces, process=False).export(self.mesh_dir / f"{name}.stl")
        self.assets.append(f'    <mesh name="{name}" file="meshes/{name}.stl"/>')

    def _body_meshes(self, body):
        from pxr import Usd, UsdGeom, UsdPhysics

        prims = Usd.PrimRange(body["prim"], Usd.TraverseInstanceProxies())
        for prim in prims:
            if prim != body["prim"] and prim.HasAPI(UsdPhysics.RigidBodyAPI):
                prims.PruneChildren()
            elif prim.IsA(UsdGeom.Mesh):
                yield prim

    @staticmethod
    def _is_collision(prim, body_prim):
        from pxr import UsdPhysics

        while prim and prim != body_prim:
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                return True
            prim = prim.GetParent()
        return False

    def body_geoms(self, body):
        """(mesh name, is_collision, rgba) for every mesh of a body, baked into the body frame."""
        from pxr import UsdGeom

        body_inv = np.linalg.inv(body["authored"])
        geoms = []
        for prim in self._body_meshes(body):
            points, faces, polygon = _triangulate(UsdGeom.Mesh(prim))
            if len(points) < 4 or len(faces) == 0:
                continue
            to_body = body_inv @ _gf_to_np(self.xform_cache.GetLocalToWorldTransform(prim))
            points = points @ to_body[:3, :3].T + to_body[:3, 3]
            collision = self._is_collision(prim, body["prim"])
            stem = f"{body['name']}_{'col' if collision else 'vis'}_{len(geoms)}"
            self._write_mesh(stem, points, faces)
            if collision:
                geoms.append((stem, True, None))
            # Collision meshes are usually guides or invisible, but some (the DEX-01 variant's distal
            # fingertip_base) are the link's only rendered geometry, so they are drawn as well.
            if not _is_rendered(prim):
                continue
            # Draw each material subset as its own geom so the parts keep their colours.
            parts = _material_parts(prim, polygon)
            for k, (mask, rgba) in enumerate(parts):
                if not mask.any():
                    continue
                name = stem if len(parts) == 1 else f"{stem}_{k}"
                if name != stem:
                    self._write_mesh(name, points, faces[mask])
                geoms.append((name, False, rgba or DEFAULT_RGBA))
        return geoms

    def emit(self, path, pose_in_parent, joint, depth):
        body = self.bodies[path]
        pad = "  " * depth
        lines = [
            f'{pad}<body name="{body["name"]}" pos="{_fmt(pose_in_parent[:3, 3])}"'
            f' quat="{_fmt(_mat_to_quat(pose_in_parent[:3, :3]))}" gravcomp="1">'
        ]
        if joint is not None and joint["type"] == "hinge":
            axis = joint["pose1"][:3, :3] @ joint["axis"]
            lines.append(
                f'{pad}  <joint name="{joint["name"]}" pos="{_fmt(joint["pose1"][:3, 3])}" axis="{_fmt(axis)}"'
                f' range="{_fmt(joint["range"])}"/>'
            )
        geoms = self.body_geoms(body)
        collisions = [g for g in geoms if g[1]]
        for name, collision, rgba in geoms:
            if collision:
                mass = body["mass"] / len(collisions)
                lines.append(f'{pad}  <geom class="collision" mesh="{name}" mass="{mass:.9g}"/>')
            else:
                lines.append(f'{pad}  <geom class="visual" mesh="{name}" rgba="{_fmt(rgba)}"/>')
        if not collisions:
            lines.append(f'{pad}  <inertial pos="0 0 0" mass="{body["mass"]:.9g}" diaginertia="1e-6 1e-6 1e-6"/>')
        if "_sensor" in body["name"]:
            lines.append(f'{pad}  <site name="{body["name"]}" size="0.001"/>')
        for child in self.children[path]:
            lines += self.emit(child["child"], child["pose0"] @ np.linalg.inv(child["pose1"]), child, depth + 1)
        lines.append(f"{pad}</body>")
        return lines


def build_hand_mjcf(force: bool = False) -> Path:
    """Write the hand MJCF (cached by input hash) and return its path."""
    source_hash = _source_hash()
    if not force and HAND_XML.exists() and _STAMP.exists():
        if json.loads(_STAMP.read_text()).get("source_sha256") == source_hash:
            return HAND_XML

    from pxr import Usd

    if HAND_USD.stat().st_size < 10_000:
        raise RuntimeError(f"{HAND_USD} looks like a git-lfs pointer; run 'git lfs pull' first")

    stage = Usd.Stage.Open(str(HAND_USD))
    stage.GetDefaultPrim().GetVariantSets().GetVariantSet("fingertip").SetVariantSelection("dex01")
    mesh_dir = OUTPUT_DIR / "meshes"
    shutil.rmtree(mesh_dir, ignore_errors=True)  # drop meshes from earlier builds
    mesh_dir.mkdir(parents=True)
    converter = _HandConverter(stage, mesh_dir)
    tree = converter.emit(converter.roots[0], np.eye(4), None, 2)
    xml = "\n".join([
        '<mujoco model="dg5f_right_dex01">',
        '  <compiler angle="radian" meshdir="." inertiafromgeom="true"/>',
        "  <default>",
        '    <default class="visual"><geom type="mesh" contype="0" conaffinity="0" group="1" density="0"/></default>',
        # Hand geoms collide with objects (contype 1) but not with each other (contype 2 only).
        (
            '    <default class="collision"><geom type="mesh" contype="2" conaffinity="1" group="3" rgba="0.5 0.5 0.5'
            ' 0.3"/></default>'
        ),
        "  </default>",
        "  <asset>",
        *converter.assets,
        "  </asset>",
        "  <worldbody>",
        *tree,
        "  </worldbody>",
        "</mujoco>",
        "",
    ])
    HAND_XML.write_text(xml)
    _STAMP.write_text(json.dumps({"source_sha256": source_hash}) + "\n")
    return HAND_XML


if __name__ == "__main__":
    print(build_hand_mjcf(force=True))
