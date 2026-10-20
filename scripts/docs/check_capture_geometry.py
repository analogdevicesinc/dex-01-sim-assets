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

"""Independently audit loaded taxels against the captured collision triangle mesh."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import trimesh
from render_examples import validate_motion
from scipy.spatial.transform import Rotation


def check_capture(capture, tolerance_m=1e-6):
    """Check captured positions without querying PhysX/SDF or advancing simulation."""
    if not np.isfinite(tolerance_m) or tolerance_m < 0:
        raise ValueError("Tolerance must be finite and nonnegative")
    manifest = json.loads((capture / "capture.json").read_text())
    for name, expected in manifest["files"].items():
        if hashlib.sha256((capture / name).read_bytes()).hexdigest() != expected:
            raise ValueError(f"Capture hash mismatch: {name}")
    with np.load(capture / "motion.npz", allow_pickle=False) as data:
        arrays = dict(data)
    validate_motion(manifest, arrays)
    points = arrays["taxel_points_w"]
    expected_shape = (*arrays["frames"].shape[:2], len(arrays["taxel2pixel"]), 3)
    if points.shape != expected_shape or not np.isfinite(points).all():
        raise ValueError("Missing or invalid world-space taxel positions")
    candidates = [
        (i, m) for i, m in enumerate(manifest["meshes"]) if "/Asset/" in m["path"] and m["path"].endswith("/collisions")
    ]
    if len(candidates) != 1:
        raise ValueError("Expected exactly one gear collision mesh")
    mesh_index, entry = candidates[0]
    with np.load(capture / "geometry.npz", allow_pickle=False) as geometry:
        if not np.all(geometry[f"counts_{mesh_index}"] == 3):
            raise ValueError("Expected collision triangles")
        mesh = trimesh.Trimesh(
            vertices=geometry[f"points_{mesh_index}"],
            faces=geometry[f"indices_{mesh_index}"].reshape(-1, 3),
            process=True,
        )
    if not mesh.is_volume or not all(component.is_volume for component in mesh.split(only_watertight=False)):
        raise ValueError("Collision mesh components must be closed and outward oriented")
    # trimesh ray tolerances assume ordinary model units; millimetre-sized
    # contacts in metre coordinates can be misclassified at triangle edges.
    # Scale both surfaces and queries, then convert distances back to metres.
    mesh.apply_scale(1000.0)
    pixels = arrays["taxel2pixel"]
    forces = arrays["frames"][:, :, pixels[:, 0], pixels[:, 1]]
    count, outside = 0, 0.0
    for index, pose in enumerate(arrays["poses"][:, entry["body_index"]]):
        loaded = points[index][forces[index] > 1e-8]
        if not len(loaded):
            continue
        local = Rotation.from_quat(pose[3:]).inv().apply(loaded - pose[:3]) * 1000.0
        # Ray parity determines containment. The nearest-face normal heuristic
        # used by signed_distance can flip the sign near triangulation edges.
        exterior = local[~mesh.contains(local)]
        if len(exterior):
            _, distance, _ = trimesh.proximity.closest_point(mesh, exterior)
            if not np.isfinite(distance).all():
                raise ValueError("Invalid independent mesh distances")
            outside = max(outside, float(distance.max()) / 1000.0)
        count += len(loaded)
    if count == 0:
        raise ValueError("Capture has no loaded taxels")
    if outside > tolerance_m:
        raise ValueError(f"Loaded taxel lies outside collision mesh by {outside:.9g} m")
    return {
        "example": manifest["example"],
        "capture_sha256": hashlib.sha256((capture / "capture.json").read_bytes()).hexdigest(),
        "checker_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "loaded_taxel_samples": count,
        "outside_mesh_max_distance_m": outside,
        "tolerance_m": tolerance_m,
        "gear_width_m": float(np.ptp(mesh.vertices[:, 1])) / 1000.0,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = check_capture(args.capture.resolve())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(f"Verified {result['loaded_taxel_samples']} loaded taxels against collision triangles")


if __name__ == "__main__":
    main()
