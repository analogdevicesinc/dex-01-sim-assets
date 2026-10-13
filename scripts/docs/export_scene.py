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

"""Capture the current PhysX hand scene and tactile frames without camera rendering.

Run in the installed Isaac Lab environment from any directory. The exported
state can be rendered separately; it is not a simulator restart checkpoint.
"""

import argparse
import hashlib
import json
import runpy
import shutil
import subprocess
import sys
import tempfile
import traceback
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--steps", type=int, default=240)
    parser.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.steps < 1:
        parser.error("--steps must be positive")
    if not args.worker:
        capture_checked(args.output.resolve(), args.steps)
        return
    repo = Path(__file__).resolve().parents[2]
    demo = repo / "scripts/dex01_on_tesollo_dg5f.py"
    args.output.mkdir(parents=True, exist_ok=True)
    sys.path.insert(0, str(demo.parent))
    sys.argv = [str(demo), "--headless", "--flat-indenter", "--max-steps", str(args.steps)]
    # The demo starts AppLauncher and defines its scene, but this module name
    # intentionally does not enter its standalone main block.
    module = runpy.run_path(str(demo), run_name="_docs_capture")
    namespace = module["main"].__globals__
    original_run = namespace["run_simulator"]

    def capture(sim, scene):
        original_run(sim, scene)
        export_state(scene, args.output, repo, args.steps)

    namespace["run_simulator"] = capture
    try:
        module["main"]()
    except BaseException:
        # Kit's fast shutdown may terminate Python before it propagates an
        # exception. Leave an explicit failure marker for the parent process.
        failure = traceback.format_exc()
        (args.output / "failure.txt").write_text(failure)
        print(failure, file=sys.stderr, flush=True)
        raise
    finally:
        import isaaclab.sim as sim_utils

        context = sim_utils.SimulationContext.instance()
        if context is not None:
            context._disable_app_control_on_stop_handle = True
        module["simulation_app"].close()


def capture_checked(output, steps):
    """Accept a child capture only after process and artifact checks agree."""
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="dex01-capture-", dir=output.parent) as directory:
        staging = Path(directory)
        result = subprocess.run(
            [
                sys.executable,
                "-u",
                str(Path(__file__).resolve()),
                "--worker",
                "--output",
                str(staging),
                "--steps",
                str(steps),
            ],
            check=False,
        )
        failure = staging / "failure.txt"
        if result.returncode or failure.exists() or not (staging / "manifest.json").exists():
            detail = failure.read_text() if failure.exists() else "The worker did not produce a complete capture."
            raise RuntimeError(f"Capture worker failed (exit {result.returncode}): {detail}")
        manifest = json.loads((staging / "manifest.json").read_text())
        if manifest["sensor_visual_count"] != 5 or max(manifest["taxel_surface_max_error_m"]) > 1e-6:
            raise RuntimeError("Captured fingertip geometry did not pass verification")
        for name, field in [("scene.usda", "scene_sha256"), ("tactile.npz", "tactile_sha256")]:
            if hashlib.sha256((staging / name).read_bytes()).hexdigest() != manifest[field]:
                raise RuntimeError(f"Capture worker produced an inconsistent {name}")
        output.mkdir(parents=True, exist_ok=True)
        for name in ("scene.usda", "tactile.npz", "manifest.json"):
            shutil.copy2(staging / name, output / name)
    print(f"Verified capture written to {output}", flush=True)


def export_state(scene, output, repo, steps):
    from importlib.metadata import version

    import numpy as np
    import omni.usd
    from pxr import Gf, Usd, UsdGeom

    physics_version = version("isaacsim")
    if physics_version != "5.1.0.0":
        raise RuntimeError(f"This capture is validated for Isaac Sim 5.1.0.0, found {physics_version}")
    stage = omni.usd.get_context().get_stage()
    robot = scene["robot"]
    poses = {}
    for i, name in enumerate(robot.body_names):
        poses[f"/World/envs/env_0/Robot/{name}"] = (
            robot.data.body_pos_w[0, i].cpu().numpy(),
            robot.data.body_quat_w[0, i].cpu().numpy(),
        )
    for finger in range(1, 6):
        # The sensor view includes the fixed-joint body pose from PhysX.
        pose = scene[f"finger{finger}_sensor"]._view.get_transforms()[0].cpu().numpy()
        poses[f"/World/envs/env_0/Robot/rl_dg_{finger}_sensor"] = (pose[:3], pose[[6, 3, 4, 5]])
    asset = scene["asset"]
    pose = asset.root_physx_view.get_transforms()[0].cpu().numpy()
    poses[asset.root_physx_view.prim_paths[0]] = (pose[:3], pose[[6, 3, 4, 5]])
    flat = Usd.Stage.Open(stage.Flatten())
    for path, (position, quaternion) in poses.items():
        prim = flat.GetPrimAtPath(path)
        if not prim.IsValid():
            raise RuntimeError(f"Body is missing from the exported stage: {path}")
        world = Gf.Matrix4d(1)
        world.SetRotate(Gf.Quatd(float(quaternion[0]), Gf.Vec3d(*map(float, quaternion[1:]))))
        world.SetTranslateOnly(Gf.Vec3d(*map(float, position)))
        parent = UsdGeom.Xformable(prim.GetParent()).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        xform = UsdGeom.Xformable(prim)
        xform.ClearXformOpOrder()
        xform.AddTransformOp().Set(world * parent.GetInverse())
    visible_sensors = [
        prim
        for prim in Usd.PrimRange(flat.GetPrimAtPath("/World/envs/env_0"), Usd.TraverseInstanceProxies())
        if prim.IsA(UsdGeom.Mesh)
        and "_sensor/visual" in str(prim.GetPath())
        and UsdGeom.Imageable(prim).ComputeVisibility() != "invisible"
    ]
    if len(visible_sensors) != 5:
        raise RuntimeError(f"Expected five visible sensor meshes, found {len(visible_sensors)}")
    flat.GetRootLayer().Export(str(output / "scene.usda"))
    arrays = {"taxel2pixel": scene["finger2_sensor"].taxel2pixel.cpu().numpy()}
    forces = []
    for finger in range(1, 6):
        sensor = scene[f"finger{finger}_sensor"]
        frame = sensor.get_tactile_image()[0].cpu().numpy()
        normal = sensor.data.normal_forces[0].cpu().numpy()
        pixels = arrays["taxel2pixel"]
        if not np.array_equal(frame[pixels[:, 0], pixels[:, 1]], normal):
            raise RuntimeError(f"Firmware mapping mismatch on finger {finger}")
        arrays[f"frame{finger}"] = frame
        arrays[f"points{finger}"] = sensor.data.taxel_points_w[0].cpu().numpy()
        forces.append(float(normal.sum()))
    # Mesh proximity is evaluated in millimetres to avoid metre-scale
    # tolerances inside trimesh merging distinct nearby vertices.
    import trimesh

    surface_errors = []
    for finger in range(1, 6):
        mesh = UsdGeom.Mesh(flat.GetPrimAtPath(f"/World/envs/env_0/Robot/rl_dg_{finger}_sensor/visual"))
        transform = UsdGeom.Xformable(mesh).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        vertices = np.array([transform.Transform(point) for point in mesh.GetPointsAttr().Get()])
        faces = np.array(mesh.GetFaceVertexIndicesAttr().Get()).reshape(-1, 3)
        surface = trimesh.Trimesh(vertices * 1000, faces, process=False)
        distance_m = surface.nearest.on_surface(arrays[f"points{finger}"] * 1000)[1] / 1000
        maximum = float(distance_m.max())
        if maximum > 1e-6:
            raise RuntimeError(f"Finger {finger} taxels are displaced from its visible surface: {maximum} m")
        surface_errors.append(maximum)
    np.savez_compressed(output / "tactile.npz", **arrays)
    manifest = {
        "physics_version": physics_version,
        "steps": steps,
        "indenter": "0.1 kg flat cube",
        "forces_n": forces,
        "sensor_visual_count": len(visible_sensors),
        "taxel_surface_max_error_m": surface_errors,
        "source_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=repo, text=True).strip(),
        "demo_sha256": hashlib.sha256((repo / "scripts/dex01_on_tesollo_dg5f.py").read_bytes()).hexdigest(),
        "scene_sha256": hashlib.sha256((output / "scene.usda").read_bytes()).hexdigest(),
        "tactile_sha256": hashlib.sha256((output / "tactile.npz").read_bytes()).hexdigest(),
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"Captured five fingertips and physical loads (N): {forces}", flush=True)


if __name__ == "__main__":
    main()
