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

"""Capture real example motion and force arrays for documentation videos.

The source examples run unchanged in Isaac Sim 5.1. Rendering is performed
separately from these recorded physical states; no motion is interpolated.
"""

import argparse
import hashlib
import inspect
import json
import runpy
import shutil
import subprocess
import sys
import tempfile
import traceback
from pathlib import Path
from types import SimpleNamespace

PRESETS = {
    "flat-pad": ("simple_example.py", 100, 2, 0),
    "hand": ("dex01_on_tesollo_dg5f.py", 1200, 12, 0),
    "stationary-press": ("dex01_on_tesollo_dg5f.py", 900, 4, 600),
    "robotiq": ("dex01_on_robotiq_2f85.py", 1440, 8, 0),
}


def simulation_source_hashes(repo, example=None):
    """Identify the actual simulation inputs, including uncommitted runtime edits."""
    package = repo / "source/dex01_sim_asset/dex01_sim_asset"
    paths = list(package.rglob("*.py")) + list(package.rglob("*.npz"))
    paths += [
        repo / name
        for name in (
            "scripts/simple_example.py",
            "scripts/dex01_on_tesollo_dg5f.py",
            "scripts/dex01_articulated_press.py",
            "scripts/object_geometry.py",
            "scripts/docs/capture_examples.py",
            "scripts/docs/motion_geometry.py",
            "assets/robots/tesollo/dg5f_right.usda",
            "assets/robots/tesollo/dex01.usda",
            "assets/sensors/dex01/reference.json",
        )
    ]
    if example == "robotiq":
        paths += [
            repo / name
            for name in (
                "scripts/dex01_on_robotiq_2f85.py",
                "scripts/build_robotiq_asset.py",
                "assets/robots/robotiq_2f85/finger.usda",
                "assets/robots/robotiq_2f85/reference.json",
            )
        ]
    return {str(path.relative_to(repo)): hashlib.sha256(path.read_bytes()).hexdigest() for path in sorted(paths)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--example", choices=PRESETS, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()
    output = args.output.resolve()
    if args.worker:
        capture(args.example, output)
        return
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="dex01-example-", dir=output.parent) as tmp:
        staging = Path(tmp)
        run = subprocess.run(
            [
                sys.executable,
                "-u",
                str(Path(__file__).resolve()),
                "--worker",
                "--example",
                args.example,
                "--output",
                str(staging),
            ],
            check=False,
        )
        if run.returncode or (staging / "failure.txt").exists() or not (staging / "capture.json").exists():
            error = (
                (staging / "failure.txt").read_text() if (staging / "failure.txt").exists() else "No complete capture"
            )
            raise RuntimeError(f"Example capture failed (exit {run.returncode}): {error}")
        manifest = json.loads((staging / "capture.json").read_text())
        for name, expected in manifest["files"].items():
            if hashlib.sha256((staging / name).read_bytes()).hexdigest() != expected:
                raise RuntimeError(f"Capture hash mismatch: {name}")
        output.mkdir(parents=True, exist_ok=True)
        for file in staging.iterdir():
            shutil.copy2(file, output / file.name)
    print(f"Verified {args.example} example capture: {output}", flush=True)


def load_example(repo, example, output):
    """Initialize each example's application before importing simulator modules."""
    demo_name, max_steps, _, _ = PRESETS[example]
    sys.argv = [str(repo / "scripts" / demo_name), "--headless", "--max-steps", str(max_steps)]
    if example == "stationary-press":
        sys.argv += ["--stationary-press", "--verification-report", str(output / "press-report.json")]
    # Definitions and AppLauncher run, but the demo's standalone block does not.
    module = runpy.run_path(sys.argv[0], run_name="_docs_video_capture")
    if example == "robotiq":
        from isaaclab.app import AppLauncher

        launcher = AppLauncher({"headless": True, "device": "cuda:0"})
        module["simulation_app"] = launcher.app
    return module


def capture(example, output):
    repo = Path(__file__).resolve().parents[2]
    demo_name, max_steps, stride, start = PRESETS[example]
    sys.path.insert(0, str(repo / "scripts"))
    sys.path.insert(0, str(repo / "source/dex01_sim_asset"))
    output.mkdir(parents=True, exist_ok=True)
    module = load_example(repo, example, output)
    robotiq_report = {"status": "running", "observations": []}
    from importlib.metadata import version

    import dex01_sim_asset
    import numpy as np
    import omni.usd
    from isaaclab.scene import InteractiveScene
    from motion_geometry import apply_body_poses, capture_geometry, verify_geometry
    from pxr import Usd

    if version("isaacsim") != "5.1.0.0":
        raise RuntimeError("Documentation example captures require the validated Isaac Sim 5.1.0.0 runtime")
    if Path(dex01_sim_asset.__file__).resolve().parent != repo / "source/dex01_sim_asset/dex01_sim_asset":
        raise RuntimeError("The simulator imported dex01_sim_asset from another checkout")
    source_hashes = simulation_source_hashes(repo, example)
    original_update = InteractiveScene.update
    recorded = []
    paths, reference_stage, dt = None, None, None
    pixels, labels = None, None
    meshes, geometry = None, None

    def record(scene, time_step, *args, **kwargs):
        nonlocal paths, reference_stage, dt, pixels, labels, meshes, geometry
        original_update(scene, time_step, *args, **kwargs)
        caller = inspect.currentframe().f_back
        if caller.f_code.co_name not in {"main", "run", "run_simulator", "run_articulated_press"}:
            return
        step = caller.f_locals.get("step")
        if step is None or step < start or step % stride or step >= max_steps:
            return
        dt = time_step
        if example == "flat-pad":
            sensors = [scene["sensor"]]
        elif example == "robotiq":
            sensors = [scene["left_sensor"], scene["right_sensor"]]
        else:
            sensors = [scene[f"finger{i}_sensor"] for i in range(1, 6)]
        current = {}
        if example == "flat-pad":
            pad = scene["finger_pad"]
            for view in [pad.root_physx_view, scene["asset"].root_physx_view]:
                for path, pose in zip(view.prim_paths, view.get_transforms().cpu().numpy()):
                    current[path] = pose
        else:
            robot = scene["robot"]
            for i, name in enumerate(robot.body_names):
                position = robot.data.body_pos_w[0, i].cpu().numpy()
                quat = robot.data.body_quat_w[0, i].cpu().numpy()
                prefix = "/World/envs/env_0/Robot/Robotiq_2F_85" if example == "robotiq" else "/World/envs/env_0/Robot"
                current[f"{prefix}/{name}"] = np.concatenate([position, quat[[1, 2, 3, 0]]])
            view = scene["block" if example == "robotiq" else "asset"].root_physx_view
            for path, pose in zip(view.prim_paths, view.get_transforms().cpu().numpy()):
                current[path] = pose
            if example != "robotiq":
                for i, sensor in enumerate(sensors, 1):
                    current[f"/World/envs/env_0/Robot/rl_dg_{i}_sensor"] = (
                        sensor._view.get_transforms()[0].cpu().numpy()
                    )
        frames = []
        totals = []
        for sensor in sensors:
            image = sensor.get_tactile_image()[0].cpu().numpy()
            taxels = sensor.data.normal_forces[0].cpu().numpy()
            mapping = sensor.taxel2pixel.cpu().numpy()
            if not np.array_equal(image[mapping[:, 0], mapping[:, 1]], taxels):
                raise RuntimeError("Frame mapping changed during example motion")
            if not np.isfinite(image).all() or (image < 0).any():
                raise RuntimeError("Invalid physical force array")
            frames.append(image)
            totals.append(float(taxels.sum()))
        if reference_stage is None:
            stage = omni.usd.get_context().get_stage()
            reference_stage = Usd.Stage.Open(stage.Flatten())
            paths = list(current)
            meshes, geometry = capture_geometry(reference_stage, paths)
            pixels = sensors[0].taxel2pixel.cpu().numpy()
            labels = {"flat-pad": ["Pad"], "robotiq": ["Left", "Right"]}.get(
                example, [f"Finger {i}" for i in range(1, 6)]
            )
        if set(paths) != set(current):
            raise RuntimeError("Rigid body set changed during capture")
        phase = caller.f_locals.get("phase", "drop / contact")
        if example == "flat-pad":
            phase = "falling" if totals[0] < 0.01 else ("impact" if step < 20 else "settling / contact")
        if example == "hand":
            phase = f"Finger {module['PRESSED_FINGERS'][step // module['STEPS_PER_FINGER']]} contact"
        recorded.append({
            "step": step,
            "phase": phase,
            "poses": np.stack([current[p] for p in paths]),
            "frames": np.stack(frames),
            "taxel_points_w": np.stack([sensor.data.taxel_points_w[0].cpu().numpy().copy() for sensor in sensors]),
        })

    InteractiveScene.update = record
    try:
        if example == "robotiq":
            config = SimpleNamespace(
                device="cuda:0",
                gripper_usd=None,
                asset_root=None,
                max_steps=max_steps,
                num_envs=1,
                debug_vis=False,
                no_tactile_ui=True,
                headless=True,
                livestream=0,
            )
            module["run"](config, module["simulation_app"], robotiq_report)
            robotiq_report["status"] = "complete"
            report_path = output / "grasp-report.json"
            report_path.write_text(json.dumps(robotiq_report, indent=2) + "\n")
            module["check_report"](report_path)
        else:
            module["main"]()
        if simulation_source_hashes(repo, example) != source_hashes:
            raise RuntimeError("Simulation sources changed during capture; rerun with stable inputs")
        if not recorded:
            raise RuntimeError("No physical example frames captured")
        if example == "stationary-press":
            from dex01_articulated_press import check_articulated_report

            check_articulated_report(json.loads((output / "press-report.json").read_text()))
        first = recorded[0]
        apply_body_poses(reference_stage, paths, first["poses"], geometry["body_scales"])
        _, vertex_error = verify_geometry(reference_stage, paths, first["poses"], meshes, geometry)
        reference_stage.GetRootLayer().Export(str(output / "scene.usda"))
        np.savez_compressed(output / "geometry.npz", **geometry)
        np.savez_compressed(
            output / "motion.npz",
            poses=np.stack([r["poses"] for r in recorded]),
            frames=np.stack([r["frames"] for r in recorded]),
            taxel2pixel=pixels,
            taxel_points_w=np.stack([r["taxel_points_w"] for r in recorded]),
            simulation_time_s=np.array([(r["step"] + 1) * dt for r in recorded]),
        )
        manifest = {
            "example": example,
            "physics_version": version("isaacsim"),
            "physics_dt_s": dt,
            "steps": [r["step"] for r in recorded],
            "phases": [r["phase"] for r in recorded],
            "body_paths": paths,
            "meshes": meshes,
            "body_scales": geometry["body_scales"].tolist(),
            "maximum_exported_vertex_error_m": vertex_error,
            "geometry_script_sha256": hashlib.sha256(
                Path(__file__).with_name("motion_geometry.py").read_bytes()
            ).hexdigest(),
            "labels": labels,
            "frame_count": len(recorded),
            "capture_stride": stride,
            "playback_fps": 1 / (dt * stride),
            "source_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=repo, text=True).strip(),
            "source_files_sha256": source_hashes,
            "demo_sha256": hashlib.sha256((repo / "scripts" / demo_name).read_bytes()).hexdigest(),
            "capture_script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            "hold_pacing_s": 0.0,
            "files": {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in output.iterdir() if p.is_file()},
        }
        if example == "robotiq":
            manifest["resolved_gripper_asset"] = robotiq_report["resolved_gripper_asset"]
            manifest["grasp_report_verified"] = True
        (output / "capture.json").write_text(json.dumps(manifest, indent=2) + "\n")
        print(f"Captured {len(recorded)} {example} frames with exact matching force data", flush=True)
    except BaseException:
        text = traceback.format_exc()
        (output / "failure.txt").write_text(text)
        print(text, file=sys.stderr, flush=True)
        raise
    finally:
        InteractiveScene.update = original_update
        from isaaclab.sim import SimulationContext

        sim = SimulationContext.instance()
        if sim is not None:
            sim._disable_app_control_on_stop_handle = True
        module["simulation_app"].close()


if __name__ == "__main__":
    main()
