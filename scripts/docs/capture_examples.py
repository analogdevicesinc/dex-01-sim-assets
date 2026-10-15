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
import time
import traceback
from pathlib import Path

PRESETS = {
    "flat-pad": ("simple_example.py", 100, 2, 0),
    "hand": ("dex01_on_tesollo_dg5f.py", 1200, 12, 0),
    "stationary-press": ("dex01_on_tesollo_dg5f.py", 900, 4, 600),
}


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


def capture(example, output):
    repo = Path(__file__).resolve().parents[2]
    demo_name, max_steps, stride, start = PRESETS[example]
    sys.path.insert(0, str(repo / "scripts"))
    output.mkdir(parents=True, exist_ok=True)
    sys.argv = [str(repo / "scripts" / demo_name), "--headless", "--max-steps", str(max_steps)]
    if example == "stationary-press":
        sys.argv += ["--stationary-press", "--verification-report", str(output / "press-report.json")]
    # Definitions and AppLauncher run, but the demo's standalone block does not.
    module = runpy.run_path(sys.argv[0], run_name="_docs_video_capture")
    from importlib.metadata import version

    import numpy as np
    import omni.usd
    from isaaclab.scene import InteractiveScene
    from motion_geometry import apply_body_poses, capture_geometry, verify_geometry
    from pxr import Usd

    if version("isaacsim") != "5.1.0.0":
        raise RuntimeError("Documentation example captures require the validated Isaac Sim 5.1.0.0 runtime")
    original_update = InteractiveScene.update
    recorded = []
    paths, reference_stage, dt = None, None, None
    pixels, labels = None, None
    meshes, geometry = None, None

    def record(scene, time_step, *args, **kwargs):
        nonlocal paths, reference_stage, dt, pixels, labels, meshes, geometry
        original_update(scene, time_step, *args, **kwargs)
        caller = inspect.currentframe().f_back
        if caller.f_code.co_name not in {"main", "run_simulator", "run_articulated_press"}:
            return
        step = caller.f_locals.get("step")
        if (
            example == "stationary-press"
            and step is not None
            and caller.f_locals.get("phase") == "hold"
            and caller.f_locals.get("cycle", 0) >= 4.5
        ):
            # Preserve physical integration while pacing the headless capture
            # enough for the demo's observed hold-duration check.
            time.sleep(0.025)
        if step is None or step < start or step % stride or step >= max_steps:
            return
        dt = time_step
        sensors = [scene["sensor"]] if example == "flat-pad" else [scene[f"finger{i}_sensor"] for i in range(1, 6)]
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
                current[f"/World/envs/env_0/Robot/{name}"] = np.concatenate([position, quat[[1, 2, 3, 0]]])
            view = scene["asset"].root_physx_view
            for path, pose in zip(view.prim_paths, view.get_transforms().cpu().numpy()):
                current[path] = pose
            for i, sensor in enumerate(sensors, 1):
                current[f"/World/envs/env_0/Robot/rl_dg_{i}_sensor"] = sensor._view.get_transforms()[0].cpu().numpy()
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
            labels = ["Pad"] if example == "flat-pad" else [f"Finger {i}" for i in range(1, 6)]
        if set(paths) != set(current):
            raise RuntimeError("Rigid body set changed during capture")
        phase = caller.f_locals.get("phase", "drop / contact")
        if example == "flat-pad":
            phase = "falling" if totals[0] < 0.01 else ("impact" if step < 20 else "settling / contact")
        if example == "hand":
            phase = f"Finger {module['PRESSED_FINGERS'][step // module['STEPS_PER_FINGER']]} contact"
        recorded.append(
            {"step": step, "phase": phase, "poses": np.stack([current[p] for p in paths]), "frames": np.stack(frames)}
        )

    InteractiveScene.update = record
    try:
        module["main"]()
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
            "demo_sha256": hashlib.sha256((repo / "scripts" / demo_name).read_bytes()).hexdigest(),
            "capture_script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            "hold_pacing_s": 0.025 if example == "stationary-press" else 0.0,
            "files": {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in output.iterdir() if p.is_file()},
        }
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
