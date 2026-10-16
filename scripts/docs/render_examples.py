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

"""Render captured example motion as compact videos without rerunning physics."""

import argparse
import hashlib
import json
import shutil
import subprocess
import sys
import tempfile
from importlib.metadata import version
from pathlib import Path


def validate_motion(manifest, arrays):
    """Reject invalid timing, frame mapping, or unverified motion before rendering."""
    import numpy as np

    count = manifest["frame_count"]
    poses, frames, pixels, times = (arrays[k] for k in ("poses", "frames", "taxel2pixel", "simulation_time_s"))
    if poses.shape != (count, len(manifest["body_paths"]), 7) or frames.shape[:2] != (count, len(manifest["labels"])):
        raise ValueError("Incomplete recorded frame sequence")
    if len(manifest["phases"]) != count or times.shape != (count,) or count < 2:
        raise ValueError("Incomplete recorded timeline")
    if not np.isfinite(poses).all() or not np.isfinite(frames).all() or not np.isfinite(times).all():
        raise ValueError("Nonfinite captured data")
    if (frames < 0).any() or not np.allclose(np.linalg.norm(poses[..., 3:], axis=-1), 1, atol=1e-5):
        raise ValueError("Invalid physical forces or body orientations")
    expected_interval = manifest["physics_dt_s"] * manifest["capture_stride"]
    if not np.allclose(np.diff(times), expected_interval, rtol=0, atol=1e-9):
        raise ValueError("Captured frames are not uniformly timed")
    if pixels.shape != (740 if len(manifest["labels"]) == 5 else 1024, 2):
        raise ValueError("Captured taxel layout differs from the example")
    if not np.issubdtype(pixels.dtype, np.integer) or (pixels < 0).any() or (pixels >= 32).any():
        raise ValueError("Invalid firmware pixel indices")
    mask = np.zeros((32, 32), dtype=bool)
    mask[pixels[:, 0], pixels[:, 1]] = True
    if mask.sum() != len(pixels) or frames.shape[2:] != (32, 32) or (frames[:, :, ~mask] != 0).any():
        raise ValueError("Captured force frames contain invalid unoccupied cells")
    if manifest.get("backend") == "newton":
        steps = np.asarray(manifest["steps"])
        if not np.array_equal(np.diff(steps), np.full(count - 1, manifest["capture_stride"])):
            raise ValueError("Newton capture omits sampled ticks")
        if not np.allclose(times, (steps + 1) * manifest["physics_dt_s"], rtol=0, atol=1e-9):
            raise ValueError("Newton capture time differs from physics steps")
        taxels = arrays["taxel_forces_n"]
        if taxels.shape != (count, len(manifest["labels"]), len(pixels)) or not np.array_equal(
            frames[:, :, pixels[:, 0], pixels[:, 1]], taxels
        ):
            raise ValueError("Newton capture taxel forces differ from firmware frames")
    if manifest["example"] == "stationary-press":
        if (times[-1] - times[0]) > 2.5 or set(manifest["phases"]) != {"unloaded", "approach", "hold", "retract"}:
            raise ValueError("Expected one complete captured press cycle")
        stationary_paths = [
            "/World/envs/env_0/Robot/rl_dg_mount",
            (
                "/World/envs/env_0/Probe"
                if manifest.get("backend") == "newton"
                else "/World/envs/env_0/Asset/factory_gear_medium"
            ),
        ]
        for name in stationary_paths:
            index = manifest["body_paths"].index(name)
            if np.ptp(poses[:, index], axis=0).max() > 1e-5:
                raise ValueError("The wrist or stationary gear moves during the captured press")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    capture = args.capture.resolve()
    manifest = json.loads((capture / "capture.json").read_text())
    if not manifest.get("meshes") or "geometry.npz" not in manifest["files"]:
        raise RuntimeError("Capture lacks physical geometry and scale evidence; recapture with capture_examples.py")
    for name, expected in manifest["files"].items():
        if hashlib.sha256((capture / name).read_bytes()).hexdigest() != expected:
            raise RuntimeError(f"Recorded example hash mismatch: {name}")
    if manifest.get("backend") == "newton":
        import numpy as np

        required = {"scene.usda", "geometry.npz", "motion.npz"}
        if not required.issubset(manifest["files"]):
            raise RuntimeError("Newton capture is missing required files")
        with np.load(capture / "motion.npz", allow_pickle=False) as arrays:
            validate_motion(manifest, arrays)
    if not args.worker:
        with tempfile.TemporaryDirectory(prefix="dex01-example-render-", dir=args.output.resolve().parent) as tmp:
            staging = Path(tmp)
            result = subprocess.run(
                [
                    sys.executable,
                    "-u",
                    str(Path(__file__).resolve()),
                    "--worker",
                    "--capture",
                    str(capture),
                    "--output",
                    str(staging),
                ],
                check=False,
            )
            evidence = staging / (manifest["example"] + ".json")
            if result.returncode or (staging / "failure.txt").exists() or not evidence.exists():
                failure = (
                    (staging / "failure.txt").read_text() if (staging / "failure.txt").exists() else "Incomplete render"
                )
                raise RuntimeError(f"Example renderer failed (exit {result.returncode}): {failure}")
            rendered = json.loads(evidence.read_text())
            if rendered["frame_count"] != manifest["frame_count"]:
                raise RuntimeError("The rendered clip omits captured frames")
            for name, expected in rendered["files"].items():
                if hashlib.sha256((staging / name).read_bytes()).hexdigest() != expected:
                    raise RuntimeError(f"Rendered example hash mismatch: {name}")
            for file in staging.iterdir():
                shutil.copy2(file, args.output / file.name)
        print(f"Verified {manifest['example']} video written to {args.output}", flush=True)
        return
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True, "multi_gpu": False, "renderer": "RaytracedLighting"})
    try:
        render(capture, args.output.resolve(), manifest, app)
    except BaseException:
        import traceback

        (args.output / "failure.txt").write_text(traceback.format_exc())
        raise
    finally:
        app.close()


def render(capture, output, manifest, app):
    import imageio_ffmpeg
    import numpy as np
    import omni.replicator.core as rep
    import omni.timeline
    import omni.usd
    from motion_geometry import (
        apply_body_poses,
        pose_matrix,
        transform_points,
        verify_geometry,
    )
    from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdShade

    omni.usd.get_context().open_stage(str(capture / "scene.usda"))
    while omni.usd.get_context().get_stage_loading_status()[2] > 0:
        app.update()
    timeline = omni.timeline.get_timeline_interface()
    timeline.stop()
    start_time = timeline.get_current_time()
    stage = omni.usd.get_context().get_stage()
    with np.load(capture / "motion.npz", allow_pickle=False) as arrays:
        validate_motion(manifest, arrays)
        poses = arrays["poses"]
        frames = arrays["frames"]
        pixels = arrays["taxel2pixel"]
        times = arrays["simulation_time_s"]
    with np.load(capture / "geometry.npz", allow_pickle=False) as arrays:
        geometry = dict(arrays)
    if manifest["example"] == "flat-pad":
        pad_index = manifest["body_paths"].index("/World/envs/env_0/FingerPad")
        pad_points = np.concatenate(
            [geometry[f"points_{i}"] for i, mesh in enumerate(manifest["meshes"]) if mesh["body_index"] == pad_index]
        )
        manifest["pad_size_m"] = np.ptp(pad_points, axis=0)[:2].tolist()
    count = manifest["frame_count"]
    if poses.shape != (count, len(manifest["body_paths"]), 7) or frames.shape[0] != count:
        raise RuntimeError("Incomplete recorded frame sequence")
    if not np.isfinite(poses).all() or not np.isfinite(frames).all() or (frames < 0).any():
        raise RuntimeError("Invalid captured states or forces")
    manifest["force_scale_n"] = float(np.ceil(frames.max() * 100) / 100) or 0.01
    # Debug markers obscure the geometry; omitting them is a presentation choice.
    markers = stage.GetPrimAtPath("/Visuals")
    if markers.IsValid():
        UsdGeom.Imageable(markers).MakeInvisible()
    material = UsdShade.Material.Define(stage, "/World/DocsGround")
    shader = UsdShade.Shader.Define(stage, "/World/DocsGround/Surface")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.10, 0.12, 0.14))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.85)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    ground = stage.GetPrimAtPath("/World/defaultGroundPlane/Environment/Geometry")
    if ground.IsValid():
        UsdShade.MaterialBindingAPI.Apply(ground).Bind(material)
    light = UsdLux.DistantLight.Define(stage, "/World/DocsKey")
    light.CreateIntensityAttr(1800)
    light.CreateAngleAttr(8)
    UsdGeom.Xformable(light).AddRotateXYZOp().Set(Gf.Vec3f(25, -35, -35))
    dome = UsdLux.DomeLight.Define(stage, "/World/DocsDome")
    dome.CreateIntensityAttr(900)
    camera = UsdGeom.Camera.Define(stage, "/World/DocsCamera")
    camera.CreateFocalLengthAttr(35)
    camera.CreateHorizontalApertureAttr(36)
    camera.CreateVerticalApertureAttr(20.25)
    camera.CreateClippingRangeAttr(Gf.Vec2f(0.001, 10))
    if manifest["example"] == "flat-pad":
        eye, target = Gf.Vec3d(0.22, -0.25, 0.28), Gf.Vec3d(0, 0, 0.01)
    elif manifest["example"] == "stationary-press":
        eye, target = Gf.Vec3d(-0.23, -0.38, 0.30), Gf.Vec3d(0.035, 0.026, 0.075)
    else:
        eye, target = Gf.Vec3d(-0.25, -0.28, 0.40), Gf.Vec3d(0.02, 0.035, 0.055)
    UsdGeom.Xformable(camera).AddTransformOp().Set(
        Gf.Matrix4d(1).SetLookAt(eye, target, Gf.Vec3d(0, 0, 1)).GetInverse()
    )
    product = rep.create.render_product(str(camera.GetPath()), (960, 540))
    rgb = rep.AnnotatorRegistry.get_annotator("rgb")
    rgb.attach([product])
    detail_rgb = None
    if manifest["example"] != "flat-pad":
        detail_camera = UsdGeom.Camera.Define(stage, "/World/DocsContactCamera")
        detail_camera.CreateFocalLengthAttr(35)
        detail_camera.CreateHorizontalApertureAttr(36)
        detail_camera.CreateVerticalApertureAttr(27)
        detail_camera.CreateClippingRangeAttr(Gf.Vec2f(0.001, 10))
        detail_op = detail_camera.AddTransformOp()
        detail_product = rep.create.render_product(str(detail_camera.GetPath()), (320, 240))
        detail_rgb = rep.AnnotatorRegistry.get_annotator("rgb")
        detail_rgb.attach([detail_product])
        gear_mesh = next(
            (i, mesh)
            for i, mesh in enumerate(manifest["meshes"])
            if (
                (manifest.get("backend") == "newton" and mesh["path"] == "/World/envs/env_0/Probe/collision")
                or ("/Asset/" in mesh["path"] and mesh["path"].endswith("/visuals"))
            )
        )
    rep.orchestrator.set_capture_on_play(False)
    import carb.settings

    carb.settings.get_settings().set("/exts/isaacsim.core.throttling/enable_async", False)
    for _ in range(40):
        app.update()
    playback_slowdown = {"flat-pad": 3, "stationary-press": 2, "hand": 1}[manifest["example"]]
    fps = manifest["playback_fps"] / playback_slowdown
    writer = imageio_ffmpeg.write_frames(
        str(output / (manifest["example"] + ".mp4")),
        (960, 800),
        fps=fps,
        codec="libx264",
        pix_fmt_out="yuv420p",
        output_params=["-crf", "23", "-movflags", "+faststart"],
        macro_block_size=1,
    )
    writer.send(None)
    image_frames = []
    maximum_pose_error = 0.0
    maximum_vertex_error = 0.0
    try:
        for index in range(count):
            apply_body_poses(stage, manifest["body_paths"], poses[index], geometry["body_scales"])
            if detail_rgb is not None:
                mesh_index, mesh_entry = gear_mesh
                gear_world = transform_points(
                    np.asarray(UsdGeom.Mesh(stage.GetPrimAtPath(mesh_entry["path"])).GetPointsAttr().Get()),
                    UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(mesh_entry["path"])),
                )
                center = (gear_world.min(0) + gear_world.max(0)) / 2
                detail_target = Gf.Vec3d(*map(float, center))
                detail_offset = np.array([-0.025, -0.035, 0.045])
                if manifest["example"] == "stationary-press":
                    # View the contact face obliquely in its own frame. A fixed
                    # world offset grazes the bent pad and hides the contact.
                    rotation = np.asarray(pose_matrix(poses[index, mesh_entry["body_index"]]))[:3, :3]
                    detail_offset = detail_offset @ rotation
                detail_eye = Gf.Vec3d(*map(float, center + detail_offset))
                detail_op.Set(Gf.Matrix4d(1).SetLookAt(detail_eye, detail_target, Gf.Vec3d(0, 0, 1)).GetInverse())
            for _ in range(2):
                app.update()
            rep.orchestrator.step(rt_subframes=4, delta_time=0.0, pause_timeline=True)
            if timeline.is_playing() or timeline.get_current_time() != start_time:
                raise RuntimeError("Renderer advanced the recorded physical timeline")
            pose_error, vertex_error = verify_geometry(
                stage, manifest["body_paths"], poses[index], manifest["meshes"], geometry
            )
            maximum_pose_error = max(maximum_pose_error, pose_error)
            maximum_vertex_error = max(maximum_vertex_error, vertex_error)
            data = rgb.get_data()
            if data is None or data.size == 0 or np.std(data[..., :3]) < 2:
                raise RuntimeError(f"Empty camera frame {index}")
            detail = detail_rgb.get_data()[..., :3] if detail_rgb is not None else None
            if detail is not None and (detail.size == 0 or np.std(detail) < 2):
                raise RuntimeError(f"Empty contact camera frame {index}")
            image = composite(
                data[..., :3], frames[index], pixels, manifest, index, times[index], playback_slowdown, detail
            )
            writer.send(np.array(image))
            image_frames.append(image)
            if index % 10 == 0:
                print(f"Rendered {manifest['example']} frame {index + 1}/{count}", flush=True)
    finally:
        writer.close()
    if manifest["example"] == "stationary-press":
        held = np.flatnonzero(np.array(manifest["phases"]) == "hold")
        poster_index = int(held[len(held) // 2])
    elif manifest["example"] == "flat-pad":
        poster_index = count * 4 // 5
    else:
        poster_index = count // 5
    image_frames[poster_index].save(output / (manifest["example"] + ".webp"), quality=90, method=6)
    # Three captured states explain the transition even without playing video.
    candidates = [
        0,
        int(np.argmax(frames.sum((2, 3)).max(1))) if manifest["example"] == "flat-pad" else poster_index,
        count - 1,
    ]
    for phase, idx in zip(["before", "contact", "after"], candidates):
        image_frames[idx].save(output / f"{manifest['example']}-{phase}.webp", quality=90, method=6)
    result = {
        "example": manifest["example"],
        "capture_sha256": hashlib.sha256((capture / "capture.json").read_bytes()).hexdigest(),
        "physics_version": manifest["physics_version"],
        "renderer_version": version("isaacsim"),
        "frame_count": count,
        "fps": fps,
        "playback_slowdown": playback_slowdown,
        "duration_s": count / fps,
        "simulation_time_range_s": [float(times[0]), float(times[-1])],
        "physical_timeline_advanced_by_renderer": False,
        "force_frames_interpolated": False,
        "maximum_rendered_pose_error": maximum_pose_error,
        "maximum_rendered_vertex_error_m": maximum_vertex_error,
        "checked_mesh_count": len(manifest["meshes"]),
        "contact_closeup_camera": detail_rgb is not None,
        "geometry_script_sha256": hashlib.sha256(
            Path(__file__).with_name("motion_geometry.py").read_bytes()
        ).hexdigest(),
        "shared_force_scale_n": manifest["force_scale_n"],
        "renderer_script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "poster_frame": poster_index,
        "files": {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in output.iterdir() if p.is_file()},
    }
    # VP9 is broadly supported, including Chromium without proprietary codecs.
    subprocess.run(
        [
            imageio_ffmpeg.get_ffmpeg_exe(),
            "-y",
            "-i",
            str(output / (manifest["example"] + ".mp4")),
            "-c:v",
            "libvpx-vp9",
            "-crf",
            "34",
            "-b:v",
            "0",
            "-an",
            str(output / (manifest["example"] + ".webm")),
        ],
        check=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    captions = ["WEBVTT", ""]
    first = 0
    for i in range(1, count + 1):
        if i == count or manifest["phases"][i] != manifest["phases"][first]:

            def timestamp(t):
                millis = round(t * 1000)
                return f"{millis // 3600000:02}:{millis // 60000 % 60:02}:{millis // 1000 % 60:02}.{millis % 1000:03}"

            captions.extend([
                f"{timestamp(first / fps)} --> {timestamp(i / fps)}",
                manifest["phases"][first].capitalize(),
                "",
            ])
            first = i
    (output / (manifest["example"] + ".vtt")).write_text("\n".join(captions).rstrip() + "\n")
    result["video_size"] = [960, 800]
    if manifest["example"] == "flat-pad":
        result["heatmap_display"] = {
            "pad_size_m": manifest["pad_size_m"],
            "width_to_height": manifest["pad_size_m"][0] / manifest["pad_size_m"][1],
            "columns_axis": "pad x (length)",
            "rows_axis": "pad y (width)",
            "resampling": "nearest",
        }
    result["files"] = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in output.iterdir() if p.is_file()}
    (output / (manifest["example"] + ".json")).write_text(json.dumps(result, indent=2) + "\n")
    print(f"Completed {manifest['example']} video: {count} verified physical frames", flush=True)


def composite(rgb, frames, pixels, manifest, index, time_s, slowdown, detail=None):
    import importlib.util

    from PIL import Image, ImageDraw, ImageFont

    repo = Path(__file__).resolve().parents[2]
    spec = importlib.util.spec_from_file_location(
        "tactile_colors", repo / "source/dex01_sim_asset/dex01_sim_asset/vis_utils/viewport.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    maximum = manifest.get("force_scale_n", 0.2)
    canvas = Image.new("RGB", (960, 800), "white")
    canvas.paste(Image.fromarray(rgb), (0, 42))
    draw = ImageDraw.Draw(canvas)
    font = ImageFont.truetype("DejaVuSans.ttf", 16)
    if detail is not None:
        canvas.paste(Image.fromarray(detail).resize((280, 210), Image.Resampling.LANCZOS), (660, 78))
        draw.rectangle((659, 55, 940, 288), outline="#6e8498", width=2)
        draw.rectangle((660, 56, 939, 77), fill="white")
        draw.text((669, 58), "Contact close-up", font=font, fill="#17324d")
    title = {
        "flat-pad": "Flat pad · falling gear",
        "hand": "Tesollo hand · sequential contacts",
        "stationary-press": "Fixed wrist · articulated press",
    }[manifest["example"]]
    draw.text((20, 11), title, font=font, fill="#17324d")
    draw.text(
        (470, 11),
        (
            f"t = {time_s:.2f} s · {manifest['phases'][index]} · {slowdown}× slower"
            if slowdown > 1
            else f"t = {time_s:.2f} s · {manifest['phases'][index]}"
        ),
        font=font,
        fill="#17324d",
    )
    size = 138
    for i, frame in enumerate(frames):
        x = 35 + i * 184 if len(frames) == 5 else 110
        if len(frames) == 1:
            x = 180
        image = Image.fromarray(module.force_frame_rgba(frame, pixels, maximum)).convert("RGB")
        display_size = (size, size)
        if manifest["example"] == "flat-pad":
            length, width = manifest["pad_size_m"]
            display_size = (round(size * length / width), size)
            x = 90
        canvas.paste(image.resize(display_size, Image.Resampling.NEAREST), (x, 607))
        draw.text((x, 585), f"{manifest['labels'][i]} · {frame.sum():.3f} N", font=font, fill="#17324d")
    if len(frames) == 1:
        length, width = manifest["pad_size_m"]
        draw.text((400, 607), f"Pad: {length * 1000:g} × {width * 1000:g} mm · top view", font=font, fill="#17324d")
        draw.text((400, 633), "32 × 32 grid · 1,024 samples", font=font, fill="#17324d")
        draw.text((400, 659), "0.5 kg gear · weight 4.905 N", font=font, fill="#17324d")
        draw.text((400, 685), f"Force per taxel: 0–{maximum:.3f} N", font=font, fill="#17324d")
    else:
        draw.text(
            (35, 770),
            f"Shared force per taxel: 0–{maximum:.3f} N · grey: no taxel",
            font=ImageFont.truetype("DejaVuSans.ttf", 14),
            fill="#17324d",
        )
    return canvas


if __name__ == "__main__":
    main()
