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

"""Render a captured physical scene without advancing its simulation state.

Use an Isaac Sim rendering environment. Input is produced by export_scene.py;
only camera and lighting are authored here, and the timeline stays stopped.
"""

import argparse
import hashlib
import json
import shutil
import subprocess
import sys
import tempfile
import traceback
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--state", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()
    state = args.state.resolve()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    manifest = json.loads((state / "manifest.json").read_text())
    for name in ("scene", "tactile"):
        filename = state / ("scene.usda" if name == "scene" else "tactile.npz")
        if hashlib.sha256(filename.read_bytes()).hexdigest() != manifest[f"{name}_sha256"]:
            raise RuntimeError(f"Captured {name} hash differs from its manifest")
    if not args.worker:
        render_checked(state, output, manifest)
        return
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True, "multi_gpu": False, "renderer": "RaytracedLighting"})
    try:
        render(state, output, manifest, app)
    except BaseException:
        failure = traceback.format_exc()
        (output / "failure.txt").write_text(failure)
        print(failure, file=sys.stderr, flush=True)
        raise
    finally:
        app.close()


def render_checked(state, output, manifest):
    """Reject partial output or failures hidden by the Kit shutdown path."""
    from PIL import Image

    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="dex01-render-", dir=output.parent) as directory:
        staging = Path(directory)
        process = subprocess.run(
            [
                sys.executable,
                "-u",
                str(Path(__file__).resolve()),
                "--worker",
                "--state",
                str(state),
                "--output",
                str(staging),
            ],
            check=False,
        )
        failure = staging / "failure.txt"
        result_path = staging / "render-manifest.json"
        if process.returncode or failure.exists() or not result_path.exists():
            detail = failure.read_text() if failure.exists() else "The camera worker did not produce verified output."
            raise RuntimeError(f"Camera worker failed (exit {process.returncode}): {detail}")
        result = json.loads(result_path.read_text())
        if (
            result["timeline_advanced"]
            or not result["fingertip_transforms_unchanged"]
            or not result["rigid_body_transforms_unchanged"]
        ):
            raise RuntimeError("The camera worker changed the captured simulation state")
        if (
            result["rendered_sensor_visuals"] != 5
            or result["tactile_sha256"] != manifest["tactile_sha256"]
            or result["scene_sha256"] != manifest["scene_sha256"]
        ):
            raise RuntimeError("The camera worker did not render the verified five-fingertip capture")
        for name in (
            "hand-camera.png",
            "loaded-contact.png",
            "loaded-contact.webp",
            "dex01-demo.png",
            "dex01-demo.webp",
        ):
            path = staging / name
            if hashlib.sha256(path.read_bytes()).hexdigest() != result["rendered_files"][name]:
                raise RuntimeError(f"Camera output hash differs from its manifest: {name}")
            with Image.open(path) as image:
                image.verify()
        with Image.open(staging / "hand-camera.png") as image:
            if list(image.size) != result["camera_resolution"]:
                raise RuntimeError("Camera output dimensions differ from the recorded render settings")
        output.mkdir(parents=True, exist_ok=True)
        for name in (
            "hand-camera.png",
            "loaded-contact.png",
            "loaded-contact.webp",
            "dex01-demo.png",
            "dex01-demo.webp",
            "render-manifest.json",
        ):
            shutil.copy2(staging / name, output / name)
    print(f"Verified camera images written to {output}", flush=True)


def render(state, output, manifest, app):
    import numpy as np
    import omni.replicator.core as rep
    import omni.timeline
    import omni.usd
    from PIL import Image
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade

    if not omni.usd.get_context().open_stage(str(state / "scene.usda")):
        raise RuntimeError("Could not open the captured stage")
    while omni.usd.get_context().get_stage_loading_status()[2] > 0:
        app.update()
    timeline = omni.timeline.get_timeline_interface()
    timeline.stop()
    if timeline.is_playing():
        raise RuntimeError("Rendering must not advance the captured physics state")
    stage = omni.usd.get_context().get_stage()
    prims = [
        p
        for p in Usd.PrimRange(stage.GetPrimAtPath("/World/envs/env_0"), Usd.TraverseInstanceProxies())
        if p.IsA(UsdGeom.Mesh)
        and "_sensor/visual" in str(p.GetPath())
        and UsdGeom.Imageable(p).ComputeVisibility() != "invisible"
    ]
    if len(prims) != 5 or manifest["sensor_visual_count"] != 5:
        raise RuntimeError("The camera scene must contain all five visible sensor meshes")
    captured_transforms = {
        str(prim.GetPath()): np.array(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()))
        for prim in prims
    }
    captured_body_transforms = {
        str(prim.GetPath()): np.array(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()))
        for prim in Usd.PrimRange(stage.GetPrimAtPath("/World/envs/env_0"), Usd.TraverseInstanceProxies())
        if prim.HasAPI(UsdPhysics.RigidBodyAPI)
    }
    captured_time = timeline.get_current_time()
    # A neutral ground material removes the simulator grid without changing
    # geometry, contact material, or the measured state.
    material = UsdShade.Material.Define(stage, "/World/DocsGroundMaterial")
    shader = UsdShade.Shader.Define(stage, "/World/DocsGroundMaterial/Surface")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.10, 0.12, 0.14))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.85)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    ground = stage.GetPrimAtPath("/World/defaultGroundPlane/Environment/Geometry")
    UsdShade.MaterialBindingAPI.Apply(ground).Bind(material)
    # Preserve measured geometry. Camera, lighting and ground shading are
    # presentation settings; the hand's authored materials stay intact.
    dome = UsdLux.DomeLight.Define(stage, "/World/DocsDome")
    dome.CreateIntensityAttr(800)
    dome.CreateColorAttr(Gf.Vec3f(0.88, 0.92, 1.0))
    light = UsdLux.DistantLight.Define(stage, "/World/DocsKey")
    light.CreateIntensityAttr(1800)
    light.CreateAngleAttr(8)
    UsdGeom.Xformable(light).AddRotateXYZOp().Set(Gf.Vec3f(25, -35, -35))
    camera = UsdGeom.Camera.Define(stage, "/World/DocsCamera")
    camera.CreateFocalLengthAttr(35)
    camera.CreateHorizontalApertureAttr(36)
    camera.CreateVerticalApertureAttr(36 * 720 / 1280)
    camera.CreateClippingRangeAttr(Gf.Vec2f(0.001, 10))
    xform = UsdGeom.Xformable(camera)
    eye = Gf.Vec3d(-0.25, -0.28, 0.40)
    target = Gf.Vec3d(0.02, 0.035, 0.055)
    view = Gf.Matrix4d(1).SetLookAt(eye, target, Gf.Vec3d(0, 0, 1))
    xform.AddTransformOp().Set(view.GetInverse())
    product = rep.create.render_product(str(camera.GetPath()), (1280, 720))
    rgb = rep.AnnotatorRegistry.get_annotator("rgb")
    rgb.attach([product])
    rep.orchestrator.set_capture_on_play(False)
    for _ in range(80):
        app.update()
    rep.orchestrator.step(rt_subframes=16, delta_time=0.0, pause_timeline=True)
    if timeline.is_playing() or timeline.get_current_time() != captured_time:
        raise RuntimeError("The captured timeline advanced while rendering")
    for prim in prims:
        transform = np.array(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()))
        if not np.array_equal(transform, captured_transforms[str(prim.GetPath())]):
            raise RuntimeError(f"Captured fingertip moved during rendering: {prim.GetPath()}")
    data = rgb.get_data()
    if data is None or data.size == 0 or np.std(data[..., :3]) < 2:
        raise RuntimeError("Camera output is empty or uniform")
    Image.fromarray(data[..., :3]).save(output / "hand-camera.png")
    make_composite(data[..., :3], state, output)
    with np.load(state / "tactile.npz", allow_pickle=False) as arrays:
        contact_points = arrays["points2"]
        frame = arrays["frame2"]
        pixels = arrays["taxel2pixel"]
        weights = frame[pixels[:, 0], pixels[:, 1]]
    contact_center = (contact_points * weights[:, None]).sum(0) / weights.sum()
    close_eye = Gf.Vec3d(*map(float, contact_center + np.array([-0.050, 0.045, 0.080])))
    close_target = Gf.Vec3d(*map(float, contact_center))
    camera.GetOrderedXformOps()[0].Set(
        Gf.Matrix4d(1).SetLookAt(close_eye, close_target, Gf.Vec3d(0, 0, 1)).GetInverse()
    )
    for _ in range(12):
        app.update()
    rep.orchestrator.step(rt_subframes=16, delta_time=0.0, pause_timeline=True)
    closeup = rgb.get_data()
    if closeup is None or closeup.size == 0 or np.std(closeup[..., :3]) < 2:
        raise RuntimeError("Close-up camera output is empty or uniform")
    if timeline.is_playing() or timeline.get_current_time() != captured_time:
        raise RuntimeError("The captured state advanced during the close-up")
    for prim in prims:
        transform = np.array(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()))
        if not np.array_equal(transform, captured_transforms[str(prim.GetPath())]):
            raise RuntimeError(f"Captured fingertip moved during close-up: {prim.GetPath()}")
    for path, expected in captured_body_transforms.items():
        body = stage.GetPrimAtPath(path)
        transform = np.array(UsdGeom.Xformable(body).ComputeLocalToWorldTransform(Usd.TimeCode.Default()))
        if not np.array_equal(transform, expected):
            raise RuntimeError(f"Captured body moved during rendering: {path}")
    Image.fromarray(closeup[..., :3]).save(output / "loaded-contact.png")
    Image.fromarray(closeup[..., :3]).save(output / "loaded-contact.webp", quality=92, method=6)
    from importlib.metadata import version

    result = {
        **manifest,
        "renderer_version": version("isaacsim"),
        "camera_resolution": [1280, 720],
        "timeline_advanced": False,
        "rendered_sensor_visuals": len(prims),
        "fingertip_transforms_unchanged": True,
        "rigid_body_transforms_unchanged": True,
        "checked_rigid_bodies": len(captured_body_transforms),
        "overview_camera": {"eye_m": list(eye), "target_m": list(target), "focal_length_mm": 35},
        "closeup_camera": {"eye_m": list(close_eye), "target_m": list(close_target)},
        "ground_presentation": "Neutral diffuse material; captured geometry unchanged",
        "rendered_files": {
            name: hashlib.sha256((output / name).read_bytes()).hexdigest()
            for name in (
                "hand-camera.png",
                "loaded-contact.png",
                "loaded-contact.webp",
                "dex01-demo.png",
                "dex01-demo.webp",
            )
        },
    }
    (output / "render-manifest.json").write_text(json.dumps(result, indent=2) + "\n")
    print(f"Rendered all five fingertips from frozen {manifest['physics_version']} state", flush=True)


def make_composite(rgb, state, output):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    from PIL import Image

    with np.load(state / "tactile.npz") as arrays:
        pixels = arrays["taxel2pixel"]
        frames = [arrays[f"frame{i}"].copy() for i in range(1, 6)]
    occupied = np.zeros((32, 32), dtype=bool)
    occupied[pixels[:, 0], pixels[:, 1]] = True
    maximum = max(float(frame.max()) for frame in frames)
    fig = plt.figure(figsize=(12.8, 2.8), facecolor="#ffffff")
    for i, frame in enumerate(frames):
        ax = fig.add_axes((0.055 + i * 0.185, 0.19, 0.15, 0.65))
        cmap = plt.get_cmap("turbo").copy()
        cmap.set_bad("#d2d9df")
        ax.imshow(np.ma.array(frame, mask=~occupied), vmin=0, vmax=maximum or 0.01, cmap=cmap, interpolation="nearest")
        ax.set_title(f"Finger {i + 1}  |  {frame.sum():.3f} N", fontsize=11, color="#17324d")
        ax.set_axis_off()
    fig.text(
        0.5,
        0.035,
        f"Force per taxel: 0–{maximum:.4f} N · grey: no taxel · 0.1 kg flat load",
        ha="center",
        fontsize=11,
        color="#17324d",
    )
    panel = output / "tactile-panel.png"
    fig.savefig(panel, dpi=100, facecolor=fig.get_facecolor())
    plt.close(fig)
    camera = Image.fromarray(rgb).convert("RGB")
    with Image.open(panel) as tactile:
        composite = Image.new("RGB", (camera.width, camera.height + tactile.height), "white")
        composite.paste(camera, (0, 0))
        composite.paste(tactile, (0, camera.height))
    composite.save(output / "dex01-demo.png")
    composite.save(output / "dex01-demo.webp", quality=92, method=6)
    panel.unlink()


if __name__ == "__main__":
    main()
