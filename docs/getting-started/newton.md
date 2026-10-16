# Newton / Isaac Lab 3.0 Early Access

Newton runs in a separate Python 3.12 environment; the PhysX Isaac Sim 5.1 commands
continue to use the original environment. The Newton setup pins Isaac Lab
`v3.0.0-EA` (`ae37b028ea415c91ea2bc32609efcd759ed2b974`) and its upstream lockfile:
Newton 1.5.2, Warp 1.16.0, MuJoCo/MuJoCo-Warp 3.11.0 and Torch 2.11.0.
Kit viewing/streaming requires Isaac Sim 6.1.0. This migration targets Linux x86-64.
Live GPU validation must be repeated for this checkout before release.

```bash
./setup_newton_env.sh --video
```

The default checkout is `.newton/IsaacLab`; use `--isaaclab PATH` to select an
isolated checkout elsewhere. Setup refuses an unexpected or modified revision.
Run with `uv` from that checkout so its dependency overrides remain effective.
Replace the paths below with your checkout paths:

```bash
cd /path/to/IsaacLab-Newton
uv run --frozen --extra isaacsim --extra video python /path/to/dex-01-sim-assets/scripts/newton_preflight.py --isaaclab . --video --report /tmp/newton-preflight.json
PYTHONPATH="$PWD/source/isaaclab" uv run --frozen --extra isaacsim --extra video python /path/to/dex-01-sim-assets/scripts/dex01_newton.py --mode press --viz kit --livestream 2 --record /tmp/newton-press.mp4 --report /tmp/newton-press.json
```

Accept NVIDIA's Omniverse license when prompted. After accepting it, you can set
`OMNI_KIT_ACCEPT_EULA=YES` for unattended launches. Connect the WebRTC client only
after `NEWTON_SCENE_READY`. Use a client resolution large enough for all five
heatmaps. Local viewing uses `--viz kit` without `--livestream`; unattended
physics uses `--viz none`. Recording uses Newton's camera renderer and combines
RGB and firmware frames from the same simulation tick. RTX camera recording is
not validated for this EA configuration.

`--mode hand --indenter flat` runs the 0.1 kg load on fingers 2–5. `--mode press`
uses the fixed wrist, a stationary gear and joint actuator targets. `--indenter
sphere` uses a closed triangle-mesh sphere; `--indenter gear` fetches the same
NVIDIA asset as the legacy demo into a temporary cache. Generated hand and
collider overlays also stay in the temporary cache. Released visual geometry,
taxel poses and mounting metadata are preserved.

```bash
uv run --frozen python /path/to/dex-01-sim-assets/scripts/dex01_newton.py --mode hand --indenter flat --max-steps 1440 --report /tmp/newton-load.json
uv run --frozen python /path/to/dex-01-sim-assets/scripts/dex01_newton.py --check-report /tmp/newton-load.json
uv run --frozen python /path/to/dex-01-sim-assets/scripts/verify_dex01_newton.py --sweep-all --report /tmp/newton-sweep.json
```

The Newton backend regression tests require the pinned environment and a CUDA GPU.
They run the real Warp distance kernel and actual MuJoCo-Warp contact pipeline;
missing dependencies or GPU access fail when the required flag is set. Install
`pytest` into that environment without changing the simulation pins, then run:

```bash
DEX01_REQUIRE_NEWTON_TESTS=1 /path/to/IsaacLab-Newton/.venv/bin/python -m pytest /path/to/dex-01-sim-assets/tests/test_newton_backend.py -q
```

CPU-only CI runs the Newton mathematics, report and CLI lifecycle tests and skips
the optional GPU module. GPU validation must be run separately; a skipped module
is not evidence of Newton correctness. The contact verifier checks five locations
at separation, touching and two indentation depths by default; `--sweep-all`
checks those four conditions at every taxel. Both modes also check analytic sphere
depths, normal-load conservation and two-environment subset isolation.

The load checker requires every physics tick and verifies mapping, filtered
normal-force agreement, settling, unloaded fingers and momentum balance. Press
reports require three complete cycles and verify the fixed wrist/object, real
joint motion, fingertip travel and release. The full sweep tests all 738 contact
locations against the actual asset. A gear run is a demonstration, not a known
weight calibration fixture.

The Newton sensor is available from `dex01_sim_asset.newton`. Bind it after
scene reset to sensing-link and counter-object state providers plus an Isaac Lab
contact sensor filtered to that counter object. Providers return XYZW link
poses, world COM positions and COM linear/angular velocities. Public tactile
outputs retain Torch tensors, metres/newtons units and WXYZ taxel orientations;
`offset_rot_wxyz` also retains the reference convention. Contact data must update
every physics tick. Only one counter collider per sensor, positive uniform scale
and a fixed collider-to-link transform are supported.

MuJoCo-Warp uses `implicitfast` and Newton's native SDF collision pipeline.
The hand's stiff drives require smaller solver substeps; these demo settings are
not hardware parameters. Force distributions remain a normal-load approximation
and do not model measured material response, shear or hardware calibration.
Use `--benchmark PATH --num-envs N` for warm physics/sensing timing; pair with
`--no-sensing` for physics-only timing. GPU integration results must accompany
any new solver/version or geometry change.


## Capture and render Newton evidence

Capture real Newton link poses and firmware arrays, validating the complete run
before exporting. This exports environment zero; use a new output directory.

```bash
uv run --frozen python /path/to/dex-01-sim-assets/scripts/dex01_newton.py --mode hand --indenter flat --max-steps 1440 --report /tmp/newton-hand.json --capture /tmp/newton-hand-capture
uv run --frozen python /path/to/dex-01-sim-assets/scripts/dex01_newton.py --mode press --max-steps 1800 --report /tmp/newton-press.json --capture /tmp/newton-press-capture
```

The press capture contains the third complete cycle. Exports preserve body
scales, mesh offsets and topology. Geometry is checked after export and at every
replayed pose, using the same audit as the PhysX documentation renderer.

Use the separate RTX rendering environment described in
[the maintainer guide](https://github.com/analogdevicesinc/dex-01-sim-assets/blob/main/DOCUMENTATION.md). From the repository root:

```bash
python scripts/docs/render_examples.py --capture /tmp/newton-hand-capture --output /tmp/newton-hand-render
```

The renderer verifies file hashes, tick spacing, XYZW rotations, matching taxel
forces and firmware frames, and fixed wrist/object poses for pressing. It keeps
physics stopped and uses one force scale for the full clip. Existing PhysX
images, videos and validation results do not validate Newton. The known media
regeneration warning remains in the [media provenance](https://github.com/analogdevicesinc/dex-01-sim-assets/blob/main/docs/media/README.md).

Live `--record` also saves timestamped force arrays and a JSON manifest beside
the video, with source, array and video hashes and one shared force scale.
GPU physics and camera checks remain mandatory before publishing Newton media.

## Continue by topic

- [Installation and streaming](installation.md#install-newton)
- [First simulation](quickstart.md#first-newton-simulation)
- [Examples by backend](../examples/index.md#choose-a-newton-example)
- [Attachment and post-reset binding](../guides/attachment.md#attach-with-newton)
- [Reading data](../guides/read-data.md#newton-update-order) and [patterns](../guides/patterns.md#patterns-with-newton)
- [Sensor API](../reference/api.md#newton-tactilesensorcfg)
- [Numerical settings](../reference/numerical-settings.md#newton-demo-settings) and [compatibility](../reference/compatibility.md#newton-pinned-baseline)
- [Contact model](../concepts/model.md#newton-contact-and-distance-queries) and [limitations](../concepts/limitations.md#newton-boundaries)
- [Validation](../validation.md#newton-validation) and [troubleshooting](../troubleshooting.md#newton-startup-and-data-errors)
