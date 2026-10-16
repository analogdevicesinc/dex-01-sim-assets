# Install Newton

The Newton environment is independent of PhysX. Use Linux x86_64 with a working
NVIDIA CUDA GPU and driver, Git LFS and [uv](https://docs.astral.sh/uv/).
Kit viewing and video additionally need the Isaac Sim 6.1 EA prerequisites;
PhysX's Isaac Sim 5.1 driver recommendation is not a Newton compatibility claim.

## Create the pinned workspace

From a fresh clone:

```bash
git clone https://github.com/analogdevicesinc/dex-01-sim-assets.git
cd dex-01-sim-assets/newton
git lfs install
git lfs pull
./setup_env.sh
source .isaaclab/.venv/bin/activate
```

The default Isaac Lab checkout is `newton/.isaaclab`. Setup requires
`ae37b028ea415c91ea2bc32609efcd759ed2b974` (`v3.0.0-EA`), refuses a modified or
unexpected checkout, rejects a pre-existing environment with the wrong Python
or a PhysX backend, and syncs its frozen upstream lockfile. It then installs
`dex01-sim-common` and `dex01-sim-newton` from this repository without resolving
or replacing the simulation pins. Use `--isaaclab /absolute/path` for another
isolated checkout and activate that checkout's `.venv`.

Use `./setup_env.sh --kit` for a local window or WebRTC and `--video` for live
recording. The video option includes Kit. Rerun setup with the chosen extras
before launching those features.

## Check the installation

From `newton/` with the environment activated:

```bash
python scripts/newton_preflight.py --isaaclab .isaaclab --report /tmp/newton-preflight.json
python scripts/dex01_newton.py --mode hand --indenter flat --max-steps 240
```

Preflight checks Python and package pins, checkout identity, GPU/driver
availability and optional video tools. `--kit` requires Isaac Sim; `--video`
also requires FFmpeg. Preflight success is a prerequisite, not a completed
physics or camera test. Continue to [your first run](quickstart.md) and the
[required validation](validation.md).

## View locally or through WebRTC

After installing the Kit extra, run:

```bash
PYTHONPATH="$PWD/.isaaclab/source/isaaclab" python scripts/dex01_newton.py --mode hand --viz kit
PYTHONPATH="$PWD/.isaaclab/source/isaaclab" python scripts/dex01_newton.py --mode hand --viz kit --livestream 2
```

Replace `.isaaclab` in the path for a custom checkout. Accept NVIDIA's Omniverse
license when prompted. Connect a compatible Isaac Sim WebRTC client only after
`NEWTON_SCENE_READY`. TCP 49100 and UDP 47998 must be reachable; these private
network ports should be used by one simulator at a time. Use `--no-tactile-ui`
to hide the five-finger overlay. Physics without Kit uses `--viz none`.
