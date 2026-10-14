# Installation

## Check the host

The supported baseline is Linux x86_64, Python 3.11, GLIBC 2.35 or newer,
Isaac Sim 5.1.0, and the Isaac Lab revision pinned by `setup_env.sh`.
Use an NVIDIA RTX-capable GPU with enough VRAM for Isaac Sim, at least
32 GB host RAM, and at least 50 GB free SSD space for the simulator.

For desktop, streaming, or camera rendering, use NVIDIA's **validated Linux
driver 580.65.06 (R580)** for Isaac Sim 5.1.0. Check the installed version:

```bash
nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv
```

See [NVIDIA's version-specific requirements](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/requirements.html)
before configuring the host. A newer driver is not automatically compatible
with this release. In particular, NVIDIA documents an RTX SceneDB startup
crash with newer driver branches; [troubleshooting](../troubleshooting.md)
links the upstream diagnosis. A GPU visible to `nvidia-smi` alone does not
verify camera or viewport rendering.

Install Git, Git LFS, GCC, G++, Make, and
[uv](https://docs.astral.sh/uv/getting-started/installation/).
On Ubuntu, `build-essential` provides GCC, G++, and Make. Install `ffmpeg`
if you intend to record videos.

## Install the simulation environment

Run from the repository root:

```bash
git clone https://github.com/analogdevicesinc/dex-01-sim-assets.git
cd dex-01-sim-assets
git lfs install
git lfs pull
./setup_env.sh
source ../env_isaaclab/bin/activate
```

The setup script installs Python 3.11, Isaac Sim 5.1.0, PyTorch, Isaac Lab's
extensions, and this package. It installs CMake 3.x for native dependencies
such as `egl-probe`. `uv` fetches Python, so your host Python need not be 3.11.

A fresh Isaac Lab checkout is pinned to
`b0542fe2d45bf91c4e1d9ef6952b9c709c80b4e8`. Existing checkouts and environments
are reused. Read the reported versions before reusing them; the script does
not replace a working installed simulator just because its version differs.
The first simulator launch can request NVIDIA's EULA, download extensions,
and compile shaders.

## Choose installation locations

Use absolute paths for custom locations:

```bash
./setup_env.sh --venv /path/to/dex01-env --isaaclab /path/to/IsaacLab
source /path/to/dex01-env/bin/activate
```

With an existing compatible Isaac Lab installation, fetch the LFS assets and
install this package into its Python environment:

```bash
/path/to/IsaacLab/isaaclab.sh -p -m pip install -e source/dex01_sim_asset
```

All example commands run from the repository root in that environment.
Simulator modules must be imported after `AppLauncher` starts Isaac Sim.

## Confirm the installation

```bash
python - <<'PY'
from importlib.metadata import version
print("Isaac Sim:", version("isaacsim"))
print("PyTorch:", version("torch"))
PY
git -C ../IsaacLab rev-parse HEAD
MPLBACKEND=Agg python scripts/dex01_on_tesollo_dg5f.py \
  --headless \
  --flat-indenter \
  --max-steps 240
```

For a custom checkout, replace `../IsaacLab` with the path passed to setup.
The headless smoke check prints per-finger forces, runs 240 steps, and exits.
Finger 2 should settle near 0.981 N while the other fingers remain unloaded.
This local cube fixture does not download the gear asset. Continue with
[your first visual simulation](quickstart.md) and then
[complete physical-force validation](../validation.md).

## View a remote simulation

After installing and activating the environment on the simulation host, run
from the repository root:

```bash
MPLBACKEND=Agg python scripts/dex01_on_tesollo_dg5f.py --livestream=2
```

Use the RTX-capable GPU and validated R580 driver described above.
On your viewing computer, install the
[Isaac Sim WebRTC Streaming Client](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/manual_livestream_clients.html).
Wait for `Setup complete...`, enter the simulation host's reachable IP address,
and connect. TCP 49100 and UDP 47998 must be reachable from that computer.
Run one streaming simulator at a time on these ports. `--livestream=2` is for
a private network; NVIDIA's guide covers public endpoints.

The streamed viewport includes the hand, tactile frames, and normal loads in
newtons. Only environment 0 is displayed in multi-environment runs. The same
overlay is available in `simple_example.py` and `verify_dex01_sim.py`.
Use `--no-tactile-ui` to hide it or `--debug-vis` for 3D sampling markers.
Stop with Ctrl+C. Pure headless runs without streaming skip the viewport.
Continue with [your first simulation](quickstart.md) to interpret the result.
