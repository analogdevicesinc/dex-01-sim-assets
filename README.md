# DEX-01 Tactile Sensor

An Isaac Lab tactile sensor model with a physically scaled DEX-01 V10 fingertip
on the Tesollo DG-5F hand. The fingertip is one exterior-only mesh, approximately
32.70 x 22.41 x 19.00 mm, with a 22.40 x 19.00 mm rear profile. It has 738 contact
samples mapped into a 32x32 firmware frame.

The model queries the counter object's PhysX signed distance field (SDF) and
distributes the measured normal contact load across penetrating samples by
depth, following [TacSL](https://arxiv.org/abs/2408.06506). These are simulation
contact forces, not calibrated hardware readings.

## Install

Verified on Linux x86_64 with Isaac Sim 5.1.0, Python 3.11, GLIBC 2.35+ and a
supported NVIDIA GPU/driver. Install Git, Git LFS, GCC, G++, Make and
[uv](https://docs.astral.sh/uv/getting-started/installation/) first. Allow tens
of gigabytes of disk space for Isaac Sim.

```bash
git lfs install
git lfs pull
./setup_env.sh
source ../env_isaaclab/bin/activate
```

Setup installs Python, Isaac Sim, PyTorch, the full Isaac Lab extensions and
this package. It installs CMake 3.x for native dependencies such as `egl-probe`.
Fresh Isaac Lab checkouts use the tested revision recorded in the script;
existing checkouts are preserved. Use `--venv PATH` and `--isaaclab PATH` for
other locations. The first run may request NVIDIA's EULA, download extensions
and compile shaders. With an existing Isaac Lab installation, install into
its Python environment using:

```bash
/path/to/IsaacLab/isaaclab.sh -p -m pip install -e source/dex01_sim_asset
```

## Run and view

For a local desktop:

```bash
python scripts/dex01_on_tesollo_dg5f.py
```

For a remote headless host:

```bash
MPLBACKEND=Agg python scripts/dex01_on_tesollo_dg5f.py --livestream=2
```

Open the [Isaac Sim WebRTC Streaming Client](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/manual_livestream_clients.html)
on the viewing computer. Wait for `Setup complete...`, enter the host's reachable
IP address and connect. TCP 49100 and UDP 47998 must be reachable. `--livestream=2`
is for a private network; NVIDIA's guide covers public endpoints. Run one
streaming simulator at a time on these ports.

A gear contacts fingers 2–5 in turn. The Isaac Sim viewport includes live
firmware heatmaps and each finger's measured normal load in newtons, so they
appear in the WebRTC GUI. Blue is zero load; cyan, yellow and red show increasing
force per taxel, with a shared adaptive scale printed below the images. Grey
cells have no sensor taxel. Firmware row/column positions are preserved, with
no smoothing: the contact footprint (including gear teeth when resolved by the
sampling) is visible rather than just an aggregate force. These frames unwrap
the curved fingertip; they are not an optical picture of the object.

The same overlay is enabled in `simple_example.py` and `verify_dex01_sim.py`.
Only environment 0 is displayed when running multiple environments. Add
`--no-tactile-ui` to hide it, `--local-plot` to the hand demo for the optional
separate desktop plot, or `--debug-vis` for 3D sampling markers. Stop with Ctrl+C.
Pure headless runs without streaming skip the UI. Offscreen camera recordings
exclude Kit UI; the existing recorder adds its own heatmaps to the video.

### Press a stationary gear with a fixed wrist

```bash
MPLBACKEND=Agg python scripts/dex01_stationary_press.py --livestream=2
```

The wrist remains fixed. Finger 2's MCP, PIP and DIP flexion joints are driven
with actuator position targets to bend the fingertip toward the stationary gear,
hold contact, then extend back to rest. The original fixed-base hand is used;
there is no sliding fixture. Setup samples an articulated endpoint to place the
gear against the pad and orient its face to the endpoint surface normal.
Runtime motion uses joint targets, not joint-state or root-pose teleportation.

The GUI shows the fixed wrist, visible finger articulation, measured loads in
newtons and firmware contact footprints. The force is read from PhysX and
redistributed across penetrating taxels. Joint gains and endpoint fractions are
simulation demo settings, not DG-5F hardware specifications.

Record three cycles and independently verify wrist/object invariance, actual
joint movement, fingertip travel, contact, mapping and release:

```bash
MPLBACKEND=Agg python scripts/dex01_stationary_press.py --livestream=2 --max-steps 900 --verification-report /tmp/dex01-articulated.json
python scripts/dex01_stationary_press.py --check-report /tmp/dex01-articulated.json
```

`--press-time-scale` controls the physical joint trajectory timing. The default
cycle is 2.5 simulation seconds; observed duration depends on host performance. The target
force is a contact-seeking threshold, not guaranteed force tracking. A small
gear's individual teeth may not resolve in the sparse fingertip contact patch.
The existing flat-pad example is suited to viewing broader object footprints.

To record one cycle with both the rendered hand and taxel heatmaps, install
`ffmpeg` and run:

```bash
MPLBACKEND=Agg python scripts/dex01_on_tesollo_dg5f.py --headless --enable_cameras --record --video-dir ./videos
```

## Verify a known load

In the activated setup environment:

```bash
python -m pip install scipy rtree
MPLBACKEND=Agg python scripts/dex01_on_tesollo_dg5f.py --headless --flat-indenter --max-steps 1200 --verify-thumb --verification-report /tmp/dex01-hand.json
python scripts/verify_dex01_sim.py --check-hand-report /tmp/dex01-hand.json
```

The 0.1 kg flat load should settle near **0.981 N** on each pressed finger,
with zero on unloaded fingers. The checker verifies forces, counter-object
filtering and momentum balance, then reports the separate thumb probe result.
The gear can rock and is not a calibration load.

For the complete 738-contact sweep, or for multiple-angle inspection images:

```bash
python scripts/verify_dex01_sim.py --headless --sweep-all --transformed-probe --num-envs 2 --report /tmp/dex01-contact.json
MPLBACKEND=Agg python scripts/dex01_on_tesollo_dg5f.py --headless --enable_cameras --max-steps 1 --inspection-dir /tmp/dex01-views
```

## Read data and attach the sensor

Simulator modules must be imported after `AppLauncher` starts Isaac Sim.
Inside a running scene:

```python
import torch

sensor = scene["finger2_sensor"]
forces_n = sensor.data.normal_forces[0]  # 738 values
frame_n = sensor.get_tactile_image()[0]  # 32 x 32 firmware frame
rows, columns = sensor.taxel2pixel.T
assert torch.equal(frame_n[rows, columns], forces_n)
print("Load (N):", forces_n.sum().item())
```

Unoccupied pixels are zero. World positions and normals are available in
`taxel_points_w` and `taxel_normals_w`. `depths` is negative in contact and zero
otherwise; `depth_dots` is in metres per second.

Use `GridPatternCfg` for a flat pad, `CustomPatternCfg` for arbitrary NPZ poses,
or `Dex01PatternCfg` for the V10 exterior. Custom NPZ files contain `poses` of
shape `(N,3,4)`, with the third axis as the surface normal, and integer
`taxel2pixel` pairs `(row,column)`. `dex01_pattern.npz` preserves the corrected
original layout; `dex01_contact_v10.npz` contains the exterior sampling poses.
Their taxel order and pixel mapping are identical.

The original layout aligns to the overmold interior by one translation of
approximately `(-2.775670, -0.0000166, -1.429237)` mm in the sensor frame, with
no scaling, additional rotation or individual taxel movement. Reference-CAD
comparison gave 1.37 µm RMS and 4.29 µm maximum residual. Contact poses were then
projected along the original normals through approximately 1.43 mm of overmold.
`assets/sensors/dex01/reference.json` records this shift and the mounting offset.
Offset quaternions use `(w,x,y,z)`; translations and OBJ coordinates use metres.

Both sensor body and counter object need contact sensors enabled. The sensor
body must use compliant contact; the counter object requires an SDF collision
mesh. Use the matching mounting offset and a small timestep. The
[verification example](scripts/verify_dex01_sim.py) contains a complete scene
with a local sphere; the [hand demo](scripts/dex01_on_tesollo_dg5f.py) shows robot
attachment. The released OBJ is the editable exterior source; regenerate its
USD with `python scripts/build_dex01_asset.py` after updating the source hash
in `reference.json`.

Supported counter objects are rigid bodies or articulation links, one per
sensor, with positive uniform scale and a fixed mesh-to-body transform shared
across environments. The hand demo uses 20 g sensor mass, 10,000 N/m contact
stiffness and 50 N s/m damping at a 1/240 s timestep. These are simulation
parameters. Only normal forces are modeled; material response, shear,
hysteresis, manufacturing tolerances and physical calibration are not established.
Fit was checked in the supplied hand pose, not every articulation angle.

## Numerical settings

| Values | Meaning |
|---|---|
| `mount_translation_m`, `mount_rotation_wxyz` | Fixed transform from the original taxel coordinates into the mounted fingertip frame; metres and `(w,x,y,z)` quaternion. |
| `registration_shift_in_sensor_m` | Translation fitted to the overmold interior, with no scaling or rotation of the distribution. |
| 0.1 kg and 9.81 m/s² | Known-load fixture; their product is the expected 0.981 N. |
| 20 g, 10,000 N/m, 50 N s/m | Compliant rigid-body simulation parameters, not measured hardware properties. |
| 1/240 s, 192 position and 4 velocity iterations | Conservative validated demo solver settings; not a claimed optimal configuration. |
| 0.1 mm contact offset; SDF resolutions 384/128 | Numerical contact margin and field resolutions for the sensor/test objects; unrelated to taxel count. |
| 1 ms SDF probe time | Finite-difference half-width used to estimate depth rate, independent of the physics timestep. |
| 5% load and 1 mm thumb-centroid tolerances | Verification acceptance bounds, not physical sensor specifications. |

Camera positions, colors, gear scale and plot cadence are presentation choices.
Joint-holding gains are inherited demo settings; they are not identified DG-5F
actuator parameters. Contact sampling and geometry have been checked in the
supplied pose; full articulated clearance and hardware force calibration remain
unverified. The normal-force distribution follows the net PhysX contact result
and cannot resolve independently measured multiple simultaneous contact loads.

## Troubleshooting, contributions and license

For mesh loading failures, run `git lfs pull`. For `egl-probe` build failures,
use the setup script's CMake 3.x and check GCC/G++/Make. If streaming fails,
check startup logs and network ports. Activate the installation environment
before launching and import simulator modules after `AppLauncher`. Include
the command, versions, GPU model and final error lines when opening an issue.

See [CONTRIBUTING.md](CONTRIBUTING.md) for test dependencies, formatting and
required Developer Certificate of Origin sign-offs. ADI-authored code and
assets use [Apache-2.0](LICENSE). Isaac Lab adaptations and the Tesollo hand
retain BSD-3-Clause terms in [NOTICE](NOTICE); the exact Tesollo license is
retained in [LICENSE_Tesollo](LICENSE_Tesollo). The demos fetch the gear from
NVIDIA's asset server under NVIDIA's terms; it is not redistributed here.
