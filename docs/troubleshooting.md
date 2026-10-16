# Troubleshooting

For mesh loading failures, run `git lfs pull`. For `egl-probe` build failures,
use the setup script's CMake 3.x and check GCC/G++/Make. If streaming fails,
check startup logs and network ports. Activate the installation environment
before launching and import simulator modules after `AppLauncher`. Include
the command, versions, GPU model and final error lines when opening an issue.

For remote access, see [remote viewing setup](getting-started/installation.md#view-a-remote-simulation).
For force mismatches, run [validation](validation.md) with the flat fixture
and check [numerical settings](reference/numerical-settings.md).

## Renderer exits with code 139

If camera or viewport startup crashes in `librtx.scenedb.plugin.so`, check the
GPU driver against Isaac Sim **5.1.0** requirements. A newer driver can run
headless physics while failing RTX rendering.

NVIDIA identifies incompatible newer driver branches in
[Isaac Sim issue 619](https://github.com/isaac-sim/IsaacSim/issues/619) and
[issue 650](https://github.com/isaac-sim/IsaacSim/issues/650). The validated Linux
driver is 580.65.06 (R580). Configure the supported driver through your host's
normal maintenance process, then retry the visual demo.

## Read a tactile frame

Grey visualization cells have no taxel. Active pixels at zero force use the
zero-load color. The unwrapped image retains firmware `(row, column)` indices;
it is not a camera picture. See [reading data](guides/read-data.md).

## Newton startup and data errors

| Symptom | Action |
|---|---|
| Unexpected checkout or package versions | Use the exact `setup_newton_env.sh` revision and `uv run --frozen` from that Lab checkout; do not reuse the PhysX environment |
| CUDA unavailable | Fix host driver/device access, then rerun preflight and required GPU tests; CPU mode does not validate Newton SDF integration |
| Data read before binding | Reset the scene, bind providers/contact/mesh, then update the sensor |
| Contact paths or force matrix rejected | Configure one sensing link and one matching counter-object filter per environment with update period zero |
| Closed mesh or transform error | Check triangle topology, degeneracy, scale and unit XYZW collider quaternion |
| Report rejects missing ticks, interruption or settling | Preserve the failed report, run complete cycles, and investigate the physical fixture rather than deleting observations |
| Native collision capacity exhausted | Inspect contact counts and scene complexity; a saturated run is invalid evidence |
| Recording/capture rejected | Check frame cadence, array mapping, geometry and hashes; regenerate from actual simulation states |

For Kit use the `isaacsim` extra and `--viz kit`; connect after
`NEWTON_SCENE_READY`. Recording also requires the `video` extra and FFmpeg.
The Isaac Sim 5.1 crash diagnosis above applies to PhysX; Newton uses the EA Kit
runtime and needs its own camera test. Include the Newton command, commit,
preflight JSON, GPU/driver, package versions and final log lines in an issue.
