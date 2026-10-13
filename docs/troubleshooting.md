# Troubleshooting

For mesh loading failures, run `git lfs pull`. For `egl-probe` build failures,
use the setup script's CMake 3.x and check GCC/G++/Make. If streaming fails,
check startup logs and network ports. Activate the installation environment
before launching and import simulator modules after `AppLauncher`. Include
the command, versions, GPU model and final error lines when opening an issue.

For remote access, see [streaming](examples/streaming-recording.md).
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
