# Environment compatibility

Choose the environment matching the task and its pinned upstream stack.
Keep the two Python installations separate.

| Component | PhysX | Newton |
|---|---|---|
| Python | 3.11 | 3.12 |
| Isaac Lab | `b0542fe2d45bf91c4e1d9ef6952b9c709c80b4e8` | `v3.0.0-EA`, `ae37b028ea415c91ea2bc32609efcd759ed2b974` |
| Physics | Isaac Sim 5.1 / PhysX | Newton 1.5.2, Warp 1.16.0, MuJoCo/MuJoCo-Warp 3.11.0 |
| Kit | Isaac Sim 5.1 | Isaac Sim 6.1 EA, optional |
| Torch | 2.7.0 | 2.11.0 in frozen workspace |
| Setup | `physx/setup_env.sh` | `newton/setup_env.sh` |
| Installed backend | `dex01-sim-asset` | `dex01-sim-newton` |
| Host | Linux x86_64, GLIBC 2.35+, supported NVIDIA GPU | Linux x86_64, working CUDA driver; EA requirements for Kit |
| Recorded validation | [Specified PhysX fixtures](../physx/validation.md) | [Recorded CUDA, load, press and camera checks](../newton/validation.md) |

The R580 / 580.65.06 display-driver recommendation is specific to Isaac Sim 5.1.
A visible GPU or passing preflight does not establish camera compatibility.
The shared data and pattern library is installed into each environment without
installing the other backend. Documentation uses its own CPU-only toolchain.
