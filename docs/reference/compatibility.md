# Compatibility

| Component | PhysX tested baseline |
|---|---|
| Host | Linux x86_64, GLIBC 2.35 or newer |
| Python | 3.11 |
| Isaac Sim | 5.1.0 |
| Isaac Lab | `b0542fe2d45bf91c4e1d9ef6952b9c709c80b4e8` |
| GPU | NVIDIA RTX-capable GPU; see NVIDIA’s 5.1 requirements for VRAM |
| Linux display/camera driver | NVIDIA-validated 580.65.06 (R580) for Isaac Sim 5.1.0 |

The setup script pins a fresh Isaac Lab checkout to the tested revision and
preserves an existing checkout, reporting any revision difference. An existing
checkout is not thereby validated. Newer upstream documentation can describe
APIs and backends that this release does not support.

The package version is recorded in `source/dex01_sim_asset/config/extension.toml`.
Use documentation matching your release and include the release or commit in
bug reports. The `v0.1.0-a.1` snapshot is a prerelease, not a stable release.

Documentation builds use a separate Python environment and require no GPU,
Isaac Sim, CUDA, or simulation asset downloads.

## Newton pinned baseline

| Component | Newton configuration |
|---|---|
| Host | Linux x86_64 with working NVIDIA CUDA driver |
| Python | 3.12, isolated from PhysX |
| Isaac Lab | `v3.0.0-EA`, `ae37b028ea415c91ea2bc32609efcd759ed2b974` |
| Newton / Warp | 1.5.2 / 1.16.0 |
| MuJoCo / MuJoCo-Warp | 3.11.0 / 3.11.0 |
| Torch | 2.11.0 in upstream frozen lockfile |
| Kit viewing / streaming | Isaac Sim 6.1.0; `isaacsim` extra |
| Video | `isaacsim` and `video` extras; live camera uses NewtonWarpRenderer |
| Documentation RTX replay | Separate rendering environment; recorded physics stays stopped |

CPU mathematics, report, capture and recording tests run on Python 3.11 and 3.12.
They do not replace CUDA integration. Current Newton GPU physics, live camera
and replay media validation remain pending. The PhysX R580 driver statement is
specific to Isaac Sim 5.1; check the EA runtime's host requirements independently.
