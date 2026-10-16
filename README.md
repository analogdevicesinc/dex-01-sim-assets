# DEX-01 tactile simulation

A physically scaled DEX-01 fingertip for NVIDIA Isaac Lab, with 738 contact
samples mapped into a 32 × 32 firmware force frame. Shared geometry and data
are used by two independent simulation environments.

[![CI](https://github.com/analogdevicesinc/dex-01-sim-assets/actions/workflows/ci.yml/badge.svg)](https://github.com/analogdevicesinc/dex-01-sim-assets/actions/workflows/ci.yml)
[![Documentation](https://github.com/analogdevicesinc/dex-01-sim-assets/actions/workflows/docs.yml/badge.svg)](https://analogdevicesinc.github.io/dex-01-sim-assets/)
[![License Apache-2.0](https://img.shields.io/badge/license-Apache--2.0-blue)](LICENSE)

## Choose an environment

| Target | Repository entry | Documentation |
|---|---|---|
| Isaac Sim 5.1 / PhysX, Python 3.11 | [physx/](physx/README.md) | [PhysX workflow](docs/physx/index.md) |
| Isaac Lab 3.0 EA / Newton, Python 3.12 | [newton/](newton/README.md) | [Newton workflow](docs/newton/index.md) |

Start in the target directory and use its own installer and virtual environment.
Backend packages have no dependency on each other; `common/` and `assets/`
provide one shared set of patterns, buffers and physical geometry.

[Documentation](https://analogdevicesinc.github.io/dex-01-sim-assets/) ·
[Examples by environment](docs/examples/index.md) ·
[Sensor model](docs/concepts/model.md) · [Data contract](docs/reference/data.md) ·
[Repository layout](docs/reference/repository.md)

Known-load fixtures test normal force against 0.981 N; output is a simulated
normal-load approximation, not calibrated hardware readings. PhysX fixture
results and Newton validation status are recorded separately in their tracks.

## Contribute and license

Read [CONTRIBUTING.md](CONTRIBUTING.md) for development checks and required DCO
sign-offs. ADI-authored material uses [Apache-2.0](LICENSE); Isaac Lab adaptations
and Tesollo geometry retain BSD-3-Clause terms in [NOTICE](NOTICE) and
[LICENSE_Tesollo](LICENSE_Tesollo). The NVIDIA gear is fetched at runtime under
NVIDIA's terms and is not redistributed here.

## Citation

If you use this repository in your work, please cite it as:

```bibtex
@misc{analogdevices2026dex01,
  author       = {{Analog Devices, Inc.}},
  title        = {{DEX-01 Tactile Sensor Simulation Asset}},
  year         = {2026},
  howpublished = {GitHub repository},
  url          = {https://github.com/analogdevicesinc/dex-01-sim-assets}
}
```

Include the commit hash or release tag you used for reproducibility.
