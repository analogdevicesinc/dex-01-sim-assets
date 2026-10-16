# Repository layout

Choose an environment at the top level before setup. There is no combined
simulator installer.

```text
physx/                  Isaac Sim 5.1 / Python 3.11
  setup_env.sh
  source/dex01_sim_asset/
  scripts/              PhysX examples and capture tools
  tests/                PhysX behavior and report checks
newton/                 Isaac Lab 3.0 EA / Python 3.12
  setup_env.sh
  source/dex01_sim_newton/
  scripts/              Newton examples, reports and captures
  tests/                Newton behavior and GPU integration
common/
  source/dex01_sim_common/
  tests/                Geometry, pattern and visualization contracts
assets/                 One copy of exterior and Tesollo geometry
scripts/                Asset build and shared offline rendering/publishing
tests/                  Shared rendering, packaging and documentation checks
docs/
  physx/                Complete PhysX workflow
  newton/               Complete Newton workflow
  concepts/, reference/ Shared sensor model and output contract
```

The Python distributions have separate namespaces. `dex01-sim-asset` retains
the PhysX namespace; `dex01-sim-newton` owns Newton. Both use
`dex01-sim-common` for patterns, data and visual helpers. Shared code has no
import of either sensor backend. Scene execution remains in each environment's
scripts, not in the installed common library.

Default virtual environments and upstream checkouts live inside their target
directory and are Git-ignored. Shared assets are never duplicated by backend;
Newton authors its collider overlay in a temporary cache. Recorded media
provenance retains the source paths from the revision that produced it.
