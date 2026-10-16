# DEX-01 for PhysX

Use this directory for Isaac Sim 5.1 and Python 3.11. From here:

```bash
./setup_env.sh
source .venv/bin/activate
python scripts/dex01_on_tesollo_dg5f.py
```

Setup installs the pinned PhysX environment, `dex01-sim-common`, and the
`dex01-sim-asset` backend. Defaults are `.venv` and `.isaaclab` in this directory.
Git LFS is required for the shared `../assets` and common taxel files.

Follow [PhysX installation](../docs/physx/installation.md),
[examples](../docs/physx/examples/index.md), [attachment](../docs/physx/attachment.md),
[API](../docs/physx/api.md) and [validation](../docs/physx/validation.md).
Use [Newton](../newton/README.md) in a separate environment when targeting EA.
