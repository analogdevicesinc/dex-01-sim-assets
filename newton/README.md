# DEX-01 for Newton

Use this directory for the pinned Isaac Lab 3.0 Early Access stack and Python
3.12. From here:

```bash
./setup_env.sh
source .isaaclab/.venv/bin/activate
python scripts/dex01_newton.py --mode hand --indenter flat --max-steps 1440 --report /tmp/newton-hand.json
python scripts/dex01_newton.py --check-report /tmp/newton-hand.json
```

Use `--kit` or `--video` on setup for viewing or recording. The installer syncs
the frozen upstream workspace, then installs only `dex01-sim-common` and
`dex01-sim-newton` from this repository without changing simulation pins.
The default checkout and environment live under `.isaaclab` in this directory.

Follow [Newton installation](../docs/newton/installation.md),
[examples](../docs/newton/examples/index.md), [binding](../docs/newton/attachment.md),
[API](../docs/newton/api.md) and [validation](../docs/newton/validation.md).
CUDA contacts and camera validation remain required. The common CPU checks do
not validate Newton physics. Use [PhysX](../physx/README.md) separately for 5.1.
