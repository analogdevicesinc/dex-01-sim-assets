# Newton: single fingertip

Use a standalone curved DEX-01 sensor to inspect sampling and localization.
It has the same 738 samples and firmware mapping as each mounted fingertip;
it is not a flat 32 × 32 grid.

From `newton/` in the activated environment:

```bash
python scripts/dex01_newton.py --mode sensor --indenter sphere --max-steps 240
python scripts/verify_dex01_newton.py --sweep-all --report /tmp/newton-sweep.json
```

The sweep places the actual mesh sphere at separation, touching and two
indentation depths at all 738 taxels. It checks analytic sphere depth,
normal-load conservation and two-environment isolation. A default bounded
verification without `--sweep-all` samples five representative locations.

This mode's saved demo report is not a hand-load report. Use the sweep for
contact verification and the [hand example](hand.md) for the 0.981 N fixture.
The [shared patterns guide](../../concepts/patterns.md) describes custom grids.
