# First Newton simulation

Complete [installation](installation.md), activate `newton/.isaaclab/.venv`,
and run all commands on this page from `newton/`.

## Run and check a known load

```bash
python scripts/dex01_newton.py --mode hand --indenter flat --max-steps 1440 --report /tmp/newton-hand.json
python scripts/dex01_newton.py --check-report /tmp/newton-hand.json
```

`NEWTON_SCENE_READY` marks startup. A 0.1 kg local fixture is placed over
fingers 2–5 in turn, 360 physics ticks per finger at 1/240 s. A loaded finger
should settle near 0.981 N while the other frames remain zero. The report
checker verifies every tick, mapping, filtered normal force, settling and
momentum balance; inspecting the final number does not substitute for it.

For a live window, install `--kit` and use the
[viewing command](installation.md#view-locally-or-through-webrtc).
The gear option is a footprint demonstration, not a known-load fixture.

Continue with [examples](examples/index.md),
[attachment and binding](attachment.md),
[reading tactile data](../reference/data.md) and [validation](validation.md).
