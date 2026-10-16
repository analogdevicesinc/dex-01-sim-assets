# Your first simulation

Select the backend installed on the [installation page](installation.md).
The first command below is PhysX; Newton has its own command later on this page.

## Start PhysX

Complete [installation](installation.md) and activate its Python environment.
Run from the repository root. On a local desktop with a compatible R580
driver, launch the hand demo:

```bash
python scripts/dex01_on_tesollo_dg5f.py
```

Wait for `Setup complete...`. A gear contacts fingers 2–5 in turn. The viewport
shows the hand, per-finger normal loads in newtons, and 32 × 32 tactile frames.
Stop with Ctrl+C. The first launch can download extensions and compile shaders.

For remote viewing use the [remote viewing setup](installation.md#view-a-remote-simulation).
Learn how to interpret the frames in the [hand example](../examples/hand.md),
or check a reproducible load with the [validation guide](../validation.md).

## First result without a display

```bash
MPLBACKEND=Agg python scripts/dex01_on_tesollo_dg5f.py \
  --headless \
  --flat-indenter \
  --max-steps 240
```

This uses a local 0.1 kg cube and prints physical normal forces. It exits after
240 steps, with finger 2 near 0.981 N and the other fingers at zero. Headless
physics does not display a viewport; for live visual access from a remote
host, follow the [remote viewing setup](installation.md#view-a-remote-simulation).

## First Newton simulation

From the pinned Newton Isaac Lab checkout:

```bash
uv run --frozen python /path/to/dex-01-sim-assets/scripts/dex01_newton.py --mode hand --indenter flat --max-steps 1440 --report /tmp/newton-hand.json
uv run --frozen python /path/to/dex-01-sim-assets/scripts/dex01_newton.py --check-report /tmp/newton-hand.json
```

`NEWTON_SCENE_READY` marks startup. A 0.1 kg flat fixture is placed over fingers
2–5, one finger every 360 ticks. Each tick is 1/240 s. Loaded frames should settle
near 0.981 N while the others remain zero. The checker, rather than this expected
behavior, determines whether the run passes. For viewing use the Kit command
on the installation page. Continue with [hand examples](../examples/hand.md),
[reading data](../guides/read-data.md) and [validation](../validation.md).
