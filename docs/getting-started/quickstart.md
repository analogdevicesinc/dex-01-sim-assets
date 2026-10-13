# Your first simulation

Complete [installation](installation.md) and activate its Python environment.
Run from the repository root. On a local desktop with a compatible R580
driver, launch the hand demo:

```bash
python scripts/dex01_on_tesollo_dg5f.py
```

Wait for `Setup complete...`. A gear contacts fingers 2–5 in turn. The viewport
shows the hand, per-finger normal loads in newtons, and 32 × 32 tactile frames.
Stop with Ctrl+C. The first launch can download extensions and compile shaders.

For remote viewing use the [streaming guide](../examples/streaming-recording.md).
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
host, follow the WebRTC client and network setup in the streaming guide.
