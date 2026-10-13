# Tesollo hand demo

Prerequisites: [installed environment](../getting-started/installation.md),
a supported NVIDIA GPU, and fetched Git LFS assets.

For a local desktop:

```bash
python scripts/dex01_on_tesollo_dg5f.py
```

A gear contacts fingers 2–5 in turn. The Isaac Sim viewport includes live
firmware heatmaps and each finger's measured normal load in newtons, so they
appear in the WebRTC GUI. Blue is zero load; cyan, yellow and red show increasing
force per taxel, with a shared adaptive scale printed below the images. Grey
cells have no sensor taxel. Firmware row/column positions are preserved, with
no smoothing: the contact footprint (including gear teeth when resolved by the
sampling) is visible rather than just an aggregate force. These frames unwrap
the curved fingertip; they are not an optical picture of the object.

## Inspect a known contact

```{figure} ../media/loaded-contact.webp
:alt: A 0.1 kilogram cube resting on the DEX-01 pad of finger 2, with the mounting frame and adjacent fingertips visible.
:width: 100%

The local flat-load fixture on finger 2. The corresponding tactile frame
is active only on the loaded finger; the total load is near 0.981 N after settling.
```

Use a flat indenter for quantitative [known-load validation](../validation.md).
The gear can rock and is not a calibration fixture.
