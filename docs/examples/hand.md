# Tesollo hand demo

The video and existing example commands on this page use **PhysX**.
The Newton section describes the corresponding workflow and its differences.

Watch the object contact fingers 2–5 in turn. The active tactile frame follows
the loaded finger while the remaining fingers stay unloaded.

```{warning}
The bundled PhysX hand and press media require regeneration after a scale-export
correction. They are not Newton recordings. See the
[media provenance notice](https://github.com/analogdevicesinc/dex-01-sim-assets/blob/main/docs/media/README.md).
```

<figure class="example-video">
<video controls playsinline loop preload="none" width="960" height="800"
       poster="../../_static/videos/hand.webp"
       aria-label="A captured 5-second hand sequence at simulation-time playback, with synchronized per-finger load and firmware frames.">
  <source src="../../_static/videos/hand.webm" type="video/webm">
  <source src="../../_static/videos/hand.mp4" type="video/mp4">
  <track kind="captions" src="../../_static/videos/hand.vtt" srclang="en" label="Phases">
  <a href="../../_static/videos/hand.mp4">Download the example video</a>
</video>
<figcaption>A captured 5-second hand sequence at simulation-time playback, with synchronized per-finger load and firmware frames.</figcaption>
</figure>

<div class="phase-gallery">
<figure><img src="../../_static/videos/hand-before.webp" alt="The gear starts above finger 2 with unloaded tactile frames"><figcaption><strong>Start on finger 2</strong><span>The gear begins above the fingertip.</span></figcaption></figure>
<figure><img src="../../_static/videos/hand-contact.webp" alt="Finger 2 supports the gear and its force frame is active"><figcaption><strong>Read the loaded frame</strong><span>The close-up shows the small contact object.</span></figcaption></figure>
<figure><img src="../../_static/videos/hand-after.webp" alt="The gear is now supported by finger 5 and its frame is active"><figcaption><strong>Advance to finger 5</strong><span>The active frame follows the loaded finger.</span></figcaption></figure>
</div>

## Read the sequence

1. The gear is placed above finger 2 and drops into contact.
2. The contact frame shows the sampled footprint, with its total force in newtons.
3. The demo resets the gear over each following finger; the active frame moves
   across fingers 3, 4, and 5. Finger 1 is checked separately by the thumb probe.

The native camera inset follows the gear at its physical size (about 6.29 mm
wide). Match that contact to the active patch in the corresponding finger's
unwrapped frame; the heatmap shows force per taxel, not an optical image.

## Run the example

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

## Newton hand load

From the pinned Newton Lab checkout:

```bash
uv run --frozen python /path/to/dex-01-sim-assets/scripts/dex01_newton.py --mode hand --indenter flat --max-steps 1440 --report /tmp/newton-hand.json
uv run --frozen python /path/to/dex-01-sim-assets/scripts/dex01_newton.py --check-report /tmp/newton-hand.json
```

The 0.1 kg fixture loads fingers 2–5 for 360 ticks each at 1/240 s. Use the
flat fixture for the 0.981 N check; `--indenter gear` is a footprint demo.
Finger 1 is displayed but this hand checker does not perform the PhysX thumb
probe. Newton places the gear using its own centered mesh and dimensions;
the 6.29 mm PhysX gear dimension is not a Newton setting.

Add `--capture /tmp/newton-hand-capture` to export audited environment-zero
geometry, poses and force arrays into a new directory. The complete report
must pass before the capture is published. See [recording and replay](../getting-started/newton.md#capture-and-render-newton-evidence).
