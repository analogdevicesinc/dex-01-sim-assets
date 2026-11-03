# Sequential fingertip contact

A gear is placed above each of the Tesollo hand's fingers 2–5 in turn and
falls into contact. Compare the loaded fingertip's tactile frame with the
unloaded frames. The thumb is outside this sequence.

<figure class="example-video">
<video controls playsinline loop preload="none" width="960" height="800"
       poster="../../_static/videos/hand.webp"
       aria-label="Five seconds of sequential contact on fingers 2–5, shown at simulation-time speed.">
  <source src="../../_static/videos/hand.webm" type="video/webm">
  <source src="../../_static/videos/hand.mp4" type="video/mp4">
  <track kind="captions" src="../../_static/videos/hand.vtt" srclang="en" label="Phases">
  <a href="../../_static/videos/hand.mp4">Download the example video</a>
</video>
<figcaption>Five seconds of sequential contact on fingers 2–5, shown at simulation-time speed.</figcaption>
</figure>

<div class="phase-gallery">
<figure><img src="../../_static/videos/hand-before.webp" alt="The gear starts above finger 2 with unloaded tactile frames"><figcaption><strong>Start on finger 2</strong><span>The gear begins above the fingertip.</span></figcaption></figure>
<figure><img src="../../_static/videos/hand-contact.webp" alt="Finger 2 supports the gear and its force frame is active"><figcaption><strong>Read the loaded frame</strong><span>The close-up shows the small contact object.</span></figcaption></figure>
<figure><img src="../../_static/videos/hand-after.webp" alt="The gear is now supported by finger 5 and its frame is active"><figcaption><strong>Advance to finger 5</strong><span>The active frame follows the loaded finger.</span></figcaption></figure>
</div>

## Run the example

From the repository root in the [installed environment](../getting-started/installation.md):

```bash
python scripts/dex01_on_tesollo_dg5f.py
```

The sequence repeats until closed. The gear is repositioned above each finger;
the hand holds its pose. For contact driven by finger motion, run
[stationary-object pressing](stationary-press.md).

## Read the tactile output

The viewport shows five tactile frames and each finger's total normal load
in newtons. Contact moves across frames 2, 3, 4, and 5 as the gear advances;
the other frames remain unloaded. The video inset shows which fingertip is
supporting the gear.

Each frame unwraps a curved fingertip onto a 32 × 32 image with 740 sensing
sites. Match the active patch to the contact location rather than expecting
an optical outline of the gear. See
[display colors and units](index.md#read-the-tactile-display).

The gear can rock, so use the [flat-load fixture and optional thumb probe](../validation.md)
for quantitative force checks and thumb coverage.
