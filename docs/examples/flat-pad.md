# Falling-object contact

A 0.5 kg gear falls onto a flat pad with a 32 × 32 sampling grid. Contact
creates a footprint, and the total normal force approaches the gear's
4.905 N weight as it settles.

<figure class="example-video">
<video controls playsinline loop preload="none" width="960" height="800"
       poster="../../_static/videos/flat-pad.webp"
       aria-label="A falling gear loads the flat pad and settles. Playback is 3× slower than simulation time.">
  <source src="../../_static/videos/flat-pad.webm" type="video/webm">
  <source src="../../_static/videos/flat-pad.mp4" type="video/mp4">
  <track kind="captions" src="../../_static/videos/flat-pad.vtt" srclang="en" label="Phases">
  <a href="../../_static/videos/flat-pad.mp4">Download the example video</a>
</video>
<figcaption>A falling gear loads the flat pad and settles. Playback is 3× slower than simulation time.</figcaption>
</figure>

<div class="phase-gallery">
<figure><img src="../../_static/videos/flat-pad-before.webp" alt="Gear above the unloaded pad"><figcaption><strong>Before contact</strong><span>The grid starts unloaded.</span></figcaption></figure>
<figure><img src="../../_static/videos/flat-pad-contact.webp" alt="Gear touching the pad with an active tactile footprint"><figcaption><strong>Impact and contact</strong><span>A localized footprint develops.</span></figcaption></figure>
<figure><img src="../../_static/videos/flat-pad-after.webp" alt="Gear settled on the pad"><figcaption><strong>Settled load</strong><span>The load approaches 4.905 N.</span></figcaption></figure>
</div>

## Run the example

From the repository root in the [installed environment](../getting-started/installation.md):

```bash
python scripts/simple_example.py
```

The object resets every 100 simulation steps; the scene repeats until closed.
Sampling markers are visible by default. For a bounded run without a window:

```bash
python scripts/simple_example.py --headless --max-steps 120
```

## Read the tactile output

The heatmap is a top view of the 200 × 100 mm pad, shown at its physical 2:1
aspect ratio. Columns follow the long axis and rows the short axis. The
footprint changes during impact, then steadies as the gear settles. Resetting
the gear unloads the pad.

The console prints peak force per taxel and total normal load. Expect the
settled total to approach 4.905 N; impact readings can be higher.
See [display colors and units](index.md#read-the-tactile-display).

For quantitative checks on the curved DEX-01 fingertip, use the separate
[known-load validation](../validation.md). To compare this grid with other
layouts, see [taxel patterns](../guides/patterns.md).
