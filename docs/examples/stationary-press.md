# Stationary-object pressing

Finger 2 of the Tesollo hand bends toward a fixed gear, holds contact, then
retracts. The wrist and gear remain stationary, so the changing tactile
footprint follows the finger's motion.

<figure class="example-video">
<video controls playsinline loop preload="none" width="960" height="800"
       poster="../../_static/videos/stationary-press.webp"
       aria-label="Finger 2 approaches, holds contact, and retracts. Playback is 2× slower than simulation time.">
  <source src="../../_static/videos/stationary-press.webm" type="video/webm">
  <source src="../../_static/videos/stationary-press.mp4" type="video/mp4">
  <track kind="captions" src="../../_static/videos/stationary-press.vtt" srclang="en" label="Phases">
  <a href="../../_static/videos/stationary-press.mp4">Download the example video</a>
</video>
<figcaption>Finger 2 approaches, holds contact, and retracts. Playback is 2× slower than simulation time.</figcaption>
</figure>

<div class="phase-gallery">
<figure><img src="../../_static/videos/stationary-press-before.webp" alt="Straight finger with zero load"><figcaption><strong>Unloaded</strong><span>Wrist and gear remain fixed.</span></figcaption></figure>
<figure><img src="../../_static/videos/stationary-press-contact.webp" alt="Finger 2 pressing the stationary gear"><figcaption><strong>Hold contact</strong><span>The fingertip moves and its frame becomes active.</span></figcaption></figure>
<figure><img src="../../_static/videos/stationary-press-after.webp" alt="Finger extended with zero load"><figcaption><strong>Release</strong><span>The finger extends and force returns to zero.</span></figcaption></figure>
</div>

## Run the example

From the repository root in the [installed environment](../getting-started/installation.md):

```bash
python scripts/dex01_stationary_press.py
```

The default cycle lasts 2.5 simulation seconds and repeats until closed.
`--press-time-scale` changes the trajectory duration; elapsed wall-clock time
also depends on host performance. For remote viewing, follow the
[streaming setup](../getting-started/installation.md#view-a-remote-simulation).

## Read the tactile output

Finger 2's frame becomes active during approach and hold, then returns to
zero after retraction. The other four frames remain unloaded. The contact
close-up shows the fingertip's planar patch meeting the gear's underside;
the loaded samples surround the central hole.

The displayed load is the measured normal contact force. The controller's
force setting is a contact-seeking threshold, so it does not guarantee a
constant force during hold. Individual gear teeth may be too small to resolve
on this fingertip grid. See [display colors and units](index.md#read-the-tactile-display).

Use the [articulated-press validation](../validation.md#articulated-pressing)
to check motion, alignment, contact coverage, and release. For a stationary
hand with a repositioned object, see [sequential fingertip contact](hand.md).
