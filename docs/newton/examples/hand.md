# Newton: Tesollo hand


<figure class="example-video">
<video controls playsinline loop preload="none" width="960" height="800"
       poster="../../../_static/videos/newton/hand.webp"
       aria-label="Validated Newton known hand loads, replaying actual link poses and matching firmware frames.">
  <source src="../../../_static/videos/newton/hand.webm" type="video/webm">
  <source src="../../../_static/videos/newton/hand.mp4" type="video/mp4">
  <track kind="captions" src="../../../_static/videos/newton/hand.vtt" srclang="en" label="Phases">
  <a href="../../../_static/videos/newton/hand.mp4">Download the Newton video</a>
</video>
<figcaption>Newton physics with native RTX replay and matching force frames. The clock is simulation time. Hand playback is at simulation speed. <a href="../../media/">Capture and render provenance</a>.</figcaption>
</figure>

<div class="phase-gallery">
<figure><img src="../../../_static/videos/newton/hand-before.webp" alt="Initial Newton state before sustained contact"><figcaption><strong>Before contact</strong></figcaption></figure>
<figure><img src="../../../_static/videos/newton/hand-contact.webp" alt="Newton contact and its synchronized active firmware frame"><figcaption><strong>Contact</strong></figcaption></figure>
<figure><img src="../../../_static/videos/newton/hand-after.webp" alt="Final captured Newton state and force frame"><figcaption><strong>Final state</strong></figcaption></figure>
</div>

Place a known flat fixture over fingers 2–5 and follow which firmware frame
becomes active. All five fingertips use shared geometry and 740 taxels.

From `newton/` in the activated environment:

```bash
python scripts/dex01_newton.py --mode hand --indenter flat --max-steps 1440 --report /tmp/newton-hand.json
python scripts/dex01_newton.py --check-report /tmp/newton-hand.json
```

A 0.1 kg fixture loads each finger for 360 ticks at 1/240 s. Expected settling
is 0.981 N with zero force on unloaded fingers. The checker also tests force
filtering, frame mapping and momentum balance. Finger 1 is displayed but this
workflow does not perform the separate PhysX thumb probe.

`--indenter gear` displays a small contact footprint; its mesh is centered and
scaled to 6 mm across its largest in-plane extent. It can rock and is not a
known-weight verification fixture. Use the
[Kit setup](../installation.md#view-locally-or-through-webrtc) for viewing.

Add `--capture /tmp/newton-hand-capture` with a new destination to export
verified geometry, poses and force arrays from environment zero. The complete
flat-load report must pass before the capture is published. See
[recording and replay](../recording.md).
