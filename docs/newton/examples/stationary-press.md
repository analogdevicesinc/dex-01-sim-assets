# Newton: stationary press


<figure class="example-video">
<video controls playsinline loop preload="none" width="960" height="800"
       poster="../../../_static/videos/newton/stationary-press.webp"
       aria-label="Validated Newton joint-driven stationary pressing, replaying actual link poses and matching firmware frames.">
  <source src="../../../_static/videos/newton/stationary-press.webm" type="video/webm">
  <source src="../../../_static/videos/newton/stationary-press.mp4" type="video/mp4">
  <track kind="captions" src="../../../_static/videos/newton/stationary-press.vtt" srclang="en" label="Phases">
  <a href="../../../_static/videos/newton/stationary-press.mp4">Download the Newton video</a>
</video>
<figcaption>Newton physics with native RTX replay and matching force frames. The clock is simulation time. Press playback is 2× slower. <a href="../../media/">Capture and render provenance</a>.</figcaption>
</figure>

<div class="phase-gallery">
<figure><img src="../../../_static/videos/newton/stationary-press-before.webp" alt="Initial Newton state before sustained contact"><figcaption><strong>Before contact</strong></figcaption></figure>
<figure><img src="../../../_static/videos/newton/stationary-press-contact.webp" alt="Newton contact and its synchronized active firmware frame"><figcaption><strong>Contact</strong></figcaption></figure>
<figure><img src="../../../_static/videos/newton/stationary-press-after.webp" alt="Final captured Newton state and force frame"><figcaption><strong>Final state</strong></figcaption></figure>
</div>

Finger 2 bends against a stationary gear while the wrist stays fixed. Joint
actuator targets produce approach, hold and retraction in three 2.5 s cycles.

From `newton/` in the activated environment:

```bash
python scripts/dex01_newton.py --mode press --max-steps 1800 --report /tmp/newton-press.json
python scripts/dex01_newton.py --check-report /tmp/newton-press.json
```

The timestep is 1/240 s with 16 solver substeps. Before the run, endpoint
sampling places the gear; runtime motion comes from joint commands. The gear
is scaled to 12 mm across its largest in-plane extent. Joint gains and endpoint
fractions are demonstration parameters, not identified DG-5F hardware settings.

The checker requires every tick, a fixed wrist and object, actual joint and
fingertip travel, sustained contact, return and release. A short run cannot
pass the three-cycle check. The gear's small teeth may be unresolved by the
local taxel spacing; this is not a force-control or hardware-calibration test.

Add `--capture /tmp/newton-press-capture` to export the third complete cycle
after the report passes. See [recording](../recording.md) and
[numerical settings](../numerical-settings.md). See [validation results](../validation.md#verified-newton-fixtures) for the
exact runtime and source evidence. Validate any changed geometry or solver settings again.
