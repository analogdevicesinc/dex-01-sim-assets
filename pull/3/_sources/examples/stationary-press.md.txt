# Press a stationary gear

See how joint motion creates and releases tactile contact. Finger 2 bends
against the fixed gear while the wrist and object stay in place.

<figure class="example-video">
<video controls playsinline loop preload="none" width="960" height="800"
       poster="../../_static/videos/stationary-press.webp"
       aria-label="One captured press cycle at 2× slower playback: unloaded, approach, hold, retract, and release. Only finger 2 becomes loaded.">
  <source src="../../_static/videos/stationary-press.webm" type="video/webm">
  <source src="../../_static/videos/stationary-press.mp4" type="video/mp4">
  <track kind="captions" src="../../_static/videos/stationary-press.vtt" srclang="en" label="Phases">
  <a href="../../_static/videos/stationary-press.mp4">Download the example video</a>
</video>
<figcaption>One captured press cycle at 2× slower playback: unloaded, approach, hold, retract, and release. Only finger 2 becomes loaded.</figcaption>
</figure>

<div class="phase-gallery">
<figure><img src="../../_static/videos/stationary-press-before.webp" alt="Straight finger with zero load"><figcaption><strong>Unloaded</strong><span>Wrist and gear remain fixed.</span></figcaption></figure>
<figure><img src="../../_static/videos/stationary-press-contact.webp" alt="Finger 2 pressing the stationary gear"><figcaption><strong>Hold contact</strong><span>The fingertip moves and its frame becomes active.</span></figcaption></figure>
<figure><img src="../../_static/videos/stationary-press-after.webp" alt="Finger extended with zero load"><figcaption><strong>Release</strong><span>The finger extends and force returns to zero.</span></figcaption></figure>
</div>

The camera inset makes the 8.25 mm gear contact visible without enlarging the
object in the simulation. During hold, only finger 2's frame is active; after
release all five frames return to zero.

The finger aligns its planar pad with the gear's underside before loading.
During hold, joint targets maintain that orientation while advancing the pad
along the surface normal. The contact footprint spans the underside around
the gear's central hole.

## Run the example

Prerequisites: activate the [installed environment](../getting-started/installation.md).
The command below uses remote streaming; omit `--livestream=2` on a local desktop.

```bash
MPLBACKEND=Agg python scripts/dex01_stationary_press.py --livestream=2
```

The wrist remains fixed. Finger 2's MCP, PIP and DIP flexion joints are driven
with actuator position targets to bend the fingertip toward the stationary gear,
hold contact, then extend back to rest. The original fixed-base hand is used;
there is no sliding fixture. Setup samples a settled articulated endpoint and
selects only coplanar taxels, excluding the rounded transition. The gear's
underside must fit within that planar patch.
Runtime motion uses joint targets, not joint-state or root-pose teleportation.

The GUI shows the fixed wrist, visible finger articulation, measured loads in
newtons and firmware contact footprints. The force is read from PhysX and
redistributed across penetrating taxels. Joint gains and endpoint fractions are
simulation demo settings, not DG-5F hardware specifications.

Run three cycles and independently verify wrist/object invariance, actual
joint movement, fingertip travel, face alignment, coverage, mapping and release:

```bash
MPLBACKEND=Agg python scripts/dex01_stationary_press.py \
  --livestream=2 \
  --max-steps 900 \
  --verification-report /tmp/dex01-articulated.json
python scripts/dex01_stationary_press.py \
  --check-report /tmp/dex01-articulated.json
```

The planar-contact checker requires at least 90% of the taxels under the
actual triangulated underside to carry load, with load in all four quadrants.
It also checks face-normal mismatch below 0.05° and no underside edge more
than 2 µm above the pad plane. These are numerical acceptance tolerances;
an edge-contact strip fails even if its total force is nonzero.

`--press-time-scale` controls the physical joint trajectory timing. The default
cycle is 2.5 simulation seconds; observed duration depends on host performance. The target
force is a contact-seeking threshold, not guaranteed force tracking. A small
gear's individual teeth may not resolve in the sparse fingertip contact patch.
The existing flat-pad example is suited to viewing broader object footprints.
