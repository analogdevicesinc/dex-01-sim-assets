# Grasp, hold, and release

The Robotiq 2F-85 closes its two rectangular tactile fingers around a free
rigid block, holds contact, and opens again. Each finger produces a 32 × 32
tactile frame. Gravity and damping are disabled on the block so its movement
reflects the gripper's contact forces.

<figure class="example-video">
<video controls playsinline loop preload="none" width="960" height="800"
       poster="../../_static/videos/robotiq.webp"
       aria-label="A six-second grasp, hold, and release cycle at simulation-time speed, with both tactile frames and a contact close-up.">
  <source src="../../_static/videos/robotiq.webm" type="video/webm">
  <source src="../../_static/videos/robotiq.mp4" type="video/mp4">
  <track kind="captions" src="../../_static/videos/robotiq.vtt" srclang="en" label="Phases">
  <a href="../../_static/videos/robotiq.mp4">Download the example video</a>
</video>
<figcaption>A six-second grasp, hold, and release cycle at simulation-time speed, with both tactile frames and a contact close-up.</figcaption>
</figure>

<div class="phase-gallery">
<figure><img src="../../_static/videos/robotiq-before.webp" alt="Open gripper with zero readings on both pads"><figcaption><strong>Before contact</strong><span>The block is between the open fingers.</span></figcaption></figure>
<figure><img src="../../_static/videos/robotiq-contact.webp" alt="Both fingers grasping the block, with active left and right tactile images"><figcaption><strong>Hold</strong><span>Both pads report contact.</span></figcaption></figure>
<figure><img src="../../_static/videos/robotiq-after.webp" alt="Reopened gripper with both tactile images back at zero"><figcaption><strong>Release</strong><span>Both pads unload.</span></figcaption></figure>
</div>

## Run the example

From the repository root in the [installed environment](../getting-started/installation.md):

```bash
python scripts/dex01_on_robotiq_2f85.py
```

The example runs one six-second cycle at 240 Hz. Its fixed-base gripper acts
on a 0.1 kg, 10 × 30 × 18 mm block. Isaac Sim loads the official gripper from
its configured asset library; see the [Robotiq integration guide](../guides/robotiq.md)
for a local library or an alternative asset root.

## Read the tactile output

Both frames start unloaded, develop a contact footprint as the jaws close,
and return to zero after release. The labels show each finger's total normal
force in newtons. During hold, the two loads should be close; the block can
move in response to any imbalance.

View each contact face with its fingertip up: rows run tip to base and columns
left to right. All 1,024 pixels correspond to sensing sites. See
[display colors and units](index.md#read-the-tactile-display).

The recorded heatmaps preserve the physical taxel spacing, with width
horizontal and length vertical. They show the sampling grid, excluding the
unsampled border of the contact face.

Because gravity is disabled, the readings represent the grasping load rather
than the block's weight. Use the [grasp validation](../validation.md#robotiq-grasp-hold-and-release)
for independent force and momentum checks. For mounting, asset reuse, and
rebuilding, see the [integration guide](../guides/robotiq.md).
