# Explore the examples

Choose the scene that matches what you want to learn. Each page includes a
expected behavior, the command to run it, and how to interpret its tactile
output. Each example includes a video and static states.

<div class="example-gallery">
<a href="flat-pad/"><img src="../_static/videos/flat-pad.webp" alt="Gear contacting a flat tactile grid"><strong>Start with a flat pad</strong><span>See a contact footprint form as a falling gear settles.</span></a>
<a href="hand/"><img src="../_static/videos/hand.webp" alt="Tesollo hand and synchronized finger force frames"><strong>Explore all five fingertips</strong><span>Follow sequential contacts and compare loaded and unloaded fingers.</span></a>
<a href="stationary-press/"><img src="../_static/videos/stationary-press.webp" alt="Finger 2 pressing against a stationary gear"><strong>Watch articulated pressing</strong><span>Connect joint movement with contact, force, and release.</span></a>
<a href="robotiq/"><img src="../_static/videos/robotiq.webp" alt="Robotiq gripper holding a block with two tactile images"><strong>Grasp with the Robotiq 2F-85</strong><span>Watch both rectangular pads load and release.</span></a>
</div>

| Example | Main idea | Expected output |
|---|---|---|
| [Flat pad](flat-pad.md) | Grid sensing and a gravity load | 32 × 32 frame; gear load near 4.905 N |
| [Tesollo hand](hand.md) | Curved fingertip contact | Loaded finger's frame active; other frames zero |
| [Stationary press](stationary-press.md) | Articulation creates contact | Fixed wrist/object; finger motion; hold and release |
| [Robotiq 2F-85](robotiq.md) | Two rectangular tactile fingers on the official gripper | Two 32 × 32 frames; grasp, hold and release |

The videos use actual captured Isaac Sim 5.1 poses and tactile arrays,
rendered separately with native RTX cameras. Slow playback is labeled in the
clip. The force frames preserve their exact pixel mapping and share one scale
within each clip; grey indicates unused cells.
Hand videos include a second native camera view of the contact so the small
gear remains legible while the overview shows all five fingertips.
