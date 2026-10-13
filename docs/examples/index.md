# Explore the examples

Choose the scene that matches what you want to learn. Each page includes a
short video, expected behavior, the command to run it, and how to interpret
its tactile output.

<div class="example-gallery">
<a href="flat-pad/"><img src="../_static/videos/flat-pad.webp" alt="Gear contacting a flat tactile grid"><strong>Start with a flat pad</strong><span>See a contact footprint form as a falling gear settles.</span></a>
<a href="hand/"><img src="../_static/videos/hand.webp" alt="Tesollo hand and synchronized finger force frames"><strong>Explore all five fingertips</strong><span>Follow sequential contacts and compare loaded and unloaded fingers.</span></a>
<a href="stationary-press/"><img src="../_static/videos/stationary-press.webp" alt="Finger 2 pressing against a stationary gear"><strong>Watch articulated pressing</strong><span>Connect joint movement with contact, force, and release.</span></a>
<a href="streaming-recording/"><img src="../_images/streaming-workflow.svg" alt="Live streaming and video recording paths from the simulator"><strong>View and share a run</strong><span>Stream the viewport to a laptop or save a recording with tactile frames.</span></a>
</div>

| Example | Main idea | Expected output |
|---|---|---|
| [Flat pad](flat-pad.md) | Grid sensing and a gravity load | 32 × 32 frame; gear load near 4.905 N |
| [Tesollo hand](hand.md) | Firmware-mapped fingertip contact | Loaded finger's frame active; other frames zero |
| [Stationary press](stationary-press.md) | Articulation creates contact | Fixed wrist/object; finger motion; hold and release |
| [Streaming and recording](streaming-recording.md) | View or share the result | Live viewport on a laptop or a video with tactile frames |

The videos use actual captured Isaac Sim 5.1 poses and tactile arrays,
rendered separately with native RTX cameras. Slow playback is labeled in the
clip. The force frames preserve their exact pixel mapping and share one scale
within each clip; grey indicates unused cells.
Hand videos include a second native camera view of the contact so the small
gear remains legible while the overview shows all five fingertips.
