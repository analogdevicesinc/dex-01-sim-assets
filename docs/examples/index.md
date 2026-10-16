# Explore the examples

The gallery videos are **PhysX** captures. Use the backend command table below
for Newton. Newton media have not been produced or validated on this checkout;
the PhysX clips illustrate the tasks without serving as Newton evidence.

Choose the scene that matches what you want to learn. Each page includes a
short video, expected behavior, the command to run it, and how to interpret
its tactile output.

<div class="example-gallery">
<a href="flat-pad/"><img src="../_static/videos/flat-pad.webp" alt="Gear contacting a flat tactile grid"><strong>Start with a flat pad</strong><span>See a contact footprint form as a falling gear settles.</span></a>
<a href="hand/"><img src="../_static/videos/hand.webp" alt="Tesollo hand and synchronized finger force frames"><strong>Explore all five fingertips</strong><span>Follow sequential contacts and compare loaded and unloaded fingers.</span></a>
<a href="stationary-press/"><img src="../_static/videos/stationary-press.webp" alt="Finger 2 pressing against a stationary gear"><strong>Watch articulated pressing</strong><span>Connect joint movement with contact, force, and release.</span></a>
</div>

| Example | Main idea | Expected output |
|---|---|---|
| [Flat pad](flat-pad.md) | Grid sensing and a gravity load | 32 × 32 frame; gear load near 4.905 N |
| [Tesollo hand](hand.md) | Firmware-mapped fingertip contact | Loaded finger's frame active; other frames zero |
| [Stationary press](stationary-press.md) | Articulation creates contact | Fixed wrist/object; finger motion; hold and release |

The videos use actual captured Isaac Sim 5.1 poses and tactile arrays,
rendered separately with native RTX cameras. Slow playback is labeled in the
clip. The force frames preserve their exact pixel mapping and share one scale
within each clip; grey indicates unused cells.
Hand videos include a second native camera view of the contact so the small
gear remains legible while the overview shows all five fingertips.

## Choose a Newton example

Run from the pinned Newton Isaac Lab checkout. Prefix each command below with
`uv run --frozen python /path/to/dex-01-sim-assets/scripts/`.

| Task | Newton command | Layout and behavior |
|---|---|---|
| [Single fingertip](flat-pad.md#newton-single-fingertip) | `dex01_newton.py --mode sensor --indenter sphere --max-steps 240` | Curved DEX-01, 738 taxels; not the PhysX flat grid |
| [Hand load](hand.md#newton-hand-load) | `dex01_newton.py --mode hand --indenter flat --max-steps 1440 --report /tmp/newton-hand.json` | Five firmware frames, sequential known loads |
| [Stationary press](stationary-press.md#newton-stationary-press) | `dex01_newton.py --mode press --max-steps 1800 --report /tmp/newton-press.json` | Three joint-driven cycles against a fixed gear |

Use `--viz kit` with the `isaacsim` extra for live viewing. The shared
[model](../concepts/model.md) explains force distribution; the
[numerical settings](../reference/numerical-settings.md) distinguish solvers.
