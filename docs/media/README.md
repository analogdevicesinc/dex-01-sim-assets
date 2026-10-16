# Figure provenance

## Native camera figures

`dex01-demo.webp` and `loaded-contact.webp` were generated on October 13, 2026
from the repository's PhysX hand demo at commit
`3818a8388c970ced7ad03539668d26c1a68e0add`.

The physical state was captured by `scripts/docs/export_scene.py` in Isaac Sim
**5.1.0.0** after 240 steps with a local **0.1 kg cube**. Finger 2 carries
approximately **0.988 N** in this frame; all other fingers report zero. The
export preserves live PhysX body poses, all five visible fingertip meshes, and
the original 738-taxel firmware mapping. The maximum taxel-to-visible-surface
error in this capture is approximately **0.017 µm**.

`scripts/docs/render_scene.py` opens the frozen capture in an isolated
**Isaac Sim 6.0.1.0 / Python 3.12** RTX renderer. It does not rerun physics or
change the recorded force arrays. Checks confirm that timeline time and every
captured rigid-body transform remain unchanged through both camera views.
The hand keeps its authored materials. Camera, lighting, and a neutral ground
material are presentation settings.

The overview and close-up are native simulator-camera images. The heatmap panel
is composed from the same captured tactile arrays with nearest-neighbor display,
unchanged row/column indices, a shared force scale, and grey for unused pixels.
No sensor geometry or contact data is invented or edited for the figures.

`render.json` records physics and renderer versions, source revision, input
hashes, camera settings, and output hashes. Reproduce the figures using the
[maintainer instructions](../../DOCUMENTATION.md#reproduce-the-figures). Keep
large captured USD states outside Git; only optimized documentation media is
committed. Regenerate media and provenance together when the scene changes.

## Force and mapping figures

`scripts/docs/plot_validation.py` generates the figures. `known-load.svg` uses
an independently checked hand report; `figures.json` records its report,
checker, input-pattern, and output hashes. The trace includes the complete
impact and settled interval.

`taxel-mapping.svg` uses the exterior NPZ layout directly and can be generated
with `--mapping-only`. `taxel-mapping.json` records its input and output hashes.
Regenerate each figure and its provenance together when its inputs change.

## Example walkthroughs

The bundled hand and stationary-press media still need regeneration with the
corrected scale-preserving exporter. Their existing renders enlarged the gear
while retaining force frames from the smaller simulated object. The capture
and render checks described below apply to the corrected pipeline; replace the
affected media and their provenance together before removing this notice.

The flat-pad, hand, and stationary-press clips were captured on October 13,
2026 from the unchanged Isaac Sim 5.1.0.0 examples and rendered with native
Isaac Sim 6.0.1.0 RTX cameras. `examples.json` contains input/output hashes,
source hashes, timestamps, dimensions, and playback settings for each clip.

Every rendered pose comes from its corresponding captured simulation step.
The force frame is read at that same step and preserves the source pixel
mapping. The exporter preserves body scale, including scale inherited from
spawn wrappers. The renderer verifies position, orientation, scale, and every
mesh vertex against geometry recorded before export (1 µm tolerance), and keeps
its physical timeline stopped. No poses or force arrays are interpolated.
The press report independently checks wrist/object invariance, joint motion,
contact, filtering, and release before the capture is accepted.

The flat-pad clip covers one drop/settle sequence at 3× slower playback.
The hand clip covers fingers 2–5 in sequence at simulation-time speed.
The press clip covers a complete 2.5-second physical cycle at 2× slower
playback. The clock in each video reports physical simulation time, not
video time. These clips have no audio; captions name the phases, and the
pages provide written descriptions and static before/contact/after states.

Native images use the source asset materials with adjusted camera/light
presentation. Hand clips include a synchronized native camera close-up of the
gear at its actual physical dimensions. The flat-pad capture's debug sample
markers are hidden in the
render so they do not obscure the gear and pad. The hand's five fingertip
meshes remain visible. Each clip uses a shared force scale; blue is zero,
cyan/yellow/red is increasing force, and grey denotes unoccupied pixels.
The hand gear is approximately 6.29 mm wide and the stationary-press gear is
8.25 mm wide. Their footprints occupy only part of the fingertip's unwrapped
taxel map. Regenerated images must preserve these physical dimensions.

The repository-only recording instructions and workflow diagram live in
[DOCUMENTATION.md](../../DOCUMENTATION.md#record-a-simulation).
