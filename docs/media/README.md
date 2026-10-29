# Figure provenance

## Native camera figures

`dex01-demo.webp` and `loaded-contact.webp` were generated on October 13, 2026
from the audited PhysX hand demo. `render.json` records the base commit and
the exact source-file hashes, including the working-tree fixes used by this capture.

The physical state was captured by `scripts/docs/export_scene.py` in Isaac Sim
**5.1.0.0** after 240 steps with a local **0.1 kg cube**. Finger 2 carries
approximately **0.988 N** in this frame; all other fingers report zero. The
export preserves live PhysX body poses, all five visible fingertip meshes, and
the recorded firmware pixel mapping. The maximum taxel-to-visible-surface
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

The plotting utility rejects hand reports whose runtime, geometry, or taxel
hashes differ from the current checkout. `taxel-mapping.svg` uses the exterior
NPZ directly and can be generated with `--mapping-only`; `taxel-mapping.json`
records its input and output hashes. Regenerate each figure and its provenance
together when its inputs change.

## Example walkthroughs

The flat-pad, hand, and stationary-press clips were captured on October 13,
2026 from the corrected Isaac Sim 5.1.0.0 examples and rendered with native
Isaac Sim 6.0.1.0 RTX cameras. `examples.json` contains input/output hashes,
source hashes, timestamps, dimensions, and playback settings for each clip.

Every rendered pose comes from its corresponding captured simulation step.
The force frame is read at that same step and preserves the source pixel
mapping. The exporter preserves body scale, including scale inherited from
spawn wrappers. The renderer verifies position, orientation, scale, and every
mesh vertex and primitive dimensions against geometry recorded before export
(1 µm tolerance), and keeps
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
markers are hidden in the render so they do not obscure the gear and pad. The hand's five fingertip
meshes remain visible. Each clip uses a shared force scale; blue is zero,
cyan/yellow/red is increasing force, and grey denotes unoccupied pixels.
The hand gear is approximately 6.29 mm wide and the stationary-press gear is
8.25 mm wide. Their footprints occupy only part of the fingertip's unwrapped
taxel map. The regenerated images preserve these physical dimensions.

`audit.json` summarizes the stored October 13 regression results and report hashes.
The stored October 13 audit records 3,792 loaded taxel samples checked against
the gear triangle mesh. That external audit predates the committed capture
checker; use `scripts/docs/check_capture_geometry.py` with a fresh capture to
reproduce loaded-taxel containment, rather than treating the old report hash
as a new test result.
The press report checker additionally verifies broad face contact: at least 90% footprint coverage,
load in all four quadrants, near-parallel normals, and no separated underside
edge. Its close-up camera views the face obliquely in the gear's frame.

The repository-only recording instructions and workflow diagram live in
[DOCUMENTATION.md](../../DOCUMENTATION.md#record-a-simulation).

## Robotiq grasp, hold, and release

The Robotiq media was captured on October 25, 2026 using the repository's
`dex01_on_robotiq_2f85.py` example in Isaac Sim **5.1.0.0** and rendered with
native Isaac Sim **6.1.0.0** RTX cameras. The source gripper is the official
Isaac Sim 5.1 Robotiq asset. No SimReady Foundation profile validation is
claimed for this integration.

`robotiq.webm` and `robotiq.mp4` contain all **180 captured frames** at **30 fps**,
covering the six-second open/close/hold/open/release cycle. The block is a
free 0.1 kg, 10 × 30 × 18 mm rigid body with gravity and damping disabled.
Its recorded motion is preserved in the video.
Separate left/right images show the recorded 32 × 32 normal-force arrays
with one shared scale, and the inset is a synchronized native contact camera.
The heatmap aspect ratio comes from distances between captured taxel centers.
Each displayed cell spans one taxel pitch, giving a 14.4 × 33.6 mm grid extent
(width × length), rather than the full contact face including its border.

The exporter independently checks bilateral hold loads and block momentum
against normal and friction impulses before accepting the capture.
The renderer preserves all ten body poses and all twelve captured
meshes at every frame, with a 1 µm vertex-error limit, and never advances the
physics timeline. The before/contact/after stills are frames from that same
sequence. Captions label the actual phases.

The `robotiq` entry in `examples.json` records capture and renderer versions,
source-file and output hashes, the resolved gripper source, frame count,
camera inclusion, and maximum measured pose/vertex errors. The current
working-tree source hashes accompany the base commit, so uncommitted source
changes are represented explicitly. Large captured scenes remain outside Git.
