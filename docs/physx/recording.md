# Record and replay PhysX

Activate the PhysX environment and run from `physx/`. For a live camera recording:

```bash
MPLBACKEND=Agg python scripts/dex01_on_tesollo_dg5f.py --headless --enable_cameras --record --video-dir /tmp/dex01-videos
```

Install FFmpeg and use Isaac Sim 5.1's validated RTX driver. Offscreen cameras
exclude Kit UI; the recorder adds sensor heatmaps from the corresponding arrays.
A streamed viewport and a recorded camera video are different outputs.

## Capture physical states for documentation

From `physx/` in the simulation environment:

```bash
python scripts/docs/capture_examples.py --example flat-pad --output /tmp/dex01-examples/flat-pad
python scripts/docs/capture_examples.py --example hand --output /tmp/dex01-examples/hand
python scripts/docs/capture_examples.py --example stationary-press --output /tmp/dex01-examples/stationary-press
```

The exporter records live body poses, same-tick tactile arrays and original
mesh geometry. Body scales include inherited spawn-wrapper scale; replay must
preserve them. Press reports are checked before capture acceptance.

In the separate RTX renderer environment, run from the repository root:

```bash
python ../scripts/docs/render_examples.py --capture /tmp/dex01-examples/hand --output /tmp/dex01-example-media/hand
```

The renderer keeps physics stopped and audits body transforms, scale and every
mesh vertex at each pose. Output includes MP4/WebM, phase captions, posters,
before/contact/after stills and provenance. Slow playback is labeled, and each
clip has one force scale. Keep the original capture and render manifests.

Bundled videos retain their original capture inputs and layout in the
[media provenance](https://github.com/analogdevicesinc/dex-01-sim-assets/blob/main/docs/media/README.md). Regenerate captures after changing
layouts or geometry before using them as current evidence.
