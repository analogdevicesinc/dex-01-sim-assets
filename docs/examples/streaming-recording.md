# Streaming and recording

Use WebRTC for a live viewport on another computer, or record a video to
inspect and share later. Both paths show the geometry and tactile response.

```{figure} ../media/streaming-workflow.svg
:alt: The simulator sends its viewport and tactile overlay to a laptop by WebRTC; an offscreen camera and sensor arrays are composed into a recorded video.
:width: 100%

The live overlay travels with the viewport. The video recorder composes
camera pixels and corresponding tactile frames into the saved output.
```

## What a recording contains

<figure class="example-video">
<video controls playsinline loop preload="none" width="960" height="800"
       poster="../../_static/videos/hand.webp"
       aria-label="Recording preview: rendered hand motion, physical load in newtons, and firmware-mapped contact frames.">
  <source src="../../_static/videos/hand.webm" type="video/webm">
  <source src="../../_static/videos/hand.mp4" type="video/mp4">
  <track kind="captions" src="../../_static/videos/hand.vtt" srclang="en" label="Phases">
  <a href="../../_static/videos/hand.mp4">Download the example video</a>
</video>
<figcaption>Recording preview: rendered hand motion, physical load in newtons, and firmware-mapped contact frames.</figcaption>
</figure>

## Connect a viewing computer

Prerequisites: [installed environment](../getting-started/installation.md),
an RTX-capable NVIDIA GPU with the validated R580 driver for Isaac Sim 5.1.0,
and a viewing computer with the WebRTC client. Run all host commands from the
repository root.

For a remote headless host:

```bash
MPLBACKEND=Agg python scripts/dex01_on_tesollo_dg5f.py --livestream=2
```

Open the [Isaac Sim WebRTC Streaming Client](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/manual_livestream_clients.html)
on the viewing computer. Wait for `Setup complete...`, enter the host's reachable
IP address and connect. TCP 49100 and UDP 47998 must be reachable from the
viewing computer. `--livestream=2` is for a private network; NVIDIA's guide
covers public endpoints. Run one streaming simulator at a time on these ports.

The same overlay is enabled in `simple_example.py` and `verify_dex01_sim.py`.
Only environment 0 is displayed when running multiple environments. Add
`--no-tactile-ui` to hide it, `--local-plot` to the hand demo for the optional
separate desktop plot, or `--debug-vis` for 3D sampling markers. Stop with Ctrl+C.
Pure headless runs without streaming skip the UI. Offscreen camera recordings
exclude Kit UI; the existing recorder adds its own heatmaps to the video.

To record one cycle with both the rendered hand and taxel heatmaps, install
`ffmpeg` and run:

```bash
MPLBACKEND=Agg python scripts/dex01_on_tesollo_dg5f.py \
  --headless \
  --enable_cameras \
  --record \
  --video-dir ./videos
```

Expected output: a video under `videos/` containing the rendered hand and
tactile heatmaps. The default gear scene needs NVIDIA asset-server access.
