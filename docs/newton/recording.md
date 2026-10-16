# Record and replay Newton

Live recording and audited replay are separate outputs. Live recording uses
Newton's camera renderer during simulation. Replay renders previously captured
geometry and force arrays with the physical timeline stopped.

## Record a live run

Install the video extra with `newton/setup_env.sh --video`, activate the pinned
environment, and run from `newton/`:

```bash
PYTHONPATH="$PWD/.isaaclab/source/isaaclab" python scripts/dex01_newton.py --mode press --viz kit --max-steps 1800 --record /tmp/newton-press.mp4 --report /tmp/newton-press.json
python scripts/dex01_newton.py --check-report /tmp/newton-press.json
```

For `run.mp4`, outputs are `run.recording.npz` and `run.recording.json` beside
the video. A simulation report such as `run.json` stays separate.
The manifest records sampled simulation times, source and package provenance,
array/video hashes and one force scale for the entire clip. Samples are taken
every eight ticks (30 frames per simulation second) after camera warm-up.
Force frames keep their exact firmware mapping; debug markers are rejected
for recordings.

Live camera correctness must be verified on the EA GPU runtime. The existence
of a video and CPU encoder tests alone do not validate the camera or contacts.

## Export an audited capture

From `newton/`, use a new destination for each export:

```bash
python scripts/dex01_newton.py --mode hand --indenter flat --max-steps 1440 --report /tmp/newton-hand.json --capture /tmp/newton-hand-capture
python scripts/dex01_newton.py --mode press --max-steps 1800 --report /tmp/newton-press.json --capture /tmp/newton-press-capture
```

Capture exports environment-zero link poses, matching firmware/taxel forces,
pre-export body scales, mesh offsets and topology. The full load or press report
must pass before output is published. The press export contains its third cycle.
The exported USD is reloaded and audited before accepting the directory.

## Render without advancing physics

Use a separate RTX renderer environment. Run from `newton/` with the renderer environment activated:

```bash
python ../scripts/docs/render_examples.py --capture /tmp/newton-hand-capture --output /tmp/newton-hand-render
```

The shared renderer verifies required file hashes, sampled tick spacing,
XYZW rotations, exact taxel/frame agreement, geometry and fixed wrist/object
poses for pressing. It checks each mesh vertex at every replayed pose and uses
one force scale. It never integrates a second physical trajectory or interpolates
forces. Keep captured physics and rendered-media manifests together.

The existing PhysX clips and validation numbers cannot be reused as Newton
media evidence. Publish new Newton figures only after the
[GPU and fixture checks](validation.md) pass and the rendering is inspected.
