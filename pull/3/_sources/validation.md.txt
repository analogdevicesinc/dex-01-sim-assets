# Validation

These commands launch Isaac Sim unless they only check an existing report.
Complete [installation](getting-started/installation.md) first.

In the activated setup environment:

```bash
python -m pip install scipy rtree
MPLBACKEND=Agg python scripts/dex01_on_tesollo_dg5f.py \
  --headless \
  --flat-indenter \
  --max-steps 1200 \
  --verify-thumb \
  --verification-report /tmp/dex01-hand.json
python scripts/verify_dex01_sim.py --check-hand-report /tmp/dex01-hand.json
```

The 0.1 kg flat load should settle near **0.981 N** on each pressed finger,
with zero on unloaded fingers. The checker verifies forces, counter-object
filtering and momentum balance, then reports the separate thumb probe result.
The gear can rock and is not a calibration load.

For the complete 738-contact sweep:

```bash
python scripts/verify_dex01_sim.py \
  --headless \
  --sweep-all \
  --transformed-probe \
  --wrapped-probe \
  --num-envs 2 \
  --report /tmp/dex01-contact.json
```

The wrapper case verifies that contact forces are filtered against the resolved
rigid body. Add `--sensor-scale 1.5` to also check a scaled sensor and its taxel
layout. The supervising process rejects a missing report even if Isaac Sim
exits with code zero. Repository scripts explicitly load this checkout's
sensor implementation, and fresh reports record its source hash.

For articulated motion checks, see [stationary pressing](examples/stationary-press.md).

```{figure} media/known-load.svg
:alt: Four force traces for fingers 2 through 5 settling around the dashed 0.981 newton fixture-weight reference.
:width: 100%

The full load cycles include the initial impact transient. The lower panels
show the final 20% used for settling checks. The dashed line is `m × g`.
```

## Verified fixture results

On October 13, 2026, the corrected runtime was checked on Linux with
Isaac Sim 5.1.0 and NVIDIA RTX 4500 Ada hardware. Exact source-file and report
hashes are recorded in `media/audit.json`; the media manifests also identify
the source snapshots used for each capture.

| Check | Result |
|---|---|
| Repository geometry, mapping, force, visualization, setup and validation tests | 160 passed |
| Flat-load report checker, fingers 2–5 | 0.980522, 0.980521, 0.980793, 0.981630 N; unloaded fingers zero |
| Thumb localization and counter-object filtering | Passed |
| All 738 taxels, transformed wrapper probes, two environments | Passed at sensor scales 1 and 1.5; 1,476 cases |
| Fixed-wrist planar press and release checker | Passed; median checked hold load 0.613 N |
| Planar gear-face contact | 48 of 50 footprint taxels loaded; all four quadrants; maximum normal mismatch 0.0333° |
| Native documentation video geometry | All 225 frames checked against captured geometry |
| Loaded taxels against the gear triangle mesh | All 3,792 loaded samples lie inside the gear |

The hand report checker also checks filtered PhysX force agreement and
momentum balance. These results establish the tested physical-force fixtures and trajectories.
Check new articulated poses for clearance as part of task validation.
The press endpoint is measured only after its attached sensor body settles.
Only coplanar samples define its normal. The fixture begins with clearance
and seeks contact while joint targets correct both position and orientation.
The press reports the maximum underside gap relative to the moving pad plane,
and its checker requires that reported gap to stay within 2 µm during hold. Its force
threshold is not a calibrated force-tracking controller. Headless runs verify
physics using simulation time; GUI runs additionally check observed timing.

## Inspect the assembled geometry

```{figure} media/loaded-contact.webp
:alt: Native simulator-camera close-up showing the cube contacting the mounted DEX-01 fingertip and the neighboring finger pads.
:width: 100%

The captured flat-load fixture retains the verified live body poses. Native
RTX rendering preserves those poses and the recorded tactile frames.
```

On a host with the validated display/camera driver, generate several views:

```bash
MPLBACKEND=Agg python scripts/dex01_on_tesollo_dg5f.py \
  --headless \
  --enable_cameras \
  --max-steps 1 \
  --inspection-dir /tmp/dex01-views
```

Inspect the output images for five fully mounted fingertips, clearance around
the rigid finger backing, and alignment of the contact sample markers with the
exterior. For any new trajectory, check contact and release as well as the
visual fit. A screenshot complements the numerical checks; it does not replace
a force or invariance report.

Re-run the commands above when geometry or sensing changes. Keep the JSON
reports and simulator logs with review evidence. Documentation build checks
validate the site, while these commands validate the simulation.
