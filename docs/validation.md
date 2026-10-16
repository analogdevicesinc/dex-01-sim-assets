# Validation

This page separates **PhysX results** from **Newton required checks**.
The existing results table records PhysX only. Newton commands and the current
validation status appear in their own section below.

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

For the complete 740-contact sweep:

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

The supplied 740-taxel layout passes the following simulation fixtures:

| Check | Result |
|---|---|
| Flat-load fixture, fingers 2–5 | 0.9805–0.9816 N against the 0.981 N reference; unloaded fingers zero |
| Thumb localization and counter-object filtering | Passed |
| All 740 taxels, transformed probes, two environments | Passed |

The hand report checker also checks filtered PhysX force agreement and
momentum balance. Check new articulated poses for clearance, contact, and
release using the stationary-press example.
The press endpoint is measured after its attached sensor body settles. The
fixture uses coplanar samples to align the pad and checks its underside gap
during hold. Its force threshold is not a calibrated force-tracking controller.

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

## Newton validation

Use the pinned Newton Lab checkout and frozen dependencies. Keep every report
and log with its exact commit and package versions.

```bash
uv run --frozen python /path/to/dex-01-sim-assets/scripts/newton_preflight.py --isaaclab . --report /tmp/newton-preflight.json
uv run --frozen python /path/to/dex-01-sim-assets/scripts/dex01_newton.py --mode hand --indenter flat --max-steps 1440 --report /tmp/newton-hand.json
uv run --frozen python /path/to/dex-01-sim-assets/scripts/dex01_newton.py --check-report /tmp/newton-hand.json
uv run --frozen python /path/to/dex-01-sim-assets/scripts/dex01_newton.py --mode press --max-steps 1800 --report /tmp/newton-press.json
uv run --frozen python /path/to/dex-01-sim-assets/scripts/dex01_newton.py --check-report /tmp/newton-press.json
uv run --frozen python /path/to/dex-01-sim-assets/scripts/verify_dex01_newton.py --sweep-all --report /tmp/newton-sweep.json
```

| Check | Required evidence |
|---|---|
| Flat load | Every tick; 0.981 N settling, unloaded fingers, exact frame mapping, filtered normal force and momentum balance |
| Press | Three complete cycles, fixed wrist/object, joint and fingertip travel, sustained contact, return and release |
| All-taxel sweep | Four conditions at all 738 taxels, analytic sphere depth, normal-load conservation, two-environment isolation |
| GPU regression | Real Warp distance kernel and MuJoCo-Warp contacts, transforms, rates, filtering, subset updates and reset |
| Capture / video | Same-tick force arrays and link poses, geometry scale/offset/topology, hashes, fixed color scale and playback timing |

Install pytest without changing the simulation pins and require the GPU module:

```bash
DEX01_REQUIRE_NEWTON_TESTS=1 /path/to/IsaacLab-Newton/.venv/bin/python -m pytest /path/to/dex-01-sim-assets/tests/test_newton_backend.py -q
```

**Current status:** CPU report, geometry replay, capture corruption and real FFmpeg
recording tests pass; CUDA physics and camera checks remain pending because the
validation host lacks the pinned runtime and working driver. A skipped GPU module
is not a passing result. Do not copy the PhysX force numbers or media into a Newton
validation claim. See [capture/replay](getting-started/newton.md#capture-and-render-newton-evidence)
for producing separate Newton artifacts after the live report checks pass.
