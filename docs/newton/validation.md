# Validate Newton

Run from `newton/` with its pinned environment activated. Keep every report
and log with its exact commit and package versions.

```bash
python scripts/newton_preflight.py --isaaclab .isaaclab --report /tmp/newton-preflight.json
python scripts/dex01_newton.py --mode hand --indenter flat --max-steps 1440 --report /tmp/newton-hand.json
python scripts/dex01_newton.py --check-report /tmp/newton-hand.json
python scripts/dex01_newton.py --mode press --max-steps 1800 --report /tmp/newton-press.json
python scripts/dex01_newton.py --check-report /tmp/newton-press.json
python scripts/verify_dex01_newton.py --sweep-all --report /tmp/newton-sweep.json
```

| Check | Required evidence |
|---|---|
| Flat load | Every requested tick; 0.981 N settling, unloaded fingers, exact frame mapping, filtered normal force and momentum balance |
| Press | At least three complete cycles; all additional cycles checked, fixed wrist/object, joint and fingertip travel, sustained contact, return and release |
| All-taxel sweep | Four conditions at all 738 taxels, analytic sphere depth, normal-load conservation, two-environment isolation |
| GPU regression | Real Warp distance kernel and MuJoCo-Warp contacts, transforms, rates, filtering, subset updates and reset |
| Capture / video | Same-tick force arrays and link poses, geometry scale/offset/topology, hashes, fixed color scale and playback timing |

Install pytest without changing the simulation pins and require the GPU module:

```bash
DEX01_REQUIRE_NEWTON_TESTS=1 .isaaclab/.venv/bin/python -m pytest tests/test_newton_backend.py -q
```

## Verified Newton fixtures

On October 16, 2026, Linux x86_64 with an NVIDIA RTX 4500 Ada validated the
pinned Isaac Lab revision, Newton 1.5.2, Warp 1.16.0, MuJoCo/MuJoCo-Warp 3.11.0,
Torch 2.11.0 and Isaac Sim 6.1.0 for viewing/recording.

| Check | Result |
|---|---|
| Complete suite with required CUDA module | 238 passed, no skips at the recorded implementation state |
| Flat load, fingers 2–5 | Mean settling 0.980842, 0.980972, 0.980947, 0.981031 N; unloaded fingers zero |
| Contact sweep | 2,952 cases: four conditions at every taxel, analytic depth, normal-load conservation and two-environment isolation passed |
| Fixed-wrist pressing | All 1,800 requested ticks and three full cycles, force/frame agreement, real motion and release passed |
| Multi-environment live camera | Two-environment recording completed with separate physics report and video/array manifests; 172 frames at 30 fps |
| Audited hand replay | All 180 sampled frames, fixed scale, preserved body/mesh geometry and stopped physical timeline passed |
| Audited press replay | All 75 third-cycle frames; maximum mesh-vertex deviation below 1 µm, fixed wrist/object and stopped physical timeline passed |

The test suite includes negative checks for missing ticks, incorrect target
fingers, truncated capture ends, wrong phase labels, force mapping, collisions
between output paths and uv overrides redirecting installations. The live
press load is a configured simulation result, not a hardware force limit or
measured DG-5F specification.

[Media provenance](../media/newton-examples.json) retains the source hashes,
full capture/render manifests and report hashes for these exact runs. The source
commit field names the branch base while source hashes identify the implementation
used before commit; reproduce from the matching reviewed source state.
Re-run validation for changes to geometry, solvers, sensing, timing or camera
configuration. A skipped GPU module is not a passing result.
