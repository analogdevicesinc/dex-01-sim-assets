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
| All-taxel sweep | Four conditions at all 740 taxels, analytic sphere depth, normal-load conservation, two-environment isolation |
| GPU regression | Real Warp distance kernel and MuJoCo-Warp contacts, transforms, rates, filtering, subset updates and reset |
| Capture / video | Same-tick force arrays and link poses, geometry scale/offset/topology, hashes, fixed color scale and playback timing |

Install pytest without changing the simulation pins and require the GPU module:

```bash
DEX01_REQUIRE_NEWTON_TESTS=1 .isaaclab/.venv/bin/python -m pytest tests/test_newton_backend.py -q
```

## Interpret the evidence

A passing report establishes only the fixtures it checks. The current press
checker verifies complete cycles, load, motion, fixed poses, mapping and release;
it does not enforce the PhysX fixture's planar coverage, alignment or underside
clearance bounds. The contact sweep checks a unit-scale sensing body. Counter
mesh scale support does not establish support for scaling the sensing body.

Keep run reports and review evidence with the tested source revision. Bundled
[media provenance](media.md) identifies the original capture inputs and layout.
Rerun physical fixtures after changing layout, geometry, solvers, sensing, timing
or camera configuration. A skipped GPU module is not a passing result.
