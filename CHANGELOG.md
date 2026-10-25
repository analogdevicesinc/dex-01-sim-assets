# Changelog

## 0.1.2-a.1 — Unreleased

- Group examples by embodiment, with scenario walkthroughs beneath each.
  Separate running and reading the examples from asset integration and detailed
  validation instructions.
- Keep confidential native design inputs outside the repository. Public asset
  rebuilding uses only approved simulation artifacts, with source-file checks
  enforced by pre-commit and CI.

- Add a unified rectangular DEX-01 finger and 32 × 32 taxel layout, using the
  humanoid fingertip's physical row/column directions.
- Integrate the fingers with the official Isaac Sim Robotiq 2F-85 articulation.
  The custom adapters use a 35.9° closing limit to retain at least 0.5 mm of
  separation under the authored parallel-jaw kinematics.
- Resolve the gripper through the configured Isaac asset root or an explicit
  local/cloud source, including a standard USD asset-root expression override.
- Add native-camera grasp/hold/release documentation capture with both tactile
  images and recorded geometry checks.
- Use a free rigid block for the grasp example and verify its momentum against
  normal and friction impulses. The previous fixed fixture masked differences
  in sampled compliant contacts; bilateral hold-force checks now reject that
  imbalance rather than only checking redistribution onto taxels.

## 0.1.1-a.1 — 2026-10-16

- Correct the DEX-01 layout from 738 to 740 taxels by adding firmware cells
  `(8, 0)` and `(8, 31)`, preserving existing poses and indices.
- The added positions are estimated from neighboring samples and projected
  onto the exterior mesh using CAD normals.
