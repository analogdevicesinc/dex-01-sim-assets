# Changelog

## 0.1.1-a.1 — 2026-10-16

- Correct the DEX-01 layout from 738 to 740 taxels by adding firmware cells
  `(8, 0)` and `(8, 31)`, preserving existing poses and indices.
- The added positions are estimated from neighboring samples and projected
  onto the exterior mesh using CAD normals.
