# Repository engineering instructions

Read [CONTRIBUTING.md](CONTRIBUTING.md) for contribution and licensing rules,
[DOCUMENTATION.md](DOCUMENTATION.md) for authoring, and
[docs/validation.md](docs/validation.md) for live simulation checks.

## Structure and contracts

- `source/dex01_sim_asset/` contains the sensor package and release metadata.
  `assets/` contains sensor geometry and mounted Tesollo USDs. `scripts/`
  contains builders, examples, verification, and documentation tools.
- SDK, firmware, and CAD sources inform the simulation assets. Change this
  repository unless the task explicitly includes an upstream modification.
- Use metres for geometry and newtons for force. Quaternions are `(w, x, y, z)`.
  Preserve scale through mounting, articulation, capture, and rendering.
- NPZ poses are `(N, 3, 4)`: columns 0–2 are right-handed orthonormal axes,
  column 3 is position, and column 2 is the outward normal. `taxel2pixel` is
  integer `(N, 2)`, zero-based `(row, column)`.
- DEX-01 occupies 740 unique cells in a 32 × 32 frame. Check the complete
  firmware address set, source/contact mapping order, and zero-filled unused
  cells. Preserve existing indices and align contact sites to the exterior.
- Preserve force conservation, counter-object filtering, and environment
  isolation. Materialize LFS assets before use and load NPZs with
  `allow_pickle=False`. Update generated assets and their hashes together.

## Validation

Use the toolchain pinned by `setup_env.sh`; CPU test dependencies are in
`.github/workflows/ci.yml`. Follow the existing code style and pre-commit hooks.

| Change | Required checks |
| --- | --- |
| Python behavior | Focused regression checks, then `python -m pytest tests -q` and `pre-commit run --all-files`. |
| Layout, CAD, USD, or mounting | Geometry/mapping tests and the all-cell live sweep with transformed probes and two environments in `docs/validation.md`. Check scale, surface alignment, normals, and hashes. |
| Force/contact handling | Localized-contact and known-load checks, including force balance, filtering, thumb coverage, and unloaded environments as applicable. |
| Motion, recording, or visualization | Relevant example/replay checks; inspect physical poses, dimensions, contact alignment, and displayed forces. |
| Version or package contents | Build the wheel from `source/dex01_sim_asset`; inspect version, taxel assets, and license files. |
| Published docs/configuration | `sphinx-build -b html -W --keep-going docs docs/_build/html`; inspect affected pages and figures. |
| Instructions or prose only | Applicable pre-commit hooks; no simulation run is needed unless behavior or assets change. |

Check command results and the tested revision. Verify asset-report input hashes
against the final files. Rerun hooks after formatter changes and repeat checks
when subsequent edits affect their results. Report any unavailable checks.

## Versioning and documentation

- `source/dex01_sim_asset/config/extension.toml` is the package/docs version
  source. Increment the patch version for behavioral or asset bugfixes,
  retaining the prerelease stage. Instructions-only changes need no product bump.
- Put concise change history, compatibility notes, and estimation limitations
  in `CHANGELOG.md`. Product docs and docstrings describe current behavior.
  Keep asset metadata declarative; do not add repair history or task narratives.
- Regenerate affected figures and provenance together. Preserve recorded data
  and historical release results accurately.

## Review and delivery

- Keep the PR focused, preserve unrelated work, and verify the current base.
  Follow `.github/PULL_REQUEST_TEMPLATE.md` and the DCO/license requirements
  in `CONTRIBUTING.md`; preserve root/package license copies and headers.
- Obtain independent review for sensor math, geometry, mapping, or recording
  changes. Include a plot or capture for observable changes and distinguish
  measured data from rendered geometry.
- Put review visuals and validation summaries in the PR or attached review
  artifacts. Do not commit per-task review notes, audit bundles, or run logs.
- Verify the pushed revision and actual CI results before reporting completion.
