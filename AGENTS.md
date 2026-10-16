# Repository engineering instructions

Read [CONTRIBUTING.md](CONTRIBUTING.md) for contribution and licensing rules,
[DOCUMENTATION.md](DOCUMENTATION.md) for authoring, and
[docs/validation.md](docs/validation.md) for live simulation checks.

## Structure and contracts

- `physx/` and `newton/` contain independent backend packages, installers,
  examples, and tests. `common/` owns shared patterns and data contracts.
  `assets/` contains sensor geometry and mounted Tesollo USDs. `scripts/`
  contains shared asset builders and documentation tools.
- SDK, firmware, and CAD sources inform the simulation assets. Change this
  repository unless the task explicitly includes an upstream modification.
- Follow the coordinate conventions in [geometry documentation](docs/concepts/geometry.md).
  Preserve physical scale through mounting, articulation, capture, and rendering.
- Use [reference.json](assets/sensors/dex01/reference.json) for asset paths,
  mounting transforms, and hashes. The referenced NPZs define the layout and
  pixel mapping; do not duplicate their counts or frame dimensions here.
- Follow the schema and validation in
  [taxel_patterns.py](common/source/dex01_sim_common/dex01_sim_common/taxel_patterns/taxel_patterns.py).
  Check firmware addresses against [asset tests](common/tests/test_dex01_assets.py).
  Preserve mapping order, zero-filled unused cells, and exterior alignment.
- Preserve force conservation, counter-object filtering, and environment
  isolation. Materialize LFS assets before use and update generated assets
  and their hashes together.

## Validation

Use the selected backend installer ([PhysX](physx/setup_env.sh), [Newton](newton/setup_env.sh)) for toolchain pins and
[CI](.github/workflows/ci.yml) for CPU test dependencies. Follow the code style
and hooks in [.pre-commit-config.yaml](.pre-commit-config.yaml). Use the commands
in [CONTRIBUTING.md](CONTRIBUTING.md) and [docs/validation.md](docs/validation.md).

| Change | Required checks |
| --- | --- |
| Python behavior | Focused regression checks, then the full tests and hooks required by `CONTRIBUTING.md`. |
| Layout, CAD, USD, or mounting | Geometry/mapping tests and the live sweep documented in `docs/validation.md`. Check scale, surface alignment, normals, and hashes. |
| Force/contact handling | Localized-contact and known-load checks, including force balance, filtering, thumb coverage, and unloaded environments as applicable. |
| Motion, recording, or visualization | Relevant example/replay checks; inspect physical poses, dimensions, contact alignment, and displayed forces. |
| Version or package contents | Build the wheel from the affected packages; inspect version, taxel assets, and license files. |
| Published docs/configuration | Build using `docs/Makefile` and `DOCUMENTATION.md`; inspect affected pages and figures. |
| Instructions or prose only | Applicable pre-commit hooks; no simulation run is needed unless behavior or assets change. |

Check command results and the tested revision. Verify asset-report input hashes
against the final files. Rerun hooks after formatter changes and repeat checks
when subsequent edits affect their results. Report any unavailable checks.

## Versioning and documentation

- [extension.toml](physx/source/dex01_sim_asset/config/extension.toml) is the package/docs
  version source. Increment the patch version for behavioral or asset bugfixes,
  retaining the prerelease stage. Instructions-only changes need no product bump.
- Product documentation and docstrings explain current behavior, usage,
  configuration, requirements, and limitations that affect users. After a fix,
  describe the corrected product as its normal state.
- Put release changes, bug history, compatibility notes, and change-specific
  estimation details in [CHANGELOG.md](CHANGELOG.md). Do not add before/after
  narratives, repair explanations, or repeated caveats to user guides.
- Keep task notes, agent reviews, audit bundles, run logs, and transient test
  counts in the PR description, comments, or attachments, not in product
  documentation or committed files.
- Asset metadata contains data needed to identify, load, transform, or validate
  assets. Generated media provenance records actual inputs and outputs. Neither
  is a place for repair history, review verdicts, or a task diary.
- Reference authoritative files for changeable specifications, toolchain pins,
  schemas, and commands instead of duplicating them in these instructions.
- Keep documentation proportional to the change: update affected facts and
  examples without adding sections that serve only to explain the fix.
  Regenerate affected figures and provenance together; preserve recorded data
  and historical release results accurately.

## Review and delivery

- Keep the PR focused, preserve unrelated work, and verify the current base.
  Follow `.github/PULL_REQUEST_TEMPLATE.md` and the DCO/license requirements
  in `CONTRIBUTING.md`; preserve root/package license copies and headers.
- Obtain independent review for sensor math, geometry, mapping, or recording
  changes. Include a plot or capture for observable changes and distinguish
  measured data from rendered geometry.
- Put review visuals and validation summaries in the PR description, review
  comments, or attachments. Here, "in the PR" means the discussion and attached
  evidence, not files added to the PR's repository diff. Commit a visual only
  when it is part of the product documentation or another maintained asset.
  Do not commit per-task review notes, audit bundles, or run logs.
- Verify the pushed revision and actual CI results before reporting completion.
