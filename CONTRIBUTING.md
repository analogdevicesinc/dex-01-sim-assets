# Contributing

Open an issue to discuss a substantial change before submitting a pull request.
Keep changes focused and describe how they were verified.

## License and sign-off

ADI-authored material and new contributions are licensed under Apache-2.0.
Existing Isaac Lab and Tesollo material retains its BSD-3-Clause terms as
described in [NOTICE](NOTICE).

Every contribution must include a Developer Certificate of Origin sign-off.
By signing off, you agree to the [Developer Certificate of Origin 1.1](https://developercertificate.org/)
and certify that you have the right to submit the contribution under its license.
Add the sign-off to each commit using:

```bash
git commit -s
```

This adds `Signed-off-by: Your Name <your-email>` to the commit message.
Maintainers must check sign-offs before accepting contributions.

## Development checks

Install Git LFS and fetch the assets before running the tests or demos.
After running `setup_env.sh` and activating its environment, install the
additional test tools and enable the hooks:

```bash
python -m pip install pytest pre-commit scipy rtree
pre-commit install
```

```bash
pre-commit run --all-files
python -m pytest tests -q
```

The tests cover taxel mapping, rigid transforms, force calculation, visualization,
exterior geometry and mounting without launching Isaac Sim. Changes to sensing
or geometry must also pass the live checks in [README.md](README.md).

Newton uses the separate pinned Python 3.12 environment described in the README.
CPU CI runs on Python 3.11 and 3.12; the Newton integration module requires CUDA.
Run it with `DEX01_REQUIRE_NEWTON_TESTS=1` so missing dependencies or GPU access
fail instead of skipping. Run the live contact sweep and load/press verification
when changing Newton sensing, solver versions or collider geometry.

## Licensing documents

[LICENSE](LICENSE) and [NOTICE](NOTICE) at the repository root are the canonical
license and attribution files. Preserve existing third-party copyright notices
and license terms when modifying adapted material. Keep [LICENSE_Tesollo](LICENSE_Tesollo)
exactly as supplied by Tesollo.

The three license files are also copied into `source/dex01_sim_asset/` so that
both Python wheels and source distributions include them. When updating the
root files, update those copies in the same change. CI checks that they match.

Publish the final approved SBOM and licensing collateral as release attachments.
Keep generated documents unchanged; do not substitute dependency listings for
the release SBOM.
