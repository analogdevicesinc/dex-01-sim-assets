# Contributing

Open an issue to discuss a substantial change before submitting a pull request.
Keep changes focused and describe how they were verified.

## License and sign-off

ADI-authored material and new contributions are licensed under Apache-2.0.
Existing Isaac Lab and Tesollo material retains its BSD-3-Clause terms as
described in [NOTICE](https://github.com/analogdevicesinc/dex-01-sim-assets/blob/main/NOTICE).

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

Install Git LFS and fetch shared assets before running geometry tests. Choose
`physx/setup_env.sh` or `newton/setup_env.sh` for live simulation work; their
Python environments and backend packages are separate.

For CPU checks, use Python 3.11 or 3.12 with NumPy, Torch, pytest, Matplotlib,
Trimesh, SciPy, Rtree, Pillow, imageio-ffmpeg and usd-core 26.8. From the root:

```bash
pre-commit run --all-files
python -m pytest -q
```

Pytest discovers shared checks under `tests/` and `common/tests/`, plus backend
checks under `physx/tests/` and `newton/tests/`. Backend CPU suites can also be
selected explicitly. CI runs the complete CPU suite on Python 3.11 and 3.12.

Geometry or sensing changes must pass the selected backend's live validation:
[PhysX](docs/physx/validation.md) or [Newton](docs/newton/validation.md).
Require the Newton GPU suite with `DEX01_REQUIRE_NEWTON_TESTS=1`; absent runtime
dependencies or CUDA then fail instead of skipping. A passing CPU suite does
not establish GPU contact or camera correctness. Keep reports and logs with
the exact source and package versions.

## Licensing documents

[LICENSE](https://github.com/analogdevicesinc/dex-01-sim-assets/blob/main/LICENSE) and [NOTICE](https://github.com/analogdevicesinc/dex-01-sim-assets/blob/main/NOTICE) at the repository root are the canonical
license and attribution files. Preserve existing third-party copyright notices
and license terms when modifying adapted material. Keep [LICENSE_Tesollo](https://github.com/analogdevicesinc/dex-01-sim-assets/blob/main/LICENSE_Tesollo)
exactly as supplied by Tesollo.

The three license files are copied into the common, PhysX and Newton package
roots so that every wheel and source distribution includes them. When updating the
root files, update those copies in the same change. CI checks that they match.

Publish the final approved SBOM and licensing collateral as release attachments.
Keep generated documents unchanged; do not substitute dependency listings for
the release SBOM.

## Documentation changes

Follow the [documentation maintainer guide](https://github.com/analogdevicesinc/dex-01-sim-assets/blob/main/DOCUMENTATION.md).
Build with `make -C docs html` in the documentation environment before review.
