## Description

Replace this with a summary of the change and the motivation behind it. Open an
issue first for substantial changes, as described in
[CONTRIBUTING.md](../blob/main/CONTRIBUTING.md).

## Type

- [ ] Bug fix
- [ ] New feature
- [ ] Breaking change (changes an existing interface, asset layout or result)
- [ ] Documentation

## Verification

Describe how the change was verified, including the commands run. Changes to
sensing or geometry must also pass the live checks in the README.

## Checklist

- [ ] Every commit carries a `Signed-off-by:` line (`git commit -s`), agreeing to
      the [Developer Certificate of Origin 1.1](https://developercertificate.org/)
- [ ] `pre-commit run --all-files` and `python -m pytest tests -q` pass locally
- [ ] Third-party copyright and license notices in modified files are preserved
- [ ] `LICENSE`, `NOTICE` and `LICENSE_Tesollo` copies under
      `source/dex01_sim_asset/` still match the root files, if either changed
- [ ] Documentation (README, docstrings) is updated where behavior changed
