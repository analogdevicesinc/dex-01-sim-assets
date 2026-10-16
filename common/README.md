# Shared DEX-01 components

This directory contains `dex01-sim-common`: the one copy of taxel patterns,
public data buffers and visualization helpers. Each environment installer
installs this library locally; it does not choose or install a physics backend.
Shared geometry is under `../assets`. Backend-specific sensor implementations
and scene scripts belong to `../physx` and `../newton`.

See [shared concepts](../docs/concepts/model.md), [patterns](../docs/concepts/patterns.md)
and [the tactile data contract](../docs/reference/data.md).
