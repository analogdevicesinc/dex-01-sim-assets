# Newton environment

This track uses the pinned Isaac Lab 3.0 Early Access workspace, Python 3.12,
Newton and MuJoCo-Warp. Its installer, sensor package, scripts and tests live in
`newton/`. It installs no PhysX backend package and does not reuse the PhysX
Python environment.

Start with [installation](installation.md), run the [first simulation](quickstart.md),
then [bind](attachment.md) a sensor and [validate](validation.md) its behavior.
The [shared data reference](../reference/data.md) explains force arrays and mapping.

**Validation status:** the complete test suite with required CUDA integration
and the live load, all-taxel, press, camera and replay checks pass on the
recorded pinned stack. [Validation](validation.md) records the evidence and
required checks for changes. [Newton example videos](examples/index.md) replay
those verified physical states.

```{toctree}
:maxdepth: 2

installation
quickstart
examples/index
attachment
api
numerical-settings
validation
recording
media
troubleshooting
```
