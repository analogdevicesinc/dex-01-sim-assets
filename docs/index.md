# DEX-01 tactile simulation

Simulate normal contact loads on the DEX-01 fingertip, with 738 surface samples
mapped into its 32 × 32 firmware frame. The physical geometry and data format
are shared; each simulation environment has its own installation and workflow.

## Choose your environment

<div class="environment-cards">
<a class="environment-card" href="physx/"><strong>PhysX</strong><span>Isaac Sim 5.1 · Python 3.11</span><span>Established load and contact fixtures, a flat-grid demo, Tesollo hand and WebRTC examples.</span><span>Install and follow the PhysX workflow →</span></a>
<a class="environment-card" href="newton/"><strong>Newton</strong><span>Isaac Lab 3.0 Early Access · Python 3.12</span><span>Native SDF contacts with MuJoCo-Warp, validated hand and press fixtures, and recorded Newton examples.</span><span>Install and follow the Newton workflow →</span></a>
</div>

Choose a track before installing. Both tracks live in the same repository,
but use separate Python environments and backend packages. See
[compatibility](reference/compatibility.md) for version and validation status.

## Understand the sensor

[Contact model](concepts/model.md), [geometry](concepts/geometry.md),
[patterns](concepts/patterns.md) and [tactile data](reference/data.md)
apply to both environments. A force frame is an unwrapped sampling of the
fingertip surface, not a camera image. Summing the taxel forces gives normal
load in newtons; hardware calibration, shear and material hysteresis are outside
this model.

Browse [examples by environment](examples/index.md), or read
[the repository layout](reference/repository.md) before contributing.

```{toctree}
:hidden:
:maxdepth: 2
:caption: Simulation environments

physx/index
newton/index
```

```{toctree}
:hidden:
:maxdepth: 1
:caption: Shared sensor concepts

concepts/model
concepts/geometry
concepts/patterns
concepts/limitations
```

```{toctree}
:hidden:
:maxdepth: 1
:caption: Shared reference and project

reference/data
reference/api
reference/compatibility
reference/repository
examples/index
validation
troubleshooting
contributing
licensing
citation
```
