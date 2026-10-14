# DEX-01 Tactile Sensor

Simulate normal contact forces on a DEX-01 fingertip in NVIDIA Isaac Lab,
with 738 contact samples mapped into the sensor's 32 × 32 firmware frame.

```{figure} media/dex01-demo.webp
:alt: PhysX scene showing all five dark DEX-01 fingertips mounted on a Tesollo hand, with finger 2 loaded and its tactile frame active.
:width: 100%

A 0.1 kg cube contacts finger 2 on the Tesollo DG-5F hand. Each panel reports
physical normal force in newtons using the original firmware pixel mapping.
```

Browse [the visual examples](examples/index.md) to see contact and articulation
before setting up a scene.

Start with [installation](getting-started/installation.md), then
[run your first simulation](getting-started/quickstart.md).
For integration, see [attaching the sensor](guides/attachment.md) and the
[API reference](reference/api.md).

## From contact to physical force

Each tactile frame reports **normal force in newtons**. A 0.1 kg fixture under
9.81 m/s² gravity should produce 0.981 N; the [validation workflow](validation.md)
checks this against independently filtered PhysX forces and momentum balance.
See [how contact is modeled](concepts/model.md) and its
[supported model behavior](concepts/limitations.md).

```{toctree}
:maxdepth: 1
:caption: Get started

getting-started/installation
getting-started/quickstart
```

```{toctree}
:maxdepth: 1
:caption: Examples

examples/index
examples/flat-pad
examples/hand
examples/stationary-press
```

```{toctree}
:maxdepth: 1
:caption: Guides

guides/attachment
guides/read-data
guides/patterns
```

```{toctree}
:maxdepth: 1
:caption: Concepts and reference

concepts/model
concepts/geometry
concepts/limitations
reference/api
reference/numerical-settings
reference/compatibility
```

```{toctree}
:maxdepth: 1
:caption: Validation and support

validation
troubleshooting
contributing
licensing
citation
```
