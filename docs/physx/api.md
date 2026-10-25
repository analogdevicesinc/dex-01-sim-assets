# PhysX sensor API

Import simulator modules only after Isaac Lab's `AppLauncher` starts Isaac Sim.
This reference describes the public integration surface; examples run in a
simulation environment, while the documentation build never imports it.

## TactileSensor

`dex01_sim_asset.TactileSensor` extends Isaac Lab's `SensorBase`.
Construct it with `TactileSensorCfg` through your scene configuration.

| Interface | Result |
|---|---|
| `data` | Updated `TactileSensorData` buffers |
| `num_taxels` | Number of sample points; 740 for the supplied DEX-01 layout |
| `taxel2pixel` | Integer `(row, column)` pairs in taxel order |
| `get_tactile_image()` | Force frame `(environments, height, width)`; DEX-01 uses 32 × 32 |

See [reading data](../reference/data.md) for an executable usage snippet.

## TactileSensorCfg

Inherits Isaac Lab's `SensorBaseCfg`, including `prim_path`, `update_period`,
and `debug_vis`. The sensor's leaf prim name must be literal, not a regex.

| Field | Meaning |
|---|---|
| `mesh_prim_path` | Required counter-object path; its collision mesh must use SDF |
| `pattern_cfg` | Required taxel pattern configuration |
| `offset.pos` | Sensor-frame translation relative to the parent, in metres |
| `offset.rot` | Sensor-frame rotation as `(w, x, y, z)`; identity by default |
| `visualizer_cfg` | Optional debug marker configuration |

```{literalinclude} ../../physx/source/dex01_sim_asset/dex01_sim_asset/tactile_sensor_cfg.py
:language: python
:start-at:     mesh_prim_path:
:end-before:     visualizer_cfg:
```

## Shared data contract

See [tactile buffers, shapes and units](../reference/data.md). The PhysX sensor
returns the shared `dex01_sim_common.TactileSensorData` object.

## Lifecycle

Configure sensors on the scene before `sim.reset()`. Then advance the scene
with `scene.write_data_to_sim()`, `sim.step()`, and `scene.update(sim_dt)` before
reading frames. The sensor’s `data` property updates outdated buffers; its
`update_period` comes from Isaac Lab’s base sensor configuration.

`normal_forces` is nonnegative. `depths` is negative only for penetrating
samples when the configured object contributes contact force; otherwise it is
zero. The sum is the normal projection of the configured object’s filtered
PhysX contact force. The pattern decides the image dimensions; DEX-01 is
32 × 32, while grid patterns can have different dimensions.

## Pattern configurations

`GridPatternCfg` creates a flat sampling grid. `CustomPatternCfg` reads NPZ
poses and pixel mappings. `Dex01PatternCfg` selects the supplied DEX-01 layout.
See [pattern file format](../concepts/patterns.md).

Source definitions remain authoritative; changes to these interfaces must
update this page and the integration guides in the same PR.
