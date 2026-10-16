# API reference

The common data buffers and pattern types below apply to both backends.
`dex01_sim_asset.TactileSensor` is PhysX; `dex01_sim_asset.newton.TactileSensor`
is Newton. Configuration fields and binding lifecycle are separate.

## PhysX sensor interface

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

See [reading data](../guides/read-data.md) for an executable usage snippet.

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

Positive uniform scale inherited by the sensor body applies to local taxel
positions and `offset.pos`; normals retain unit length. Nonuniform sensor
scaling is rejected. A counter-object wrapper must resolve to exactly one
rigid body, which is also used by the contact-force filter.

```{literalinclude} ../../source/dex01_sim_asset/dex01_sim_asset/tactile_sensor_cfg.py
:language: python
:start-at:     mesh_prim_path:
:end-before:     visualizer_cfg:
```

## Shared TactileSensorData

Here `E` is the number of environments and `T` the number of taxels.
Buffers are initially `None` and become tensors during sensor initialization.

| Field | Shape | Units and interpretation |
|---|---|---|
| `normal_forces` | `(E, T)` | Normal force magnitudes, newtons |
| `taxel_points_w` | `(E, T, 3)` | Sample positions in world coordinates, metres |
| `taxel_normals_w` | `(E, T, 3)` | Unit world normals |
| `taxel_rot_w` | `(E, T, 4)` | World quaternions `(w, x, y, z)` |
| `depths` | `(E, T)` | Metres; negative in contact and zero otherwise |
| `depth_dots` | `(E, T)` | Penetration-depth rate, metres per second |

## PhysX lifecycle

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
See [pattern file format](../guides/patterns.md).

Source definitions remain authoritative; changes to these interfaces must
update this page and the integration guides in the same PR.

## Newton TactileSensorCfg

Import from `dex01_sim_asset.newton`. Inherits the pinned EA `SensorBaseCfg`.

| Field | Meaning |
|---|---|
| `prim_path` | Sensing-link path, one link per environment |
| `mesh_prim_path` | Configured counter-object path matching the contact filter |
| `pattern_cfg` | Shared pattern configuration |
| `offset_pos` | Mount translation in metres |
| `offset_rot_wxyz` | Unit WXYZ mount quaternion; identity by default |
| `debug_vis` | Enable sampling markers in Kit |

## Newton TactileSensor lifecycle

| Interface | Contract |
|---|---|
| `bind(sensor_state, counter_state, contact, vertices, faces, scale=1.0, mesh_pose=None)` | Call after scene reset; closed triangle mesh and one matching contact filter required |
| State callbacks | Return Torch link pose `(E,7)` in metres/XYZW, world COM position `(E,3)`, COM velocity `(E,6)` in m/s and rad/s |
| `mesh_pose` | Fixed collider-to-counter-link pose `(7,)` in metres/XYZW |
| `update(dt)` and `data` | Refresh the bound sensor after scene/contact updates |
| `get_tactile_image()` | Same exact firmware mapping as PhysX |
| `get_filtered_normal_force_w()` | Torch world normal-force vector `(E,3)` from the configured counter filter |
| `frame_state()` | Current sensing-link callback result |
| `reset(env_ids=None, env_mask=None)` | Clear selected force/depth buffers; scene reinitialization requires rebinding |

`TactileSensorData` is shared and public `taxel_rot_w` remains WXYZ. A data read
before binding raises an error. Contact update period must be zero and its force
matrix must contain exactly one sensing link and one counter object per environment.
