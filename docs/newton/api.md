# Newton sensor API

Import `TactileSensor` and `TactileSensorCfg` from `dex01_sim_newton`.
Output buffers and units follow [the common data reference](../reference/data.md).

## TactileSensorCfg

The configuration inherits the pinned EA `SensorBaseCfg`.

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
