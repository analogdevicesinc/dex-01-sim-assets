# Attach a sensor

The supplied mount and exterior are shared by both backends. The first
configuration below uses PhysX; Newton requires the separate configuration
and explicit post-reset binding shown later on this page.

## Use the supplied Tesollo mount

After `AppLauncher` has initialized Isaac Sim, use the mounted hand variant
and the transform in `reference.json`. The same transform is shared by the
exterior geometry and projected taxel poses:

```python
import json
from pathlib import Path

from dex01_sim_asset import TactileSensorCfg, taxel_patterns

reference = json.loads(Path("assets/sensors/dex01/reference.json").read_text())
finger2_sensor = TactileSensorCfg(
    prim_path="{ENV_REGEX_NS}/Robot/rl_dg_2_sensor",
    mesh_prim_path="/World/envs/env_.*/Asset/factory_gear_medium",
    pattern_cfg=taxel_patterns.Dex01PatternCfg(),
    offset=TactileSensorCfg.OffsetCfg(
        pos=tuple(reference["mount_translation_m"]),
        rot=tuple(reference["mount_rotation_wxyz"]),
    ),
)
```

This is a scene-configuration fragment, not a standalone program. Add it as
`finger2_sensor` on your `InteractiveSceneCfg`; the hand and counter object
must already exist under the paths shown. Spawn `dg5f_right.usda` with
`variants={"fingertip": "dex01"}`. For the local flat indenter, select
`mesh_prim_path="/World/envs/env_.*/Asset"` instead of the nested gear link.

The complete runnable scene is in the
[hand demo](https://github.com/analogdevicesinc/dex-01-sim-assets/blob/main/scripts/dex01_on_tesollo_dg5f.py).
Its `finger_sensor()` helper is the canonical configuration for all five fingers.

## Prepare collision geometry

Both sensor body and counter object need contact sensors enabled. The sensor
body must use compliant contact; the counter object requires an SDF collision
mesh. Use the matching mounting offset and a small timestep. The
[verification example](https://github.com/analogdevicesinc/dex-01-sim-assets/blob/main/scripts/verify_dex01_sim.py) contains a complete scene
with a local sphere; the [hand demo](https://github.com/analogdevicesinc/dex-01-sim-assets/blob/main/scripts/dex01_on_tesollo_dg5f.py) shows robot
attachment. The released OBJ is the editable exterior source; regenerate its
USD with `python scripts/build_dex01_asset.py` after updating the source hash
in `reference.json`.

Supported counter objects are rigid bodies or articulation links, one per
sensor, with positive uniform scale and a fixed mesh-to-body transform shared
across environments. A wrapper path must contain exactly one rigid body; the
sensor resolves that body for both SDF queries and force filtering. Sensor
bodies also support positive uniform scale, applied to taxel positions and
their mounting offset. The supplied DEX-01 hand uses scale 1.

The sensor `prim_path` must select the fingertip rigid-body link, with a literal
leaf name. `mesh_prim_path` selects the counter object. See
[configuration fields](../reference/api.md) and
[numerical settings](../numerical-settings.md).

## Attach with Newton

Use `prepare_hand_asset()` to generate a Newton collider overlay from the
released hand. It selects the DEX-01 variant and preserves visual geometry;
PhysX compliant-contact configuration is not reused as a Newton material model.
Create the tactile sensor before reset, then bind to one link and counter mesh:

```python
from dex01_sim_asset.newton import TactileSensor, TactileSensorCfg
from dex01_sim_asset.taxel_patterns import Dex01PatternCfg

sensor = TactileSensor(TactileSensorCfg(
    prim_path="/World/envs/env_.*/Robot/rl_dg_2_sensor",
    mesh_prim_path="/World/envs/env_.*/Probe",
    pattern_cfg=Dex01PatternCfg(),
    offset_pos=tuple(reference["mount_translation_m"]),
    offset_rot_wxyz=tuple(reference["mount_rotation_wxyz"]),
))
sim.reset()
index = robot.body_names.index("rl_dg_2_sensor")
def sensing_state():
    return (robot.data.body_link_pose_w.torch[:, index],
            robot.data.body_com_pose_w.torch[:, index, :3],
            robot.data.body_com_vel_w.torch[:, index])
def counter_state():
    return (probe.data.root_link_pose_w.torch,
            probe.data.root_com_pos_w.torch,
            probe.data.root_com_vel_w.torch)
sensor.bind(sensing_state, counter_state, scene["contact2"],
            mesh.vertices, mesh.faces)
```

This fragment assumes an existing Newton scene, reference metadata, and the
same `mesh` used to spawn the probe. Configure `contact2` with
`ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/rl_dg_2_sensor",
filter_prim_paths_expr=["{ENV_REGEX_NS}/Probe"], update_period=0)`.

State providers return XYZW link poses, world COM positions and world COM
linear/angular velocities. Vertices are in collider coordinates; optional
`scale` must be positive and uniform and `mesh_pose` is the fixed
collider-to-link transform in metres/XYZW. The mesh must have nondegenerate
triangles and be closed. The contact paths must match the sensor's paths.
See the complete `scripts/dex01_newton_demo.py` scene for collider preparation,
reset order and binding all five sensors.
