# Attach a sensor

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
[numerical settings](../reference/numerical-settings.md).
