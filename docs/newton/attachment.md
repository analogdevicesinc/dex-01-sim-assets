# Attach a Newton sensor

Use `prepare_hand_asset()` to generate a Newton collider overlay from the
released hand. It selects the DEX-01 variant and preserves visual geometry;
PhysX compliant-contact configuration is not reused as a Newton material model.
Create the tactile sensor before reset, then bind to one link and counter mesh:

```python
from dex01_sim_newton import TactileSensor, TactileSensorCfg
from dex01_sim_common.taxel_patterns import Dex01PatternCfg

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

Run this fragment from `newton/` inside an existing Newton scene. `reference`
contains `../assets/sensors/dex01/reference.json`, and `mesh` is the same
triangle mesh used to spawn the probe. Configure `contact2` with
`ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/rl_dg_2_sensor",
filter_prim_paths_expr=["{ENV_REGEX_NS}/Probe"], update_period=0)`.

State providers return XYZW link poses, world COM positions and world COM
linear/angular velocities. Vertices are in collider coordinates; optional
`scale` must be positive and uniform and `mesh_pose` is the fixed
collider-to-link transform in metres/XYZW. The mesh must have nondegenerate
triangles and be closed. The contact paths must match the sensor's paths.
See the complete `newton/scripts/dex01_newton_demo.py` scene for collider preparation,
reset order and binding all five sensors.
