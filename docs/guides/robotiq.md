# Integrate the Robotiq 2F-85

Use this guide to configure or reuse the rectangular tactile fingers. To run
the scene and interpret its output, start with the
[grasp, hold, and release example](../examples/robotiq.md).

The integration references the official Isaac Sim 5.1 Robotiq asset with its
`Physx_Mimic` variant. It replaces the stock fingertips on the inner-finger
links and preserves the original moving linkage and joints. Isaac Sim loads
the NVIDIA geometry from its asset library.

## Resolve the official asset library

The grasp example resolves the library with Isaac Sim's standard
`isaacsim.storage.native.get_assets_root_path()` API and appends
`Isaac/Robots/Robotiq/2F-85/Robotiq_2F_85_edit.usd`. It follows the configured
`/persistent/isaac/asset_root/default` setting, so a changed server or local
asset pack does not require editing the gripper integration.

For one invocation, `--asset-root /path/to/Assets/Isaac/5.1` selects another
library root. `--gripper-usd /path/to/Robotiq_2F_85_edit.usd` selects an exact
file instead. Keep its relative `configuration`, `payloads`, `parts` and
`materials` dependencies.

The reusable USD uses the standard `ISAAC_ASSET_ROOT` expression variable,
with the versioned NVIDIA 5.1 library as its default. A containing USD stage can
override that variable without changing the asset or its reference path:

```python
stage.GetRootLayer().expressionVariables = {
    "ISAAC_ASSET_ROOT": "/path/to/Assets/Isaac/5.1",
}
```

The builder also accepts `--asset-root` or `--gripper-usd`. USD expressions do
not read application settings themselves; the runnable example explicitly
connects the native asset-root API to the generated layer. The relative gripper
path and articulation contract remain specific to the tested asset revision.
See [NVIDIA's asset-root configuration](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_faq.html)
and [OpenUSD variable expressions](https://openusd.org/release/user_guides/variable_expressions.html).

## Geometry and indexing

The finger combines the adapter, carrier, and sensing exterior into one
connected, watertight mesh, omitting the PCB. The same mesh supplies appearance
and SDF collision geometry. Its contact face carries the rectangular sampling
grid described in [taxel patterns](patterns.md).

The local origin is at the mounting bore on the bottom mating plane. X runs
across the finger, Y toward the contact face, and Z toward the tip. Positions
are in metres and quaternions use `(w,x,y,z)`. The approved mesh and grid hashes
and coordinate conventions are in `assets/robots/robotiq_2f85/reference.json`.

The thicker fingers require a **35.9° closing limit**, below the stock 47°
limit. The authored parallel-jaw kinematics retain at least 0.5 mm of clearance
between opposing finger envelopes at this stop. Recheck clearance if changing
the geometry or mounting.

## Use the assets in another scene

Spawn `assets/robots/robotiq_2f85/robotiq_2f85_dex01.usda` as an articulation.
It contains a fixed joint anchoring its base. The standalone finger is
`assets/robots/robotiq_2f85/finger.usda`; its single `geometry` mesh supplies
separate material subsets for the structure and sensing surface.

Use `RectangularDex01PatternCfg` and point the sensors at the existing
`Robotiq_2F_85/left_inner_finger` and `right_inner_finger` rigid-body links.
The example's `sensor()` configuration applies the same mount transform as
the finger geometry. `mesh_prim_path` must identify one SDF counter object per
sensor, as described in [attachment](attachment.md).

The finger mass, assembled jaw mass, contact stiffness/damping and actuator
gains are simulation assumptions. The supplied free-block check validates
contact and indexing; it does not identify hardware dynamics or sensor material
properties. The model reports normal force only.

## Rebuild the assets

Rebuild the USD assets from the approved exterior OBJ and taxel grid:

```bash
python scripts/build_robotiq_asset.py
python -m pytest tests/test_robotiq_assets.py tests/test_taxel_patterns.py -q
```

USD rebuilding requires `usd-core` or the initialized Isaac Sim environment.
Outside Isaac Sim, USD may warn that it cannot resolve the NVIDIA HTTPS
reference; the integration layer is still authored. To inspect the composed
stage with ordinary USD tools, rebuild into a temporary directory using
`--gripper-usd` and an unchanged local mirror.

After changing sensing, mounting, or geometry, run the
[grasp validation](../validation.md#robotiq-grasp-hold-and-release).
For recording instructions, see the
[documentation maintainer guide](https://github.com/analogdevicesinc/dex-01-sim-assets/blob/main/DOCUMENTATION.md#reproduce-example-videos).
