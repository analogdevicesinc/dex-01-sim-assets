# Numerical settings

Shared geometry and units apply to both backends. Compliant-contact and
iteration settings in the first table are **PhysX** settings; Newton values
are listed separately below and are not equivalent material parameters.

## PhysX demo settings

The hand demo uses 20 g sensor mass, 10,000 N/m contact stiffness, and
50 N s/m damping at a 1/240 s timestep. These are simulation parameters.

| Values | Meaning |
|---|---|
| `mount_translation_m`, `mount_rotation_wxyz` | Fixed transform from the original taxel coordinates into the mounted fingertip frame; metres and `(w,x,y,z)` quaternion. |
| `registration_shift_in_sensor_m` | Translation fitted to the overmold interior, with no scaling or rotation of the distribution. |
| 0.1 kg and 9.81 m/s² | Known-load fixture; their product is the expected 0.981 N. |
| 20 g, 10,000 N/m, 50 N s/m | Compliant rigid-body simulation parameters, not measured hardware properties. |
| 1/240 s timestep | Advances 4.17 ms of simulation time per step: 240 steps per simulated second. |
| 192 position and 4 velocity iterations | Repeated PhysX solver passes within each step to refine contact and joint constraints; these are not taxel counts. Conservative validated demo settings, not an optimal configuration claim. |
| 0.1 mm contact offset; SDF resolutions 384/128 | Numerical contact margin and field resolutions for the sensor/test objects; unrelated to taxel count. |
| 1 ms SDF probe time | Finite-difference half-width used to estimate depth rate, independent of the physics timestep. |
| 5% load and 1 mm thumb-centroid tolerances | Verification acceptance bounds, not physical sensor specifications. |

The DEX-01 frame contains 740 taxel values in a 32 × 32 layout. A sensor
updated every step can provide 240 frames per simulated second; wall-clock
throughput depends on the host.

Camera positions, colors, and plot cadence affect presentation. Gear scale
changes collision geometry and the sampled contact patch; changes require
physics validation. The stationary-press demo uses a 1/120 s timestep and
waits for the attached sensor body to settle before measuring its endpoint.
Solver settings and actuator gains control the simulation; changes should pass
the [validation workflow](../validation.md). The contact distribution and
supported counter-object behavior are described in [the model guide](../concepts/limitations.md).

## Newton demo settings

| Setting | Value and scope |
|---|---|
| Physics timestep | 1/240 s for hand, sensor and press modes |
| Solver | MuJoCo-Warp `implicitfast`, native Newton SDF contacts (`use_mujoco_contacts=False`) |
| Substeps | 4 for hand/sensor; 16 for press |
| Collision decimation | 1 |
| Contact capacities | `nconmax=160`, `njmax=960`; native capacity exhaustion fails report runs |
| Fingertip / indenter SDF resolution | 384 / 128 |
| SDF band / padding | Inner −0.002 m, outer 0.002 m, padding 0.001 m |
| Contact `solref` / `solimp` | `(0.02, 1.0)` / `(0.9, 0.95, 0.001, 0.5, 2.0)` |
| Contact dimension / priority | `condim=3`, `priority=1` |
| Hand position drives | Stiffness 2000, damping 40, effort limit 17; demo parameters |
| Known flat fixture | 0.1 kg under 9.81 m/s²; expected 0.981 N |
| Gear in-plane extent | 6 mm in hand mode; 12 mm in press mode |
| Mesh distance-rate probe | 1 ms finite-difference half-width |
| Recording / capture cadence | Every 8 ticks, 30 frames per simulation second |

These values are source configuration, not measured hardware properties or proof
of validation on the current checkout. Solver or geometry changes require the
[Newton GPU tests and fixtures](../validation.md#newton-validation).
