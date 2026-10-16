# PhysX numerical settings

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
the [validation workflow](validation.md). The contact distribution and
supported counter-object behavior are described in [the model guide](../concepts/limitations.md).
