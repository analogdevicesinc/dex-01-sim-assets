# Newton numerical settings

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

These values are source configuration, not measured hardware properties.
The recorded checks in this track validate the specified fixtures and stack. Solver or geometry changes require the
[Newton GPU tests and fixtures](validation.md).
