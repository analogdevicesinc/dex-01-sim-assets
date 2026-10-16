# Newton troubleshooting

| Symptom | Action |
|---|---|
| Unexpected checkout or package versions | Rerun the Newton installer for the exact revision, then activate its `.venv`; use the selected Kit/video extras |
| CUDA unavailable | Fix host driver/device access, then rerun preflight and required GPU tests; CPU mode does not validate Newton SDF integration |
| Data read before binding | Reset the scene, bind providers/contact/mesh, then update the sensor |
| Contact paths or force matrix rejected | Configure one sensing link and one matching counter-object filter per environment with update period zero |
| Closed mesh or transform error | Check triangle topology, degeneracy, scale and unit XYZW collider quaternion |
| Report rejects missing ticks, interruption or settling | Preserve the failed report, run complete cycles, and investigate the physical fixture rather than deleting observations |
| Native collision capacity exhausted | Inspect contact counts and scene complexity; a saturated run is invalid evidence |
| Recording/capture rejected | Check frame cadence, array mapping, geometry and hashes; regenerate from actual simulation states |

For Kit use the `isaacsim` extra and `--viz kit`; connect after
`NEWTON_SCENE_READY`. Recording also requires the `video` extra and FFmpeg.
Newton uses the EA Kit runtime and needs its own camera test. Include the Newton command, commit,
preflight JSON, GPU/driver, package versions and final log lines in an issue.
