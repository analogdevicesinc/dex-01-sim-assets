# Read tactile data

Simulator modules must be imported after `AppLauncher` starts Isaac Sim.
After `sim.reset()`, read data after `scene.update(sim_dt)` in the simulation
loop. The following fragment assumes the scene has `finger2_sensor`:

```python
import torch

sensor = scene["finger2_sensor"]
forces_n = sensor.data.normal_forces[0]  # 740 values
frame_n = sensor.get_tactile_image()[0]  # 32 x 32 firmware frame
rows, columns = sensor.taxel2pixel.T
assert torch.equal(frame_n[rows, columns], forces_n)
print("Load (N):", forces_n.sum().item())
```

Unoccupied pixels are zero. World positions and normals are available in
`taxel_points_w` and `taxel_normals_w`. `depths` is negative in contact and zero
otherwise; `depth_dots` is in metres per second.

See the [API reference](../reference/api.md) for shapes and units.

For a batched scene, replace `[0]` with the environment index you need.
`normal_forces` contains one value per taxel; `get_tactile_image()` rearranges
those values into firmware pixels. Summation in taxel space and frame space
returns the same load. Do not interpolate the frame when checking mapping.

The supplied DEX-01 frame has 740 occupied pixels and 284 unoccupied pixels.
A zero at an occupied pixel means no normal load at that sample. Use
`taxel2pixel` to distinguish occupied zero-force pixels from unused cells.
