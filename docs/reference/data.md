# Tactile data and firmware mapping

Both environments return the shared `dex01_sim_common.TactileSensorData`
contract as Torch tensors. Initialize through the selected backend's
[PhysX lifecycle](../physx/api.md) or [Newton binding](../newton/api.md)
before reading data.

## Shapes and units

`E` is the number of environments and `T` the number of taxels.

| Field | Shape | Interpretation |
|---|---|---|
| `normal_forces` | `(E, T)` | Nonnegative normal load per sample, N |
| `taxel_points_w` | `(E, T, 3)` | World positions, m |
| `taxel_normals_w` | `(E, T, 3)` | Unit outward world normals |
| `taxel_rot_w` | `(E, T, 4)` | World quaternions, WXYZ |
| `depths` | `(E, T)` | Penetration, m; negative in contact, zero otherwise |
| `depth_dots` | `(E, T)` | Penetration-depth rate, m/s |

Both sensors expose `num_taxels`, `taxel2pixel` and `get_tactile_image()`.
The frame shape is `(E, height, width)`, derived from the layout's pixel indices.
The supplied DEX-01 has 738 occupied cells in a 32 × 32 frame; 286 cells have no
taxel. Zero in an occupied cell means no load at that sample.

## Read one environment

The following fragment applies to either initialized backend sensor:

```python
import torch

forces_n = sensor.data.normal_forces[0]
frame_n = sensor.get_tactile_image()[0]
rows, columns = sensor.taxel2pixel.T
assert torch.equal(frame_n[rows, columns], forces_n)
print("Normal load (N):", forces_n.sum().item())
```

Taxel order and firmware row/column indices are preserved. Unoccupied pixels
are zero. Do not interpolate the frame when checking force or mapping; a
firmware heatmap unwraps the curved surface and is not an optical image.

For PhysX, the tactile sensor usually belongs to the scene. Newton tactile
sensors are explicitly constructed and bound to a scene contact sensor, then
updated every tick. Link callbacks in Newton use XYZW, while this public data
contract uses WXYZ. See the selected backend API for update order.
