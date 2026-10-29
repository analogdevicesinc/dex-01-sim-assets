# Configure taxel patterns

Use `GridPatternCfg` for a flat pad, `Dex01PatternCfg` for the humanoid
fingertip, or `RectangularDex01PatternCfg` for the rectangular finger.
Use `CustomPatternCfg` for arbitrary NPZ poses. Custom NPZ files contain `poses` of
shape `(N,3,4)`, with the third axis as the surface normal, and integer
`taxel2pixel` pairs `(row,column)`. `dex01_pattern.npz` stores the source
layout; `Dex01PatternCfg` supplies exterior contact sampling poses.
Their taxel order and pixel mapping are identical.

Read [geometry and coordinate frames](../concepts/geometry.md) before changing
a layout or mounting transform. Pattern configuration requires an initialized
Isaac Sim application.

```{figure} ../media/taxel-mapping.svg
:alt: The 740 contact samples on the curved fingertip and their matching occupied locations in the 32 by 32 tactile frame.
:width: 100%

The same taxel rows color the exterior samples and frame locations.
Grey pixels are unoccupied. These are layout coordinates, not force readings.
```

## Configuration fields

| Pattern | Required or supplied fields |
|---|---|
| `GridPatternCfg` | `resolution` and `size` in metres; `ordering` is `xy` by default or `yx` |
| `CustomPatternCfg` | NPZ `filename` and `resolution` in metres |
| `Dex01PatternCfg` | Supplied exterior NPZ and resolution `(0.00072, 0.00088)` m |
| `RectangularDex01PatternCfg` | Supplied rectangular grid: `positions_m` and `normals`, both `(32,32,3)` |

For the [Robotiq finger](robotiq.md), the file is indexed directly
by row and column. Rows run tip to base and columns left to right when viewing
the contact face with the tip up. The pattern loader adapts these arrays to
the existing sensor buffers without introducing separate IDs.

The pattern generator returns positions `(T, 3)`, axes `(T, 3, 3)`, and pixel
indices `(T, 2)`. Use the matching mounting transform when selecting the
supplied exterior pattern.

## Validate a custom layout

Before loading a custom NPZ into a scene, verify the file arrays:

```python
import numpy as np

with np.load("my_pattern.npz", allow_pickle=False) as layout:
    poses = layout["poses"]
    pixels = layout["taxel2pixel"]
assert poses.ndim == 3 and poses.shape[1:] == (3, 4)
assert pixels.shape == (len(poses), 2)
assert np.isfinite(poses).all()
assert np.issubdtype(pixels.dtype, np.integer) and (pixels >= 0).all()
assert len(np.unique(pixels, axis=0)) == len(pixels)
axes = poses[:, :, :3]
assert np.allclose(axes.transpose(0, 2, 1) @ axes, np.eye(3), atol=1e-6)
assert np.allclose(np.linalg.det(axes), 1, atol=1e-6)
```

Use local positions in metres and right-handed orthonormal axes. The third
axis is the outward surface normal. Pixel indices start at zero and are
ordered `(row, column)`. Empty cells are automatically zero-filled; choose
indices appropriate to the frame dimensions your consumer expects.
