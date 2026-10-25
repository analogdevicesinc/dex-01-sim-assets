# Geometry and coordinate frames

The exterior mesh, 740 contact poses, firmware mapping and mounting metadata
on this page are shared by PhysX and Newton. Public taxel orientations and
reference mounting quaternions use WXYZ. Newton's link-state and collider-pose
callbacks use XYZW; convert at that interface, not in the stored pattern.


The fingertip exterior is 32.70 × 22.41 × 19.00 mm; its rear profile is
22.40 × 19.00 mm. OBJ positions, mounting translations, and simulator
positions are stored in **metres**. Reference mounting and public taxel quaternions use **`(w, x, y, z)`**;
backend state interfaces document their own conventions.

## Registration and mounting

The original layout aligns to the overmold interior by one translation of
approximately `(-2.775670, -0.0000166, -1.429237)` mm in the sensor frame, with
no scaling, additional rotation or individual taxel movement. Reference-CAD
comparison gave 1.37 µm RMS and 4.29 µm maximum residual. Contact poses were then
projected along the original normals through approximately 1.43 mm of overmold.
`assets/sensors/dex01/reference.json` records this shift and the mounting offset.
Offset quaternions use `(w,x,y,z)`; translations and OBJ coordinates use metres.

## Asset sources

The exterior OBJ and `reference.json` live under `assets/sensors/dex01/`.
The mounted sensor and Tesollo hand USD assets live under `assets/robots/tesollo/`.
Taxel NPZ files ship inside the Python package. Fetch them with `git lfs pull`.

The geometry builder and verification scripts are the reference workflow for
changes; see [attachment](../physx/attachment.md) and [validation](../validation.md).

## Transform a contact sample

An NPZ pose stores its three local axes in columns 0–2 and position in column 3.
Column 2 is the outward normal. First apply the mount rotation and translation
from `reference.json` to obtain coordinates in the sensor rigid body. Then
apply that body's live world pose from the selected simulator. For a point `p`:

```text
p_body  = R_mount p_pattern + t_mount
p_world = R_body  p_body    + t_body
```

Normals receive the rotations without translation. These are fixed mount
transforms; the sensor's live body pose changes as the hand articulates.
The [capture utility](https://github.com/analogdevicesinc/dex-01-sim-assets/blob/main/DOCUMENTATION.md#reproduce-the-figures) verifies
world-space sample alignment against every mounted visual mesh.

Use the exterior pattern supplied by `Dex01PatternCfg` for contact sampling.
The reference `dex01_pattern.npz` retains the original layout for traceability; it is not the
projected exterior sampling surface. Both files keep the same taxel order and
firmware pixel mapping.
