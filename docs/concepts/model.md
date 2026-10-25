# Contact-force model

Each DEX-01 fingertip has 740 contact samples on one exterior-only mesh,
mapped into a 32 × 32 firmware frame. Both environments model normal contact
load; they do not produce calibrated hardware measurements.

## From contact to taxel loads

For penetrating sample depths `d_i < 0`, compute an outward contact normal
from the penetration-weighted surface normals. Project the counter-object's
filtered normal-force vector onto it to obtain nonnegative load `F_n`.
Distribute `f_i = F_n d_i / sum(d_j)` across penetrating samples. Separated
samples and environments without filtered contact receive zero load.

The sum follows the simulated filtered normal load while the spatial
footprint follows sampled penetration. This redistribution is based on
[TacSL](https://arxiv.org/abs/2408.06506); it does not independently measure
multiple contacts or identify a material law for every taxel.

## Where the environments differ

| Input | PhysX | Newton |
|---|---|---|
| Contact dynamics | PhysX compliant contact | Native Newton SDF contacts with MuJoCo-Warp |
| Distance queries | PhysX signed-distance view | Warp queries using Newton's mesh SDF interface |
| State access | PhysX rigid-body/link views | Explicit link pose, COM and velocity callbacks |
| Mount and public data | WXYZ quaternions | WXYZ public data; XYZW state callbacks |

Solver parameters belong to their environment:
[PhysX settings](../physx/numerical-settings.md) and
[Newton settings](../newton/numerical-settings.md).
Use the selected track's fixture checks for force and momentum balance.
The [shared geometry](geometry.md), [data contract](../reference/data.md)
and [limitations](limitations.md) describe the model boundary.
