# Contact-force model

An Isaac Lab tactile sensor model with a physically scaled DEX-01 fingertip
on the Tesollo DG-5F hand. The fingertip is one exterior-only mesh, approximately
32.70 x 22.41 x 19.00 mm, with a 22.40 x 19.00 mm rear profile. It has 738 contact
samples mapped into a 32x32 firmware frame.

The model queries the counter object's PhysX signed distance field (SDF) and
distributes the measured normal contact load across penetrating samples by
depth, following [TacSL](https://arxiv.org/abs/2408.06506). Output is normal contact force in newtons.

Each sample queries the counter-object SDF. Penetrating samples receive a
share of the measured net normal contact force according to penetration depth.
The frame unwraps the fingertip surface; it is not an optical contact image.

A taxel is one contact sample on the fingertip surface. The 738 samples retain
the firmware pixel mapping; unoccupied frame cells contain zero. Summing all
taxel values gives the sensor’s total normal load.

Use [known-load validation](../validation.md) to check simulated force balance
and [limitations](limitations.md) to understand the model boundary.

## Force distribution

For taxel penetration depths `d_i < 0`, the model computes a contact normal
from the penetration-weighted surface normals and projects the filtered PhysX
contact-force vector onto it. The nonnegative projected load `F_n` is allocated
as `f_i = F_n d_i / sum(d_j)` over the penetrating taxels. Samples outside the
counter object receive zero.

This preserves the total physical normal load while distributing it across the
sampled contact patch. The known-load checker compares that sum with the
independent filtered contact force, fixture weight, and momentum change.
