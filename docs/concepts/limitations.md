# Model scope and limitations

Both environments represent normal contact against one configured counter
object per sensor. Their output follows simulated normal contact load; it is
not a calibrated model of hardware material response.

## Contact and geometry

The exterior, mounting transform, projected contact samples and firmware
mapping are shared. Frames unwrap the curved surface. Features smaller than
the local sample spacing, including fine gear teeth, can remain unresolved.
Grey visualization cells have no taxel; occupied cells at zero load use the
zero-force color.

The model does not provide shear, hysteresis, manufacturing tolerances or
independent force estimates for multiple counter objects on one sensor.
Changing geometry, scale, material settings or a joint trajectory requires
new fixture and clearance checks. A screenshot complements these checks but
cannot replace force, mapping or fixed-pose evidence.

If physical contact occurs entirely between sampling points, no penetrating
taxel receives force and the tactile frame remains zero. On curved patches,
the total projects the net contact-force vector onto the weighted sampling
normal; opposing contact directions can cancel. It is not a sum of independently
measured local contact forces.

## Environment constraints

PhysX supports a rigid body or articulation link with a fixed mesh-to-body
transform and positive uniform scale. Its counter collision uses SDF and the
sensor body uses compliant contact; see [attachment](../physx/attachment.md).

Newton binds one sensing link and matching counter collider per environment.
Its counter mesh must be closed and nondegenerate, scale positive and uniform,
and collider-to-link transform fixed. Contact data must update every tick;
see [attachment](../newton/attachment.md).

## What validation establishes

The PhysX track records known-load, all-taxel and articulated fixture results
for specified revisions. Newton CPU mathematics, capture and recording tests
do not establish CUDA contact or camera correctness. Use the selected
[validation workflow](../validation.md), keep its exact revision and package
provenance, and assess clearance for any new pose or task.
