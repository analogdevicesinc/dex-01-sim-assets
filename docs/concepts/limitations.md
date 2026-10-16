# Supported model behavior

The sensor reports physical normal contact force in newtons. Known-load checks
verify that a 0.1 kg fixture under 9.81 m/s² produces approximately 0.981 N.

## Contact representation

Each sensor reads contact against one configured rigid body or articulation
link. Penetration into its SDF determines the relative force distribution across
taxels; the sum follows the selected backend's filtered normal contact result. The model does not provide
independent force estimates for multiple counter objects on one sensor.

Frames unwrap the curved sensing surface into firmware row/column coordinates.
They show sampled contact footprints. Features smaller than the local sampling
spacing, including fine gear teeth, can be unresolved. Grey visualization cells
have no taxel; active cells at zero force remain in the zero-force color.
If physical contact occurs entirely between sampling points, there is no
penetrating taxel to receive force and the tactile frame remains zero. On
curved patches, the reported total is a projection of the net contact-force
vector onto the weighted sampling normal; opposing contact directions can
cancel. It is not a sum of individually measured local contact forces.

## Dynamics and geometry

The current interface reports normal forces. Shear and material hysteresis
require a different model. Contact stiffness, damping, timestep, and actuator
gains are configured simulation parameters; see [numerical settings](../reference/numerical-settings.md).

The released exterior, mounting, contact sample projection, and supplied hand
pose are checked. The PhysX articulated fixture verifies its fixed wrist, finger
motion, contact, and release. Assess clearance for any new pose or trajectory
before using it in a task. See [validation](../validation.md).

## Newton boundaries

One matching sensing link and counter collider per environment is supported.
The counter mesh must be closed and nondegenerate, with positive uniform scale
and a fixed collider-to-link transform. Contact updates must run every physics
tick. Multiple independent counter objects on one sensor, shear, hysteresis and
hardware calibration are not modeled.

The single-sensor Newton demo uses the curved fingertip; it is not the PhysX
flat-grid example. Newton hand validation checks fingers 2–5 and unloaded
fingers, without the separate PhysX thumb probe. Newton press reports check
joint motion, fixed wrist/object, hold and release, but their live CUDA run has
not been repeated on this checkout. Source parameters and CPU tests alone do
not establish articulated clearance or solver correctness.
