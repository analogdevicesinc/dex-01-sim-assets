# Supported model behavior

The sensor reports physical normal contact force in newtons. Known-load checks
verify that a 0.1 kg fixture under 9.81 m/s² produces approximately 0.981 N.

## Contact representation

Each sensor reads contact against one configured rigid body or articulation
link. Penetration into its SDF determines the relative force distribution across
taxels; the sum follows PhysX's normal contact result. The model does not provide
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

Compliant rigid contact is a numerical approximation: its effective stiffness
depends on the sampled contact manifold. A fixed counter object can support
unequal reaction forces. The Robotiq grasp example uses a free rigid block and
checks both bilateral loads and force/momentum balance; this does not identify
the sensor's physical material properties.

The current interface reports normal forces. Shear and material hysteresis
require a different model. Contact stiffness, damping, timestep, and actuator
gains are configured simulation parameters; see [numerical settings](../reference/numerical-settings.md).

The released exterior, mounting, contact sample projection, and supplied hand
pose are checked. The articulated demo also verifies its fixed wrist, finger
motion, contact, and release. Assess clearance for any new pose or trajectory
before using it in a task. See [validation](../validation.md).
