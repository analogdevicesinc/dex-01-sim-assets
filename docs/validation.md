# Validation by environment

Validation has two parts: shared geometry/data contracts and live checks in
the selected physics runtime. Run the appropriate backend workflow:

- [Validate PhysX](physx/validation.md): known loads, thumb localization,
  all-taxel contacts and articulated fixed-wrist pressing.
- [Validate Newton](newton/validation.md): real Warp/MuJoCo-Warp contacts,
  known hand loads, all-taxel sweep, transforms and three press cycles.

The shared CPU suite tests pattern identity, geometry, exact firmware mapping,
report rejection, USD export/reload, capture corruption and video evidence.
It does not replace GPU contact or rendering tests. Missing dependencies or
device access must fail when GPU validation is required; a skip is not a pass.

Keep the exact source commit, package and driver versions, command, logs and
reports with evidence. Capture and render hashes establish artifact identity;
they cannot make an unvalidated physical run correct.
