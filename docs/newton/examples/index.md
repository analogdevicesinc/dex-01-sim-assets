# Newton examples

Run from `newton/` after [installation](../installation.md). Each script uses
this directory's pinned environment. The hand and press videos on these pages use verified Newton captures with
native RTX replay, exact firmware arrays, stopped physics and geometry audits.
The [capture/render provenance](../../media/newton-examples.json) records their
source hashes and runtime versions.

| Example | Task | Layout and fixture |
|---|---|---|
| [Single fingertip](fingertip.md) | Inspect contact and sweep all samples | Curved DEX-01, 740 taxels and a mesh sphere |
| [Tesollo hand](hand.md) | Follow known loads across fingers | Five firmware frames with a 0.1 kg flat fixture |
| [Stationary press](stationary-press.md) | Joint-driven contact and release | Fixed wrist and gear; three 2.5 s cycles |

Newton supports `GridPatternCfg`, but this repository has no Newton flat-pad
scene equivalent to the PhysX 200 × 100 mm grid example. The single-sensor
mode uses the curved fingertip.

```{toctree}
:maxdepth: 1

fingertip
hand
stationary-press
```
