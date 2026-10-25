# PhysX examples

Run examples from `physx/` in its activated environment. The videos on these
pages are PhysX captures; force frames come from the same recorded simulation
ticks as the displayed body poses.

| Example | Task | Fixture |
|---|---|---|
| [Flat pad](flat-pad.md) | Observe a broad contact footprint | 200 × 100 mm grid, 1,024 samples and a 0.5 kg gear |
| [Tesollo hand](hand.md) | Follow contact across fingers 2–5 | Curved DEX-01 layout, 740 samples per fingertip |
| [Stationary press](stationary-press.md) | Joint-driven contact and release | Fixed wrist and gear; three verification cycles |

Use the [flat-indenter validation](../validation.md) for a quantitative 0.981 N
fixture. A gear run illustrates contact and is not a calibration load.

```{toctree}
:maxdepth: 1

flat-pad
hand
stationary-press
```
