# Choose a sensor API

The repository provides a common data contract and two separately installed
backend packages. Select the API for your active environment.

| Package | Owns | Reference |
|---|---|---|
| `dex01_sim_asset` | PhysX sensor and scene configuration | [PhysX API](../physx/api.md) |
| `dex01_sim_newton` | Newton sensor and post-reset binding | [Newton API](../newton/api.md) |
| `dex01_sim_common` | Data buffers, patterns and visual helpers | [Data](data.md), [patterns](../concepts/patterns.md) |

The backend packages have no dependency on each other. Installing the common
library does not choose a simulator or initialize a backend.
Their public force arrays have the same shapes and units; configuration fields,
state conventions and lifecycle differ.
