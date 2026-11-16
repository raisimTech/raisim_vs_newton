# External dependencies and model provenance

No RaiSim implementation, SDK headers, prebuilt library, activation key, or
private model file is redistributed by this repository.

- **RaiSim:** fetched from the public [raisim2Lib SDK](https://github.com/raisimTech/raisim2Lib).
  Its prebuilt binaries and headers retain the SDK's license requirements; users
  provide their own valid activation key. See its `LICENSE.md`.
- **ANYmal:** prepared from the public SDK's `rsc/anymal/urdf/anymal.urdf`.
  The model is Copyright 2019 ANYbotics AG, BSD 3-Clause. Model preparation copies
  the resource's complete license into `build/assets/licenses/ANYmal.txt`.
- **Strandbeest:** prepared from the public SDK's `rsc/strandbeest/strandbeest.urdf`,
  derived from [StrandbeestRobot.jl](https://github.com/rdeits/StrandbeestRobot.jl),
  Copyright 2018 Robin Deits, MIT. Its complete notice is copied to the generated
  asset directory. Nominal link poses are queried using the public SDK API;
  benchmark-specific USD and closure-anchor files are generated from these data.
- **Newton:** installed from the public [Newton repository](https://github.com/newton-physics/newton)
  at the commit pinned in `requirements.txt`, Apache-2.0. The benchmark calls its
  public API; solver implementations are not vendored here.
- **Warp, MuJoCo, MuJoCo Warp, USD and NumPy:** installed as public dependencies;
  their own distributions contain their applicable licenses.

The benchmark drivers are MIT-licensed code in this repository. The reference
measurements in `reports/published_reference.json` contain numerical data extracted
from the public benchmark page, with source URLs and retrieval date. They are
historical observations, not expected exact wall-clock timings on other builds.
