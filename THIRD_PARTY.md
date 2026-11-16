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

The manipulation driver extracts `rsc/robotiq_2f85/kinova_robotiq.urdf` from the
pinned public SDK at runtime, retaining its collision/inertial/mimic data and
removing visual elements and adding the equivalent capsule `height` attribute
for Newton importer compatibility. Kinova descriptions are Copyright (c) 2017 Kinova
Robotics inc.; Robotiq descriptions are Copyright (c) 2022 PickNik Robotics.
Both use BSD 3-Clause; complete licenses accompany the generated assets as
`kinova-LICENSE` and `robotiq_2f85-LICENSE`. Motion poses and gains follow the
public SDK's `examples/include/robotiq_pick_and_place.hpp`; simulator integration
and benchmark control are implemented independently here.
