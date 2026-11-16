# RaiSim vs Newton

Standalone, headless benchmarks for ANYmal and Strandbeest with independent random
actuation every ten physics steps. The launcher asks for your CPU and GPU prices
in **USD**, then reports **FPS** and **FPS/USD** for each engine and world count.

Only benchmark code lives here. CMake obtains the **public**
[raisim2Lib SDK](https://github.com/raisimTech/raisim2Lib); that project's CMake
configuration downloads the matching prebuilt RaiSim release. No private RaiSim
repository, implementation source, copied SDK headers, or activation key is included.

## Requirements

- CMake 3.24+, Git, and a C++20 compiler. On Linux use Clang 20.
- Python 3.12–3.13 (3.12 recommended) and the public Python packages below.
- A valid RaiSim activation key. Use RaiSim's normal `~/.raisim` installation or
  set `RAISIM_ACTIVATION_KEY` to the **path** of your own activation file.
- Newton's GPU benchmark requires a supported NVIDIA CUDA GPU and driver.
  RaiSim's CPU executable supports the SDK's Linux, macOS and Windows platforms,
  including the SDK's ARM packages. CUDA does not run on macOS; select the CPU
  backend explicitly there. There is no silent GPU-to-CPU substitution.

## Build

```sh
python3.12 -m venv .venv
. .venv/bin/activate
python -m pip install -r requirements.txt
cmake -S . -B build -DCMAKE_CXX_COMPILER=clang++-20 -DCMAKE_BUILD_TYPE=Release
cmake --build build -j12
```

CMake sparse-clones the pinned public SDK into `build/_deps/raisim2Lib`, then
configures it with examples, Python bindings and documentation disabled. The SDK
selects and downloads its platform's prebuilt libraries. All downloads remain in
the ignored build directory. Models are prepared from its public `rsc` assets.
The default SDK revision is pinned in CMake; changing it may change simulation results. Explicit local SDK checkouts record their actual commit and tracked dirty status, rather than claiming the default pin. Runs require a clean SDK checkout matching the configured commit; reconfigure and rebuild after changing it.

To reuse your public SDK checkout instead:

```sh
cmake -S . -B build -DCMAKE_CXX_COMPILER=clang++-20 \
  -DBENCHMARK_SDK_SOURCE_DIR=../raisim2Lib
```

On Windows activate `.venv\Scripts\Activate.ps1`, omit the Clang option when using
MSVC, build with `cmake --build build --config Release -j12`, and supply that
configuration with `--config Release`; CMake generates a separate paths manifest for each configuration.

## Run

```sh
python benchmark.py --build build --worlds 1,64,256,1024,4096
```

The first prompts are:

```text
How much did your CPU cost (USD)?
How much did your GPU cost (USD)?
```

Use actual component purchase prices; these are not hard-coded list-price
assumptions. Values must be positive and finite. For unattended runs:

```sh
python benchmark.py --build build --worlds 1,64,256 --cpu-price 339.99 --gpu-price 899.99
```

The output contains model, worlds, backend, CPU workers, aggregate FPS, and
FPS/USD. JSON, CSV, provenance and per-case logs are written to `results/`.
Choose a fresh `--output` directory for each run so old results cannot be mistaken
for new measurements. Optional exportable charts:

```sh
python -m pip install -r requirements-plots.txt
python scripts/plot_results.py --results results/results.json --output results/plots
```

```sh
# Short functional run; not a stable performance measurement.
python benchmark.py --worlds 1 --steps 100 --repeats 1
# Larger GPU sweep; ensure the batch fits your VRAM.
python benchmark.py --backend newton --worlds 4096,8192,16384 --models anymal
# CPU-only or explicit worker count.
python benchmark.py --backend raisim --threads 12 --worlds 64,256
```

Benchmarks run **serially**. CPU worlds use persistent workers and synchronize
at each control interval; each worker advances a world ten steps before its next
world. Workers are pinned by default on Linux; use `--no-pin-threads` to disable pinning. Pinning is unavailable on other platforms. Avoid running other workloads
alongside scored benchmarks. Large batches can exhaust GPU or host memory; a
failed case is reported as an error, never as valid throughput.

The native CPU program also prompts for prices when not supplied:

```sh
python scripts/prepare_assets.py --build build
build/bin/raisim_benchmark --assets build/assets --model anymal --worlds 64 --threads 12
```

CPU-only builds and model preparation do not require the Newton Python packages:
model preparation uses Python's standard library and the public SDK exporter.
The launcher asks both hardware prices by design; the GPU price is not used to
normalize CPU-only measurements.

## What is measured

- ANYmal: 12 PD position targets, nominal pose ±0.15 rad, Kp 200, Kd 10.
- Strandbeest: crank-speed target ±25% around 2π rad/s; Kd 200; passive linkage joints.
- Repeatable seed 20261115, keyed by environment, control interval, and joint.
  Both engines receive the same float32 action values; physics uses RaiSim FP64
  and Newton SolverMuJoCo / MuJoCo Warp FP32.
- Nominal warmup: 1,100 ANYmal or 100 Strandbeest steps. Trials restore state,
  solver warm starts and the action counter outside timing. Default horizons are
  5,000 ANYmal steps (2 ms) or 3,000 Strandbeest steps (1 ms). Use medians of three trials, except single-world RaiSim uses seven; no trials are discarded. Default workers are 1 at one world, 16 at 64 worlds, and 20 at higher counts, capped by available CPU affinity. These defaults reproduce the reference CPU scheduling; use `--threads N` for other hardware, and account for SMT/hybrid-core topology when choosing affinity.
- Timed work includes random action generation, target updates and physics.
  It excludes model setup, compilation, policy inference, observations, rewards,
  episode resets and rendering. This is **not full RL-training throughput**.
- **FPS = worlds × physics steps ÷ median wall seconds**. Divide FPS by ten for
  aggregate control transitions/s. **FPS/USD = FPS ÷ the full component price**:
  RaiSim uses CPU price; Newton uses GPU price. Do not prorate CPU price by workers.
  Host system, RAM, power, cooling, taxes and licensing costs are excluded; the
  GPU also requires a host CPU.

Solvers, constraints and contact trajectories differ. Equal action values do not
establish equal numerical work or equal accuracy. Untimed replays check finite
states, contacts, overflow and closure gaps. CPU loop gaps are sampled every ten
steps, GPU gaps every step. Raw diagnostics and timings accompany the results.

Strandbeest is generated from the public SDK's licensed URDF using link poses
queried through the public SDK. Six redundant crank-axis bearings are restored to
make 42 authored closure hinges / 84 anchor points. RaiSim recognizes and ignores
12 redundant pin/equality constraints; Newton retains 84 connect equalities. This
matches the reference workload but is not an identical constraint-system comparison.
No private USD file is copied.

## Reproduce the published page

The reference is [RaiSim vs Newton](https://raisim.com/sections/BenchmarkNewton.html).
`reports/published_reference.json` records its random-actuation numerical data,
retrieved 16 November 2026. To run all **25** cases on a supported x86-64 Linux machine:

```sh
cmake -S . -B build -DCMAKE_CXX_COMPILER=clang++-20 -DBENCHMARK_X86_64_V3=ON
cmake --build build -j12
python benchmark.py --extended-gpu --cpu-price 339.99 --gpu-price 899.99 --output results/reference
python scripts/compare_reference.py --results results/reference/results.json --output results/reference/comparison.json
```

The five primary CPU counts are 1, 64, 256, 1,024, and 4,096. GPU-only cases add
8,192 / 16,384 / 32,768 for ANYmal and 8,192 / 16,384 for Strandbeest. The driver
stops adding GPU cases after a sampled process-memory peak exceeds 5,000 MiB by
default; adjust `--max-gpu-memory-mib` for your card. Sampling is not an allocator-exact peak and is performed during the runs, as in the reference sweep. The driver warns if process memory cannot be queried (for example under WDDM or PID namespaces); in that case the memory guard is not enforced.

The comparison reports every FPS difference, action-checksum agreement, protocol
agreement and a configurable tolerance. It never substitutes published numbers for
new measurements. Exact timing equality is not a realistic portability guarantee:
CPU/GPU model, clocks, drivers, compiler and RaiSim binary version affect timings.
The public SDK currently downloads **RaiSim 2.16.0**, whereas the published reference
used an earlier development binary. Strandbeest USD is regenerated from the public
asset, so numerical trajectories need not be bit-identical to the historical USD.
The [actual public-SDK reproduction report](reports/reproduction-20261116.md) records all 25 cases, exact action agreement and the observed FPS differences.

## Tests

```sh
ctest --test-dir build -C Release -j12 --output-on-failure
```

Contract tests cover price validation, FPS/USD arithmetic, deterministic action
values, CLI errors, and the public-dependency boundary. Actual simulation smoke
runs require your activation key; GPU runs additionally require CUDA. A new SDK
release need not reproduce older development-build performance numbers.

## Randomized manipulation scaling

The separate driver uses the public **Kinova arm + Robotiq 2F-85 mimic**
pick-and-place example. Build the new `manipulation_benchmark` target with the
same CMake commands above, then run:

```sh
python manipulation.py --worlds 1,16,64,256,1024 --cpu-price 339.99 --gpu-price 899.99 --output results/manipulation
```

RaiSim advances independent worlds using **30 persistent CPU workers** by
default, capped by the number of worlds. Override this with `--threads N`.
Workers own disjoint worlds and synchronize only at trial boundaries; thread
creation is excluded from timing. Worlds are statically divided between workers,
which are not pinned; uneven partitions, shape costs and mixed CPU core speeds
can limit utilization. Available CPU affinity and requested worker count are
recorded in provenance. Thirty workers are retained even on machines with
fewer than thirty available CPUs. Newton batches
those worlds on CUDA using `SolverMuJoCo` and the **Newton collision pipeline**;
its body-variant API supports different primitive types in one batch with that
pipeline. This differs from the native MuJoCo contacts used by the older benchmarks.
There is no CPU substitution when CUDA is unavailable.

Each world has one 100 g object: a seeded box, sphere, or vertical cylinder,
with independently randomized dimensions (36–44 mm diameter/box sides/cylinder
height). Geometry stays fixed for that world across resets. Object yaw and pickup
position (±2 mm) change on each reset. The six arm joints and the single gripper
driver use the example's PD gains; all five mimic followers remain passive.
Self-collision is disabled in both engines. The table and ground collide normally.

The 21.5 s scripted cycle approaches, closes, lifts, transports, releases and
returns using the example's joint poses and smooth trajectory. Per-world delays
of 0–1 s and reset periods of 23–26 s desynchronize episodes. Independent, uniform
**±0.25 Nm additive gripper torque** changes every ten 1 ms physics steps. Both engines use one integration step
per frame, so `aggregate_fps` and `solver_steps_per_second` are equal. This
perturbs applied grip torque rather than only the position target. The robot and
object states and solver warm starts reset independently per world. Both engines
use the same seed, object specifications, targets, perturbations and reset schedule.
RaiSim uses FP64 physics and Newton FP32, so trajectories and grasp outcomes differ.
Both use circular/elliptic friction cones and matched sliding friction pairs
(pad/object 1.0, arm/object 0.5, table/object and ground/object 0.8).
Contact and constraint algorithms still differ between engines. RaiSim enforces
the model's joint velocity limits (15 rad/s arm, 2 rad/s fingers);
SolverMuJoCo does not implement those limits. They are preserved in the XML,
and the comparison includes this solver difference. Newton uses zero
contact gap and a stiff mimic equality (solref [0.002, 1], solimp
[0.9999, 0.99999, 0.00001, 0.5, 2]).

Use `--timing-only` to report throughput without prompting for hardware prices.
Default trials simulate 30 s per world and repeat three times after a short
untimed warmup. FPS uses the median trial time. Timed work includes control
updates, force noise, collisions, physics and asynchronous episode resets. Model
loading, compilation, rendering and the separate diagnostic replay are excluded.
The `--steps` count refers to 1 ms frames. Short `--steps` runs are functional checks and may never reach a grasp or reset.

JSON/CSV and per-case logs include raw times, torque checksums, reset counts,
maximum mimic error, object contact, bilateral grasp and lifted samples. Validation
rejects nonfinite states, Newton overflow, mimic errors above 0.01 rad and full
runs without robot/object contacts, bilateral grasps or lifts. These gates apply
to the batch: they do not require every world to grasp or lift successfully.
Grasp/lift counts are diagnostics, not successful-placement guarantees.
The overflow observable includes pipeline buffer truncation and MuJoCo contact
capacity as well as internal solver overflow. Provenance records
the pinned SDK, source and trajectory hashes, executable hash, packages and hardware.
Visual elements are removed; primitive collision dimensions, inertias and mimic
tags are retained, alongside both original BSD licenses. Capsule `length` is also
written as `height`: Newton's importer requires that spelling and otherwise
silently creates a 1 m capsule. This preserves equal arm geometry in both engines. Asset extraction
may fetch missing Git objects from the pinned sparse SDK checkout.

```sh
# CPU-only sweep; Newton packages are unnecessary.
python manipulation.py --backend raisim --worlds 1,16,64 --output results/manipulation-cpu
# Optional licensed integration test after a run prepared its assets.
MANIPULATION_CPU="$PWD/build/bin/manipulation_benchmark" \
MANIPULATION_ASSETS="$PWD/results/manipulation/assets" \
ctest --test-dir build -j12 --output-on-failure
```

The optional CUDA regression deliberately limits Newton to one contact per world
and checks that truncated contacts fail validation:

```sh
MANIPULATION_GPU_SMOKE=1 MANIPULATION_ASSETS="$PWD/results/manipulation/assets" \
python -B -m unittest discover -s tests -p test_manipulation.py -v
```

The [30-worker scaling report](reports/manipulation-30-threads-20261117.md) compares
the new default with the previous one-thread measurements.
