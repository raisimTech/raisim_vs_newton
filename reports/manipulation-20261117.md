# Robotiq manipulation scaling — 17 November 2026

The Kinova/Robotiq mimic pick-and-place workload was measured at 1, 16, 64 and
256 independent worlds. RaiSim uses one CPU thread; Newton uses one CUDA batch.
All cases passed validation, including contacts, bilateral grasps, lifts, finite
states, mimic error below 0.01 rad, and zero reported overflow. Every CPU/GPU
pair had exactly equal grip-torque checksums and measured reset counts.

## Throughput

Aggregate physics steps/s, median of three trials. Each trial advances every
world for 30,000 steps at 1 ms. Controls update every ten steps. Timed work
includes control, random grip torque, collision detection, integration and resets.
Model setup, compilation, rendering and the diagnostic replay are excluded.

| Worlds | RaiSim: one CPU thread | Newton: GPU | GPU / CPU |
|---:|---:|---:|---:|
| 1 | 237,150 | 3,536 | 0.01× |
| 16 | 186,273 | 42,450 | 0.23× |
| 64 | 186,317 | 149,804 | 0.80× |
| 256 | 182,084 | 483,017 | 2.65× |

Hardware: Intel Core Ultra 7 265K and NVIDIA GeForce RTX 5070 (12,227 MiB),
NVIDIA driver 595.99.02. Linux x86-64, Clang 20.1.8, Release, C++20, `-O3
-ffp-contract=off`, generic CPU target (`BENCHMARK_X86_64_V3=OFF`). Public RaiSim
SDK 2.16.0 at `d9df2c979c9ddc44ec6b6727ff72c660688fdb62`; Newton
`c552d3653352a472dcc1f7e74e004aacac202f74`, Warp 1.19.0, MuJoCo and MuJoCo Warp
3.17.0. Complete raw trials, package source URLs, source hashes and SDK library
hashes are in [the JSON report](manipulation-20261117.json). CPU and GPU run
provenance are recorded separately. Hardware prices were not assumed; FPS/USD
is null for this timing-only run. No 1,024-world result was measured here.

## Workload

Each world contains the fixed-base six-joint Kinova arm, the Robotiq 2F-85 driver
and five passive mimic followers, a workbench, ground and one 100 g object.
Dimensions are sampled independently from 36–44 mm. The 256-world batch contains
87 boxes, 82 spheres and 87 cylinders.
Object shape stays fixed per world; yaw and pickup position (±2 mm) change on
reset. Seed 20261117 determines all random inputs. Additive gripper torque is
uniform in ±0.25 Nm, refreshed every ten steps. Random per-world phase delays of
0–1 s and reset periods of 23–26 s keep episodes out of phase. The scripted
21.5 s motion approaches, closes, lifts, transfers, releases and returns.

Both engines retain the example's joint poses, PD gains, inertias and primitive
collisions. The preparation adds a capsule `height` equal to its `length`:
Newton reads `height` and otherwise silently imports a 1 m capsule. This
compatibility conversion preserves the same capsule dimensions in both engines
and is covered by a regression test. The original BSD licenses accompany the
prepared assets. Both engines disable robot self-collision and sleeping.

Sliding friction is 1.0 for pad/object, 0.5 for arm/object, and 0.8 for
table/object and ground/object. Both use circular/elliptic friction cones.
Newton uses `SolverMuJoCo` with the Newton collision pipeline and body variants
for mixed box/sphere/cylinder batches; native MuJoCo contacts do not support that
variant configuration. Its solver uses 100 iterations, 50 line-search iterations,
128 contacts/world, `implicitfast`, tolerance 1e-8, zero contact gap,
contact solref [0.002, 1], contact solimp [0.99, 0.999, 0.001, 0.5, 2],
equality solref [0.002, 1] and equality solimp
[0.9999, 0.99999, 0.00001, 0.5, 2].

RaiSim enforces the URDF joint velocity limits (15 rad/s for the arm, 2 rad/s
for the fingers); SolverMuJoCo does not implement those limits. RaiSim uses
FP64 and hard mimic coupling; Newton uses FP32 and soft mimic equalities.
Contact manifolds and grasp trajectories differ. This measures throughput for
the authored workload and each solver's behavior; equal random inputs do not
establish equal solver work or numerical accuracy. It excludes policy inference,
observations, rewards and training.

## Physical diagnostics

Counters come from an untimed replay. Object contacts include robot/object
contacts only. Bilateral and lifted percentages are fractions of world-steps
with both pads touching, or the object more than 0.10 m above its spawn height.
These are activity diagnostics; they do not assert successful placement in every
world. The quality gates are batch-wide.

| Backend | Worlds | Max mimic error (rad) | Mean robot/object contacts | Bilateral | Lifted | Resets |
|---|---:|---:|---:|---:|---:|---:|
| newton | 1 | 0.00418 | 0.637 | 31.78% | 20.32% | 1 |
| raisim | 1 | 1.67e-16 | 0.634 | 31.66% | 20.33% | 1 |
| newton | 16 | 0.00493 | 1.768 | 34.40% | 20.43% | 16 |
| raisim | 16 | 1.67e-16 | 1.203 | 34.27% | 20.44% | 16 |
| newton | 64 | 0.00592 | 1.481 | 34.67% | 20.46% | 64 |
| raisim | 64 | 2.22e-16 | 1.138 | 34.57% | 20.46% | 64 |
| newton | 256 | 0.00884 | 1.506 | 34.28% | 20.37% | 256 |
| raisim | 256 | 2.22e-16 | 1.149 | 34.32% | 20.45% | 256 |

Newton overflow validation includes pipeline-buffer truncation and the solver's
contact-capacity limit, as well as internal MuJoCo Warp overflow. A CUDA
regression intentionally restricts the solver to one contact and verifies that
the benchmark rejects the truncated trajectory. Normal scored runs all report
zero overflow. No invalid trial was discarded from the reported medians.

## Reproduce

From the `raisim-vs-newton` repository:

```sh
cmake -S . -B build-manipulation -DCMAKE_CXX_COMPILER=clang++-20 \
  -DCMAKE_BUILD_TYPE=Release -DBENCHMARK_X86_64_V3=OFF
cmake --build build-manipulation -j12
python manipulation.py --build build-manipulation --worlds 1,16,64,256 \
  --steps 30000 --repeats 3 --timing-only --output results/manipulation-fresh
```

Use an empty output directory. A RaiSim activation key is required, and GPU
runs require CUDA. The native benchmark has no worker threads. The launcher
runs cases serially. JSON/CSV, provenance, assets and full logs accompany the
local run in `results/manipulation-20261117/`.

## Validation and portability

Clang 20 builds used 12 jobs. CTest ran with 12 threads and passed, including
licensed multi-world CPU cycles and C++/Python input parity. The separate CUDA
overflow regression passed. Claude independently reviewed the code and agreed
with the final fixes; it did not run tests or benchmarks.

Docker is not installed, so the requested Ubuntu 20.04 Clang 20 and ARM GCC 10
release-toolchain checks could not run. macOS, Windows and ARM execution were
not tested here. The C++ implementation uses portable C++20 and the public SDK;
no RaiSim implementation or Python bindings were modified.
