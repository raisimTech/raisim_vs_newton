# Public-SDK reproduction — 16 November 2026

The standalone benchmark was built from a clean public SDK download and run on
an Intel Core Ultra 7 265K and NVIDIA RTX 5070. No private RaiSim headers,
implementation, prebuilt build tree, or private USD file was used. A valid local
activation key was supplied through the environment and is not distributed.

**25/25 cases matched the recorded protocol checks and final action checksums.**
Every measured FPS was within **2.07%** of the [published random-actuation data](https://raisim.com/sections/BenchmarkNewton.html).
All cases passed finite-state, contact, closure-gap and GPU-overflow validation.
The configured comparison threshold was 5%; the largest observed deviation was
2.0686% (RaiSim, Strandbeest, 64 worlds). These are observed results from one full
serial sweep, not a guarantee about another machine or future run.

**Timings were not literally identical.** The public SDK downloads RaiSim 2.16.0,
while the website's reference used an earlier development binary. The public
Strandbeest asset is regenerated and numerical trajectories need not match the
historical USD bit-for-bit. Identical inputs do not mean identical CPU/GPU physics.
The final reporting/portability fixes were checked with both full CPU horizons;
state hashes and target checksums matched the scored run exactly.

## Measured values

Prices: CPU USD 339.99; GPU USD 899.99. FPS means aggregate physics steps/s.
Divide FPS and FPS/USD by ten for control transitions/s and transitions/s/USD.

| Model | Backend | Worlds | Published FPS | Measured FPS | Measured FPS/USD | Change |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| anymal | raisim | 1 | 324,490.26 | 318,839.08 | 937.79 | -1.74% |
| anymal | newton | 1 | 3,388.29 | 3,367.12 | 3.74 | -0.62% |
| anymal | raisim | 64 | 1,904,141.59 | 1,929,648.87 | 5,675.60 | +1.34% |
| anymal | newton | 64 | 133,512.04 | 132,290.97 | 146.99 | -0.91% |
| anymal | raisim | 256 | 3,581,982.25 | 3,642,172.87 | 10,712.59 | +1.68% |
| anymal | newton | 256 | 462,763.57 | 461,885.13 | 513.21 | -0.19% |
| anymal | raisim | 1,024 | 3,558,588.98 | 3,529,397.47 | 10,380.89 | -0.82% |
| anymal | newton | 1,024 | 1,436,516.51 | 1,432,101.31 | 1,591.24 | -0.31% |
| anymal | raisim | 4,096 | 3,687,225.28 | 3,704,531.86 | 10,896.00 | +0.47% |
| anymal | newton | 4,096 | 2,869,820.78 | 2,866,597.33 | 3,185.14 | -0.11% |
| anymal | newton | 8,192 | 3,476,489.74 | 3,490,869.68 | 3,878.79 | +0.41% |
| anymal | newton | 16,384 | 3,721,160.02 | 3,712,032.45 | 4,124.53 | -0.25% |
| anymal | newton | 32,768 | 3,699,929.84 | 3,691,397.53 | 4,101.60 | -0.23% |
| strandbeest | raisim | 1 | 39,786.39 | 39,791.48 | 117.04 | +0.01% |
| strandbeest | newton | 1 | 2,961.88 | 2,961.99 | 3.29 | +0.00% |
| strandbeest | raisim | 64 | 410,471.67 | 418,962.71 | 1,232.28 | +2.07% |
| strandbeest | newton | 64 | 109,868.12 | 111,676.68 | 124.09 | +1.65% |
| strandbeest | raisim | 256 | 438,328.49 | 438,654.48 | 1,290.20 | +0.07% |
| strandbeest | newton | 256 | 317,352.71 | 321,330.32 | 357.04 | +1.25% |
| strandbeest | raisim | 1,024 | 479,450.21 | 482,025.05 | 1,417.76 | +0.54% |
| strandbeest | newton | 1,024 | 535,604.47 | 535,326.13 | 594.81 | -0.05% |
| strandbeest | raisim | 4,096 | 499,843.57 | 499,451.70 | 1,469.02 | -0.08% |
| strandbeest | newton | 4,096 | 649,527.94 | 651,738.59 | 724.16 | +0.34% |
| strandbeest | newton | 8,192 | 664,085.79 | 664,239.18 | 738.05 | +0.02% |
| strandbeest | newton | 16,384 | 641,569.95 | 639,728.62 | 710.82 | -0.29% |

## Checks

- Default CMake path sparse-cloned the public SDK and downloaded its prebuilt release.
- Clean Python environment installed Newton from the pinned public archive.
- The complete published profile ran serially: ten CPU cases and fifteen GPU cases.
- Both the launcher and native program ask CPU/GPU prices; unattended prices work too.
- Eighteen contract/integration tests passed, including golden actions, invalid prices,
  native prompts, even-sample median, duplicate comparison cases and CPU-only metadata.
- Standard Python CPU-only runs passed without Newton, NumPy or USD Python packages.
- A local-SDK configuration recorded its actual commit rather than an unrelated requested pin.
- Compilation and dynamic-link audits found only this repository and the public SDK.
- Output, dependency downloads, virtual environments and activation keys are ignored by Git.

Linux Clang 20 and the RTX 5070 were exercised. Docker was unavailable, so the
Ubuntu 20.04 Clang 20 and ARM GCC 10 release containers could not be run. macOS
and Windows were not available for runtime testing; their code paths were reviewed.

The structured [reproduction report](reproduction-20261116.json) records binary hashes,
package versions and all case comparisons. Full local trial logs are in the ignored
`results/reproduction-20261116/` directory. SDK binaries and activation keys are not committed.
