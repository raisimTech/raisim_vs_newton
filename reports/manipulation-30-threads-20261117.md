# Robotiq manipulation: 30 CPU workers

The earlier RaiSim results used one CPU thread. The updated benchmark defaults
to 30 persistent workers, capped only by the number of worlds. Each worker owns
a disjoint set of worlds, and all workers meet at trial boundaries. Thread
creation is outside timing. The authored simulation and random inputs are
unchanged. Each individual RaiSim world is advanced by exactly one worker.

## Measured throughput

Aggregate physics steps/s, median of three 30,000-step trials, at 1 ms per step.
The CPU runs below are new measurements. Newton values are reused from the
unchanged GPU implementation in [the original report](manipulation-20261117.md);
no GPU result is claimed for 30 or 1,024 worlds.

| Worlds | Active CPU workers | RaiSim now | Previous RaiSim, 1 thread | Newton, RTX 5070 |
|---:|---:|---:|---:|---:|
| 1 | 1 | 237,094 | 237,150 | 3,536 |
| 16 | 16 | 2,123,620 | 186,273 | 42,450 |
| 30 | 30 | 2,659,860 | — | — |
| 64 | 30 | 2,528,289 | 186,317 | 149,804 |
| 256 | 30 | 2,812,873 | 182,084 | 483,017 |
| 1,024 | 30 | 2,726,698 | — | — |

Hardware: Intel Core Ultra 7 265K; the recorded CPU affinity contains
20 logical CPUs. The requested 30 workers are retained, so the larger
cases oversubscribe this machine. Workers are not pinned. Worlds are statically
partitioned, so unequal world counts per worker, object-shape costs and mixed
CPU core speeds can affect utilization. Shape randomization also means the
one-world sphere case and the larger mixed-shape batches have different contact
loads. World count therefore changes both the shape mix and scheduling.

The six- and 31-world integration tests check capping below 30 and an uneven
31-world partition. The 31-world test compares one worker with 30 workers over
two timed trials and a diagnostic replay, requiring exactly matching contacts,
bilateral grasps, lifts, resets, force checksums and maximum mimic error. It also
checks even-trial median arithmetic. Thread counts of zero are rejected by both
CLIs. All tests passed. For each world count shared with the previous report,
the new diagnostics also match that report exactly.

## Run

From `raisim-vs-newton`, after building:

```sh
.venv/bin/python manipulation.py --build build-manipulation \
  --backend raisim --threads 30 --worlds 1,16,30,64,256,1024 \
  --steps 30000 --repeats 3 --timing-only --output results/manipulation-30-new
```

Use `--backend both` to run both engines. `--threads` changes only RaiSim.
`threads` in the result is the active count; `requested_threads` records the
requested count. CPU affinity and requested workers are recorded in provenance,
and the CSV includes the active worker count.

Raw timings, diagnostics and hashes are in
[the JSON report](manipulation-30-threads-20261117.json), with full logs and assets
in `results/manipulation-30-threads-20261117/`. The original one-thread report is
retained as a historical baseline. Its documented solver differences, including
Newton's missing joint velocity limits, remain applicable.

The code built with Clang 20 using 12 build jobs. CTest passed with 12 threads,
including the licensed serial/parallel diagnostic parity regression. Claude
independently checked the pool synchronization, exception handling, world
ownership and trial timing and agreed with the approach; it ran no tests or
benchmarks. Docker is unavailable, so the requested Ubuntu 20.04 Clang 20 and ARM
GCC 10 release-toolchain checks could not run. No macOS, Windows or ARM execution
was performed.
