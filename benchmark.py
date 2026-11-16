#!/usr/bin/env python3
"""Run public-SDK RaiSim and Newton benchmarks serially and print FPS and FPS/USD."""
import argparse
import csv
import json
import os
import hashlib
import importlib.metadata
import platform
import re
import shutil
import time
from pathlib import Path
import subprocess
import sys

from benchmark_common import positive_price, prompt_prices
from scripts.prepare_assets import prepare


def worlds_list(text):
    values = [int(x) for x in text.split(",")]
    if not values or any(x <= 0 for x in values) or len(set(values)) != len(values):
        raise argparse.ArgumentTypeError("Use distinct positive world counts separated by commas")
    return values


def result_from_log(text):
    result = json.loads(next(x[7:] for x in text.splitlines() if x.startswith("RESULT ")))
    checksum = re.search(r"ACTION_CHECKSUM (\S+)", text)
    if checksum:
        result["last_action_checksum"] = float(checksum[1])
    return result


def run_case(command, log_path, gpu, environment):
    peak = None
    with log_path.open("w") as log:
        process = subprocess.Popen(command, text=True, stdout=log, stderr=subprocess.STDOUT, env=environment)
        while process.poll() is None:
            if gpu and shutil.which("nvidia-smi"):
                sample = subprocess.run(["nvidia-smi", "--query-compute-apps=pid,used_gpu_memory",
                                         "--format=csv,noheader,nounits"], text=True, capture_output=True)
                for line in sample.stdout.splitlines():
                    fields = [x.strip() for x in line.split(",")]
                    if len(fields) == 2 and fields[0] == str(process.pid):
                        try:
                            peak = max(peak or 0, float(fields[1]))
                        except ValueError:
                            pass
            time.sleep(.5)
    if process.returncode:
        raise RuntimeError(f"Benchmark exited with {process.returncode}; see {log_path}")
    return result_from_log(log_path.read_text()), peak


def cpu_description():
    if sys.platform.startswith('linux'):
        info = Path('/proc/cpuinfo')
        if info.exists():
            for line in info.read_text().splitlines():
                if line.startswith('model name'):
                    return line.split(':', 1)[1].strip()
    if sys.platform == 'darwin' and shutil.which('sysctl'):
        process = subprocess.run(['sysctl','-n','machdep.cpu.brand_string'],text=True,capture_output=True)
        if process.returncode == 0 and process.stdout.strip():
            return process.stdout.strip()
    return platform.processor() or platform.machine()


def gpu_description():
    if not shutil.which('nvidia-smi'):
        return None
    process = subprocess.run(['nvidia-smi','--query-gpu=name,driver_version,memory.total',
                              '--format=csv,noheader'],text=True,capture_output=True)
    return process.stdout.strip() if process.returncode == 0 else None


def runtime_environment(paths):
    env = os.environ.copy()
    if os.name == 'nt':
        prefix = Path(paths['sdk_prefix'])
        env['PATH'] = os.pathsep.join(str(prefix/p) for p in ('bin','lib','bin/openusd','lib/openusd')) + os.pathsep + env.get('PATH','')
    return env


def package_versions():
    versions = {}
    for name in ("newton", "warp-lang", "mujoco", "mujoco-warp", "numpy", "usd-core", "newton-usd-schemas"):
        try:
            versions[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            versions[name] = None
    return versions


def worker_count(worlds, requested=None):
    available = len(os.sched_getaffinity(0)) if hasattr(os, "sched_getaffinity") else (os.cpu_count() or 1)
    return min(worlds, requested if requested is not None else min(16 if worlds == 64 else 20, available))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build", type=Path, default=Path("build"))
    parser.add_argument("--config", default="Release")
    parser.add_argument("--worlds", type=worlds_list, default=[1, 64, 256, 1024, 4096])
    parser.add_argument("--extended-gpu", action="store_true", help="Add the published 8192/16384/32768 GPU-only cases")
    parser.add_argument("--max-gpu-memory-mib", type=float, default=5000.)
    parser.add_argument("--models", nargs="+", choices=["anymal", "strandbeest"], default=["anymal", "strandbeest"])
    parser.add_argument("--backend", choices=["both", "raisim", "newton"], default="both")
    parser.add_argument("--threads", type=int, help="CPU workers; default min(worlds, 20, logical CPUs)")
    parser.add_argument("--steps", type=int, help="Override 5000 ANYmal / 3000 Strandbeest physics steps")
    parser.add_argument("--repeats", type=int)
    parser.add_argument("--cpu-price", type=positive_price)
    parser.add_argument("--gpu-price", type=positive_price)
    parser.add_argument("--action-scale", type=float, default=1., help="0 = nominal targets; 1 = full random targets")
    parser.add_argument("--pin-threads", action=argparse.BooleanOptionalAction, default=sys.platform == "linux", help="Linux defaults to pinned CPU workers")
    parser.add_argument("--output", type=Path, default=Path("results"))
    args = parser.parse_args()
    if (args.repeats is not None and args.repeats < 1) or (args.threads is not None and args.threads < 1):
        parser.error("Repeats and threads must be positive")
    if args.steps is not None and (args.steps < 10 or args.steps % 10):
        parser.error("Steps must be positive and divisible by ten")
    if not 0 <= args.action_scale <= 1:
        parser.error("Action scale must be between zero and one")
    if args.extended_gpu and args.backend == "raisim":
        parser.error("--extended-gpu requires the Newton backend")
    if not args.max_gpu_memory_mib > 0:
        parser.error("GPU memory limit must be positive")
    cpu_price, gpu_price = prompt_prices(args.cpu_price, args.gpu_price)
    paths = json.loads((args.build / f"benchmark_paths_{args.config}.json").read_text())
    if args.output.exists() and any(args.output.iterdir()):
        parser.error("Output directory is not empty; choose a new --output directory")
    args.output.mkdir(parents=True, exist_ok=True)
    metadata = {"sdk_revision": paths["sdk_revision"], "platform": platform.platform(),
                "cpu_description": cpu_description(), "cpu_price_usd": cpu_price, "gpu_price_usd": gpu_price,
                "packages": package_versions(),
                "command": sys.argv, "pin_threads": args.pin_threads,
                "sdk_dirty": paths.get("sdk_dirty"), "driver_sha256": hashlib.sha256(Path(paths["cpu"]).read_bytes()).hexdigest()}
    prefix = Path(paths["sdk_prefix"])
    metadata["sdk_library_sha256"] = {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
        for p in (prefix / "lib").glob("*raisim*") if p.is_file() and not p.is_symlink()}
    metadata["sdk_version"] = next((line.split()[-1].rstrip(")") for line in
        (Path(paths["sdk"]) / "CMakeLists.txt").read_text().splitlines()
        if line.startswith("set(_raisim_expected_version ")), "unknown")
    metadata["gpu"] = gpu_description()
    (args.output / "provenance.json").write_text(json.dumps(metadata, indent=2))
    assets = args.build.resolve() / "assets"
    prepare(paths, assets)
    backends = ["raisim", "newton"] if args.backend == "both" else [args.backend]
    records = []
    print("FPS is aggregate physics steps/s. One control transition contains ten physics steps.")
    print("Model        Worlds Backend  Workers              FPS          FPS/USD")
    root = Path(__file__).resolve().parent
    for model in args.models:
        extra = ([8192, 16384, 32768] if model == "anymal" else [8192, 16384]) if args.extended_gpu else []
        gpu_stopped = False
        for n in sorted(set(args.worlds + extra)):
            for backend in backends:
                if backend == "raisim" and n not in args.worlds:
                    continue
                if backend == "newton" and gpu_stopped:
                    continue
                steps = args.steps or (5000 if model == "anymal" else 3000)
                workers = worker_count(n, args.threads)
                repeats = args.repeats or (7 if backend == "raisim" and n == 1 else 3)
                name = f"{model}-{backend}-{n}"
                output = args.output / (name + ".json")
                if backend == "raisim":
                    command = [paths["cpu"], "--model", model, "--worlds", str(n), "--threads", str(workers),
                               "--steps", str(steps), "--repeats", str(repeats), "--assets", str(assets),
                               "--cpu-price", str(cpu_price), "--gpu-price", str(gpu_price), "--action-scale", str(args.action_scale)]
                    if args.pin_threads:
                        command.append("--pin-threads")
                else:
                    command = [sys.executable, str(root / "newton_benchmark.py"), "--scenario", model,
                               "--worlds", str(n), "--steps", str(steps), "--repeats", str(repeats),
                               "--profile", "stiff", "--assets", str(assets), "--output", str(output),
                               "--cpu-price", str(cpu_price), "--gpu-price", str(gpu_price), "--action-scale", str(args.action_scale)]
                print(f"Running {model}: {n:,} worlds, {backend}, {repeats} trials...", flush=True)
                record, peak = run_case(command, args.output / (name + ".log"), backend == "newton", runtime_environment(paths))
                if backend == "newton" and peak is None:
                    print("GPU process memory was unavailable; the memory guard cannot be enforced for this case.", flush=True)
                if not record.get("quality_pass") or not record.get("finite") or record.get("overflow_bitmask", 0):
                    raise RuntimeError(f"{name} failed validation; no throughput claim is reported")
                price = cpu_price if backend == "raisim" else gpu_price
                record.update(backend=backend, model=model, dt=.002 if model == "anymal" else .001,
                              action_scale=args.action_scale, sampled_process_peak_gpu_mib=peak, cpu_price_usd=cpu_price, gpu_price_usd=gpu_price,
                              fps_per_usd=record["aggregate_fps"] / price)
                output.write_text(json.dumps(record, indent=2))
                records.append(record)
                (args.output / "results.json").write_text(json.dumps(records, indent=2))
                with (args.output / "results.csv").open("w", newline="") as stream:
                    keys = ["model", "worlds", "backend", "aggregate_fps", "fps_per_usd", "cpu_price_usd", "gpu_price_usd"]
                    writer = csv.DictWriter(stream, fieldnames=keys, extrasaction="ignore")
                    writer.writeheader()
                    writer.writerows(records)
                print(f"{model:12} {n:6,d} {backend:8} {workers if backend == 'raisim' else '-':>7} "
                      f"{record['aggregate_fps']:16,.2f} {record['fps_per_usd']:16,.2f}", flush=True)
                if backend == "newton" and peak is not None and peak > args.max_gpu_memory_mib:
                    print(f"Stopping larger {model} GPU cases after sampled peak {peak:,.0f} MiB.", flush=True)
                    gpu_stopped = True
    print(f"Saved full measurements and trial timings to {args.output.resolve()}")


if __name__ == "__main__":
    try:
        main()
    except (ValueError, RuntimeError, FileNotFoundError) as error:
        print(f"Error: {error}", file=sys.stderr)
        raise SystemExit(1)
