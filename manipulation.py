#!/usr/bin/env python3
"""Run the randomized Kinova/Robotiq manipulation scaling benchmark serially."""
import argparse
import csv
import hashlib
import importlib.metadata
import json
import os
import platform
import subprocess
import sys
from pathlib import Path
from benchmark import cpu_description, gpu_description, package_versions, result_from_log, runtime_environment, worlds_list
from benchmark_common import positive_price, prompt_prices
from manipulation_common import prepare, SEED, DT, DECIMATION, reset_period, uniform


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--build',type=Path,default=Path('build'))
    p.add_argument('--config',default='Release')
    p.add_argument('--worlds',type=worlds_list,default=[1,16,64,256,1024])
    p.add_argument('--backend',choices=['both','raisim','newton'],default='both')
    p.add_argument('--threads',type=int,default=30,help='RaiSim workers, capped by world count')
    p.add_argument('--steps',type=int,default=30000)
    p.add_argument('--repeats',type=int,default=3)
    p.add_argument('--timing-only',action='store_true',help='Report FPS without asking hardware prices')
    p.add_argument('--cpu-price',type=positive_price)
    p.add_argument('--gpu-price',type=positive_price)
    p.add_argument('--output',type=Path,default=Path('results/manipulation'))
    a = p.parse_args()
    if a.steps < 10 or a.steps % 10 or a.repeats < 1 or a.threads < 1:
        p.error('Positive counts and steps divisible by ten required')
    if a.output.exists() and any(a.output.iterdir()):
        p.error('Choose an empty output directory')
    paths = json.loads((a.build/f'benchmark_paths_{a.config}.json').read_text())
    actual = subprocess.check_output(['git','-C',paths['sdk'],'rev-parse','HEAD'],text=True).strip()
    dirty = subprocess.check_output(['git','-C',paths['sdk'],'status','--porcelain','--untracked-files=no'],text=True).strip()
    if actual != paths['sdk_revision'] or dirty or paths.get('sdk_dirty'):
        p.error('SDK must be clean and match the configured revision')
    if a.timing_only and (a.cpu_price is not None or a.gpu_price is not None):
        p.error('--timing-only cannot be combined with prices')
    cpu_price, gpu_price = (None,None) if a.timing_only else prompt_prices(a.cpu_price,a.gpu_price)
    a.output.mkdir(parents=True,exist_ok=True)
    assets = a.output.resolve()/'assets'
    manifest = prepare(paths['sdk'],assets)
    cpu = Path(paths['cpu']).with_name('manipulation_benchmark'+('.exe' if os.name == 'nt' else ''))
    root = Path(__file__).resolve().parent
    provenance = dict(command=sys.argv,platform=platform.platform(),cpu=cpu_description(),gpu=gpu_description(),
                      packages=package_versions(),workload=manifest,cpu_threads_requested=a.threads,
                      cpu_affinity=sorted(os.sched_getaffinity(0)) if hasattr(os,'sched_getaffinity') else None,
                      cpu_price_usd=cpu_price,gpu_price_usd=gpu_price,
                      source_sha256={name:hashlib.sha256((root/name).read_bytes()).hexdigest() for name in
                                     ('manipulation.py','manipulation_common.py','newton_manipulation.py','src/manipulation_benchmark.cpp','benchmark.py','benchmark_common.py')})
    try:
        provenance['newton_source'] = json.loads(importlib.metadata.distribution('newton').read_text('direct_url.json') or 'null')
    except importlib.metadata.PackageNotFoundError:
        provenance['newton_source'] = None
    if a.backend != 'newton':
        provenance['cpu_binary_sha256'] = hashlib.sha256(cpu.read_bytes()).hexdigest()
    prefix = Path(paths['sdk_prefix'])
    provenance['sdk_library_sha256'] = {str(path.relative_to(prefix)):hashlib.sha256(path.read_bytes()).hexdigest()
        for directory in ('lib','bin') for path in (prefix/directory).glob('*raisim*') if path.is_file() and not path.is_symlink()}
    (a.output/'provenance.json').write_text(json.dumps(provenance,indent=2))
    records = []
    backends = ['raisim','newton'] if a.backend == 'both' else [a.backend]
    for n in a.worlds:
        for backend in backends:
            command = ([str(cpu)] if backend == 'raisim' else [sys.executable,str(root/'newton_manipulation.py')])
            command += ['--assets',str(assets),'--worlds',str(n),'--steps',str(a.steps),'--repeats',str(a.repeats)]
            if backend == 'raisim':
                command += ['--threads',str(a.threads)]
            name = f'robotiq-{backend}-{n}'
            workers = f', {min(n,a.threads)} workers' if backend == 'raisim' else ''
            print(f'Running {backend}, {n} worlds{workers}...',flush=True)
            with (a.output/(name+'.log')).open('w') as log:
                completed = subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,env=runtime_environment(paths))
            if completed.returncode:
                raise RuntimeError(f'{name} failed; see {a.output/(name+".log")}')
            record = result_from_log((a.output/(name+'.log')).read_text())
            if not record.get('quality_pass') or not record.get('finite') or record.get('overflow_bitmask',0):
                raise RuntimeError(f'{name} failed validation')
            expected_resets = sum((a.steps//10-1)//reset_period(w) for w in range(n))
            expected_force = sum(.25*(2*uniform(w,b,50)-1) for w in range(n) for b in range(a.steps//10))
            if record['resets'] != expected_resets or abs(record['force_checksum']-expected_force) > 1e-9:
                raise RuntimeError(f'{name} has mismatched resets or grip torque inputs')
            record.update(seed=SEED,dt=DT,action_decimation=DECIMATION,
                          solver_steps_per_second=record['solver_substeps']*record['aggregate_fps'],
                          cpu_price_usd=cpu_price,gpu_price_usd=gpu_price,
                          fps_per_usd=None if a.timing_only else record['aggregate_fps']/(cpu_price if backend == 'raisim' else gpu_price))
            records.append(record)
            (a.output/(name+'.json')).write_text(json.dumps(record,indent=2))
            (a.output/'results.json').write_text(json.dumps(records,indent=2))
            with (a.output/'results.csv').open('w',newline='') as stream:
                writer = csv.DictWriter(stream,fieldnames=['backend','worlds','threads','aggregate_fps','fps_per_usd'],extrasaction='ignore')
                writer.writeheader(); writer.writerows(records)
            cost = '' if a.timing_only else f"; {record['fps_per_usd']:,.2f} FPS/USD"
            print(f"{backend:8} {n:5d} worlds: {record['aggregate_fps']:,.0f} FPS{cost}",flush=True)


if __name__ == '__main__':
    main()
