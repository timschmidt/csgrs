"""Trace changed native geometry against both retained libraries after report replay.

Usage: python3 reproduce_report_geometry.py COST_OUTPUT REPORT_OUTPUT
"""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

record = Path(__file__).resolve().parent
cost, reports = [Path(s).resolve() for s in sys.argv[1:3]]
load = lambda path: json.loads(path.read_bytes())
info = lambda path: dict(bytes=Path(path).stat().st_size, sha256=hashlib.sha256(Path(path).read_bytes()).hexdigest())
manifest = load(cost / 'manifest.json')
artifacts = [json.loads(line) for line in (cost / 'library-build.jsonl').read_text().splitlines()]
def rlib(name):
    return next(Path(f) for a in artifacts if a.get('reason') == 'compiler-artifact' and a['target']['name'] == name for f in a['filenames'] if f.endswith('.rlib'))
baseline = load(record / 'baseline-library.json')
libraries = {'before': Path(baseline['path']), 'after': rlib('hyperdrc')}
assert info(libraries['before']) == {k:baseline[k] for k in ['bytes','sha256']}
assert info(libraries['after']) == manifest['binaries'][str(libraries['after'])]
dependencies = {name: rlib(name) for name in ['hyperlimit','serde_json']}
out = reports / 'geometry-probe'
out.mkdir(exist_ok=False)
source = record / 'report_geometry_probe.rs'
inputs = Path(load(record / 'baseline-reports/manifest.json')['command'][-2])
board = inputs / 'hvp109a/HVP109A.kicad_pcb'
records = {}
for label, library in libraries.items():
    binary = out / label
    command = ['rustc','--edition=2024','-O',str(source),'-L',f'dependency={Path.cwd()}/target/release/deps','--extern',f'hyperdrc={library}']
    for name, path in dependencies.items():
        command += ['--extern',f'{name}={path}']
    command += ['-o',str(binary)]
    with (out / f'{label}-build.log').open('w') as log:
        build = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT)
    result = None
    run_command = ['taskset','-c','0',str(binary),str(board)]
    if build.returncode == 0:
        with (out / f'{label}.json').open('w') as output, (out / f'{label}.log').open('w') as log:
            result = subprocess.run(run_command, stdout=output, stderr=log)
    records[label] = dict(command=command, build_exit_code=build.returncode, command_run=run_command,
                          exit_code=result.returncode if result else None,
                          source=info(source), board=info(board),
                          binary=info(binary) if binary.exists() else None,
                          library={str(library): info(library)}, dependencies={str(p):info(p) for p in dependencies.values()})
    (out / 'manifest.json').write_text(json.dumps(records,indent=2)+'\n')
    assert build.returncode == 0 and result.returncode == 0, label
    print(label, 'native geometry probe passed', flush=True)
