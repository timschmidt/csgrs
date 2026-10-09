"""Replay the public underflow probe and five changed findings, without timed work.

Usage: python3 reproduce_auxiliary_probes.py COST_OUTPUT
Run after cost construction has finished; its linked library is hash-verified.
The retained baseline library must be available at baseline-library.json's path.
"""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

record = Path(__file__).resolve().parent
cost = Path(sys.argv[1]).resolve()
load = lambda path: json.loads(path.read_bytes())
info = lambda path: dict(bytes=Path(path).stat().st_size, sha256=hashlib.sha256(Path(path).read_bytes()).hexdigest())
manifest = load(cost / 'manifest.json')
artifacts = [json.loads(line) for line in (cost / 'library-build.jsonl').read_text().splitlines()]
def rlib(name):
    return next(Path(f) for a in artifacts if a.get('reason') == 'compiler-artifact' and a['target']['name'] == name for f in a['filenames'] if f.endswith('.rlib'))

candidate = rlib('hyperdrc')
assert info(candidate) == manifest['binaries'][str(candidate)]
baseline = load(record / 'baseline-library.json')
assert info(baseline['path']) == {k: baseline[k] for k in ['bytes', 'sha256']}
dependencies = {name: rlib(name) for name in ['hyperlimit', 'hypercurve', 'serde_json']}
out = cost / 'auxiliary-probes'
out.mkdir(exist_ok=False)
records = {}
for label, source, library, output in [
    ('underflow-after', record / 'underflow-correction/probe.rs', candidate, None),
    ('row-findings-before', record / 'cost/row_findings.rs', Path(baseline['path']), out / 'before'),
    ('row-findings-after', record / 'cost/row_findings.rs', candidate, out / 'after'),
]:
    binary = out / label
    command = ['rustc', '--edition=2024', '-O', str(source), '-L', f'dependency={Path.cwd()}/target/release/deps', '--extern', f'hyperdrc={library}']
    for name, path in dependencies.items():
        command += ['--extern', f'{name}={path}']
    command += ['-o', str(binary)]
    with (out / f'{label}-build.log').open('w') as log:
        build = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT)
    run_command = [str(binary)] + ([str(output)] if output else [])
    result = None
    if build.returncode == 0:
        with (out / f'{label}.log').open('w') as log:
            result = subprocess.run(run_command, stdout=log, stderr=subprocess.STDOUT)
    records[label] = dict(command=command, build_exit_code=build.returncode,
                          command_run=run_command, exit_code=result.returncode if result else None,
                          source=info(source), binary=info(binary) if binary.exists() else None,
                          library={str(library): info(library)}, dependencies={str(p): info(p) for p in dependencies.values()})
    (out / 'manifest.json').write_text(json.dumps(records, indent=2) + '\n')
    assert build.returncode == 0 and result.returncode == 0, label
    print(label, 'passed', flush=True)
assert info(candidate) == manifest['binaries'][str(candidate)]
