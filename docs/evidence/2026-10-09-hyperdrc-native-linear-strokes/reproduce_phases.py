"""Diagnose layer and minimum-neck phases without changing the complete smoke gate.

Usage: python3 reproduce_phases.py FROZEN_CANDIDATE_DIRECTORY NEW_OUTPUT_DIRECTORY
Run serially after all other builds and measurements have exited.
"""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

frozen, out = map(Path, sys.argv[1:3])
out.mkdir(parents=True, exist_ok=False)
sources = json.loads((frozen / 'sources.json').read_text())
current = lambda: {p: hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in sources['source_hashes']}
assert sources['unchanged'] and current() == sources['source_hashes']
command = ['cargo', 'build', '--locked', '--release', '-p', 'hyperdrc', '--lib', '--message-format=json']
with (out / 'build.jsonl').open('w') as stdout, (out / 'build.log').open('w') as stderr:
    subprocess.run(command, stdout=stdout, stderr=stderr, check=True)
artifacts = [json.loads(line) for line in (out / 'build.jsonl').read_text().splitlines()]
libraries = {a['target']['name']: p for a in artifacts if a.get('reason') == 'compiler-artifact'
             for p in a['filenames'] if p.endswith('.rlib')}
probe = Path(__file__).with_name('phase_probe.rs')
binary = out / 'phase-probe'
build = ['rustc', '--edition=2024', '-O', str(probe), '-L', f'dependency={Path.cwd()}/target/release/deps']
for name in ['hyperdrc', 'serde_json']:
    build += ['--extern', f'{name}={libraries[name]}']
build += ['-o', str(binary)]
with (out / 'probe-build.log').open('w') as log:
    subprocess.run(build, stdout=log, stderr=subprocess.STDOUT, check=True)
inputs = [Path('/tmp/drc-native-rectangles-report-inputs/cparti-fpga/CPArti FPGA dev board.kicad_pcb'),
          Path('/tmp/drc-native-rectangles-report-inputs/hvp109a/HVP109A.kicad_pcb')]
run = ['taskset', '-c', '0', '/usr/bin/time', '-v', str(binary), *map(str, inputs)]
assert current() == sources['source_hashes']
with (out / 'phases.log').open('w') as stdout, (out / 'time.log').open('w') as stderr:
    code = subprocess.run(run, stdout=stdout, stderr=stderr).returncode
phases, findings = [], []
for line in (out / 'phases.log').read_text().splitlines():
    if line.startswith('phase\t'):
        _, board, layer, name, elapsed = line.split('\t')
        phases.append(dict(board=board, layer=layer, name=name, elapsed_ns=int(elapsed)))
    elif line.startswith('findings\t'):
        findings.append(json.loads(line.split('\t', 1)[1]))
info = lambda p: dict(bytes=p.stat().st_size, sha256=hashlib.sha256(p.read_bytes()).hexdigest())
result = dict(scope='One diagnostic process; wall-time breakdown only, not a matched speedup or replacement of the repository 120-second smoke and 60-second minimum-neck gates. Full board inputs and every layer/neck finding are retained.',
              library_build=command, probe_build=build, run=run, exit_code=code, phases=phases,
              finding_counts=list(map(len, findings)), sources_unchanged=current() == sources['source_hashes'],
              source_hashes=sources['source_hashes'], inputs={str(p): info(p) for p in inputs},
              libraries={name: info(Path(p)) for name, p in libraries.items()}, probe=info(probe), binary=info(binary))
(out / 'manifest.json').write_text(json.dumps(result, indent=2) + '\n')
assert result['sources_unchanged'] and code == 0 and len(findings) == 2
print(json.dumps(phases, indent=2), flush=True)
