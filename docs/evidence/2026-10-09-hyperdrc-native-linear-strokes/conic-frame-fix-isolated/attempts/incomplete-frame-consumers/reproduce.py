import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import time

repo = Path.cwd()
baseline = Path('/tmp/drc-linear-production-baseline-rebuild2')
work = Path('/tmp/drc-linear-conic-frame-fix-isolated')
evidence = repo / 'docs/evidence/2026-10-09-hyperdrc-native-linear-strokes/conic-frame-fix-isolated'
resume_preparation = sys.argv[1:] == ['--resume-preparation']
if not resume_preparation:
    evidence.mkdir(exist_ok=False)
    work.mkdir(exist_ok=False)
else:
    for name in ['format.log', 'status.json', 'sources.json', 'reproduce.py']:
        existing = evidence / name
        if existing.exists():
            existing.rename(evidence / ('preparation-' + name))
source = work / 'source'
target = work / 'target'
modified = 'crates/hypercurve/src/rational_bezier_general.rs'
base = json.loads((baseline / 'source.json').read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
assert base['revision'] == '4d1e32fe69c0eb5b71b9949f4b3129e961007d20'
assert all(digest(baseline / 'source' / p) == h for p, h in base['source_hashes'].items())
if not resume_preparation:
    subprocess.run(['cp', '-a', '--reflink=auto', str(baseline / 'source'), str(source)], check=True)
    subprocess.run(['cp', '-a', '--reflink=auto', str(baseline / 'target'), str(target)], check=True)
shutil.copyfile(repo / modified, source / modified)
fixtures = ['crates/hyperdrc/docs/CPArti FPGA dev board.zip', 'crates/hyperdrc/docs/HVP109A.zip']
for name in fixtures:
    destination = source / name
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(repo / name, destination)
configs = [name for name in subprocess.check_output(['git', 'ls-tree', '-r', '--name-only', base['revision']], text=True).splitlines() if Path(name).name in ['.rustfmt.toml', 'rustfmt.toml', 'rust-toolchain', 'rust-toolchain.toml']]
for name in configs:
    (source / name).parent.mkdir(parents=True, exist_ok=True)
    (source / name).write_bytes(subprocess.check_output(['git', 'show', base['revision'] + ':' + name]))
paths = sorted(set(base['source_hashes']) | set(fixtures) | set(configs))
hashes = lambda: {p: digest(source / p) for p in paths}
initial = hashes()
assert {p for p, h in base['source_hashes'].items() if initial[p] != h} == {modified}
with tarfile.open(evidence / 'source-delta.tar.gz', 'w:gz') as archive:
    archive.add(source / modified, arcname=modified)
record = dict(baseline_revision=base['revision'], only_modified_source=modified,
              source_hashes=initial, fixture_paths=fixtures, additional_baseline_configs=configs, unchanged=True)
(evidence / 'sources.json').write_text(json.dumps(record, indent=2) + '\n')
shutil.copyfile(__file__, evidence / 'reproduce.py')
environment = {**os.environ, 'CARGO_TARGET_DIR': str(target), 'CCACHE_DIR': str(work / 'ccache')}
commands = [
    ('format', ['cargo', 'fmt', '--all', '--', '--check']),
    ('rational-bezier-unit', ['cargo', 'test', '--locked', '-p', 'hypercurve', '--release', '--lib', 'rational_bezier_general::tests::', '--', '--test-threads=2']),
    ('curve-region-boolean', ['cargo', 'test', '--locked', '-p', 'hypercurve', '--release', '--test', 'hypercurve_curve_region_boolean', '--', '--test-threads=2']),
    ('curve-clippy', ['cargo', 'clippy', '--locked', '-p', 'hypercurve', '--release', '--all-targets', '--', '-D', 'warnings']),
    ('build-board-smoke', ['cargo', 'test', '--locked', '-p', 'hyperdrc', '--release', '--lib', '--no-run', '--message-format=json']),
]
status = []
def run(name, command):
    assert hashes() == initial
    started = time.monotonic()
    with (evidence / (name + '.log')).open('w') as log:
        code = subprocess.run(command, cwd=source, env=environment, stdout=log, stderr=subprocess.STDOUT).returncode
    assert hashes() == initial
    status.append(dict(name=name, command=command, exit_code=code, elapsed_seconds=time.monotonic()-started))
    (evidence / 'status.json').write_text(json.dumps(status, indent=2)+'\n')
    print(name, code, flush=True)
    if code:
        raise SystemExit(code)
for name, command in commands:
    run(name, command)
executables = []
for line in (evidence / 'build-board-smoke.log').read_text().splitlines():
    try:
        item = json.loads(line)
    except ValueError:
        continue
    if item.get('reason') == 'compiler-artifact' and item.get('target', {}).get('name') == 'hyperdrc' and item.get('executable'):
        executables.append(item['executable'])
assert len(executables) == 1, executables
run('full-board-smoke', ['/usr/bin/time', '-v', '-o', str(evidence/'full-board-smoke-time.log'), executables[0], 'zz_complex_project_performance_tests::kicad_board_completes_smoke_check_suite', '--exact', '--test-threads=1', '--nocapture'])
log = (evidence/'full-board-smoke.log').read_text()
assert 'skipping complex project fixture test' not in log
assert log.count('hyperdrc: finished emit report') == 2
assert all(digest(baseline / 'source' / p) == h for p, h in base['source_hashes'].items())
assert digest(repo / modified) == initial[modified]
print('isolated frame fix passed all gates', flush=True)
