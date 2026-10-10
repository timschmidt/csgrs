"""Run selected checks sequentially against a frozen diagnostic candidate."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
CANDIDATE = Path(sys.argv[1]).resolve()
PHASE = sys.argv[2]
OUT = CANDIDATE / PHASE
OUT.mkdir()
record = json.loads((CANDIDATE / 'sources.json').read_text())
SOURCE = Path(record['source_directory'])
TARGET = Path(record['target_directory'])
digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
info = lambda p: {'bytes': p.stat().st_size, 'sha256': digest(p)}
maps = []
for kind, directory in [('source', SOURCE), ('main', ROOT)]:
    path = Path(record[kind + '_hashes_file'])
    assert digest(path) == record[kind + '_hashes_file_sha256']
    maps.append((directory, json.loads(gzip.decompress(path.read_bytes()))))
shutil.copyfile(__file__, OUT / 'reproduce.py')
script_hash = digest(OUT / 'reproduce.py')
env = os.environ | {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache', 'CARGO_NET_OFFLINE': 'true', 'CARGO_TARGET_DIR': str(TARGET), 'RUSTDOCFLAGS': '-D warnings'}
commands = []


def verify():
    assert all(digest(directory / name) == expected for directory, values in maps for name, expected in values.items())
    assert digest(OUT / 'reproduce.py') == script_hash


def run(name, command, seconds=1800):
    verify()
    print('start', name, flush=True)
    start = time.monotonic()
    with (OUT / (name + '.stdout')).open('w') as stdout, (OUT / (name + '.stderr')).open('w') as stderr:
        result = subprocess.run(['timeout', '--signal=TERM', '--kill-after=10s', str(seconds), *command], cwd=SOURCE, env=env, stdout=stdout, stderr=stderr, check=False)
    row = {'name': name, 'command': command, 'exit_code': result.returncode, 'wall_seconds': time.monotonic() - start, 'timeout_seconds': seconds}
    summaries = re.findall(r'test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;', (OUT / (name + '.stdout')).read_text())
    if summaries:
        row['tests_passed'] = sum(int(x[0]) for x in summaries)
        row['tests_ignored'] = sum(int(x[2]) for x in summaries)
    commands.append(row)
    (OUT / 'commands.json').write_text(json.dumps(commands, indent=2) + '\n')
    verify()
    print('finish', name, result.returncode, flush=True)
    assert result.returncode == 0, name


if PHASE == 'focused':
    run('format', ['cargo', 'fmt', '--all', '--', '--check'])
    run('scalar-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hyperreal', '--test', 'checked_decimal', '--test', 'checked_scientific', '--test', 'gmp_api_coverage', '--', '--test-threads=2'])
    run('calculator', ['cargo', 'test', '--locked', '--release', '-p', 'hyperreal', '--features', 'simple', '--test', 'calculator_diagnostics'])
    run('lattice-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hyperlattice', '--test', 'diagnostic_display'])
    run('drc-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hyperdrc', '--test', 'diagnostic_formatting'])
    run('production-library', ['cargo', 'build', '--locked', '--release', '-p', 'hyperdrc', '--lib'])
elif PHASE == 'full':
    run('scalar-default', ['cargo', 'test', '--locked', '--release', '-p', 'hyperreal', '--lib', '--tests', '--', '--test-threads=2'])
    run('scalar-all-features', ['cargo', 'test', '--locked', '--release', '-p', 'hyperreal', '--all-features', '--lib', '--tests', '--', '--test-threads=2'])
    run('lattice-all-features', ['cargo', 'test', '--locked', '--release', '-p', 'hyperlattice', '--all-features', '--lib', '--tests', '--', '--test-threads=2'])
    run('drc-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hyperdrc', '--lib', '--tests', '--', '--test-threads=2'])
    for package in ['hyperreal', 'hyperlattice', 'hyperdrc']:
        run(package + '-clippy', ['cargo', 'clippy', '--locked', '--release', '-p', package, '--all-features', '--all-targets', '--', '-D', 'warnings'])
        run(package + '-doc-tests', ['cargo', 'test', '--locked', '--release', '-p', package, '--all-features', '--doc'])
        run(package + '-docs', ['cargo', 'doc', '--locked', '--release', '-p', package, '--all-features', '--lib', '--no-deps'])
    run('format', ['cargo', 'fmt', '--all', '--', '--check'])
    run('production-library', ['cargo', 'build', '--locked', '--release', '-p', 'hyperdrc', '--lib'])
else:
    raise ValueError(PHASE)
verify()
summary = {'source_record': str(CANDIDATE / 'sources.json'), 'source_record_sha256': digest(CANDIDATE / 'sources.json'), 'all_source_and_main_hashes_unchanged': True, 'commands': commands, 'libraries': {str(TARGET / 'release/libhyperdrc.rlib'): info(TARGET / 'release/libhyperdrc.rlib')}, 'scope': PHASE + ' checks for checked adaptive decimal formatting and diagnostic consumers; not the full repository pre-push workflow matrix.'}
summary['artifacts'] = {str(p.relative_to(OUT)): info(p) for p in sorted(OUT.rglob('*')) if p.is_file()}
(OUT / 'validation.json').write_text(json.dumps(summary, indent=2) + '\n')
print('Diagnostic validation complete:', PHASE, flush=True)
