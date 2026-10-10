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


run('format', ['cargo', 'fmt', '--all', '--', '--check'])
run('scalar-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hyperreal', '--all-features', '--test', 'checked_decimal', '--test', 'checked_scientific', '--test', 'gmp_api_coverage', '--test', 'calculator_diagnostics', '--', '--test-threads=2'])
run('lattice-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hyperlattice', '--test', 'diagnostic_display'])
run('drc-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hyperdrc', '--test', 'diagnostic_formatting', '--test', 'kicad_rule_export', '--test', 'native_linear_strokes', '--', '--test-threads=2'])
run('drc-rule-unit-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hyperdrc', '--lib', 'kicad_dru::', '--', '--test-threads=2'])
run('clippy', ['cargo', 'clippy', '--locked', '--release', '-p', 'hyperreal', '-p', 'hyperlattice', '-p', 'hyperdrc', '--all-features', '--all-targets', '--', '-D', 'warnings'])
run('production-library', ['cargo', 'build', '--locked', '--release', '-p', 'hyperdrc', '--lib'])

verify()
summary = {'source_record': str(CANDIDATE / 'sources.json'), 'source_record_sha256': digest(CANDIDATE / 'sources.json'), 'all_source_and_main_hashes_unchanged': True, 'commands': commands, 'libraries': {str(TARGET / 'release/libhyperdrc.rlib'): info(TARGET / 'release/libhyperdrc.rlib')}, 'scope': 'Scoped native-prototype compatibility checks. The earlier native full-board performance gate remains open; no prototype admission or full repository pre-push validation.'}
summary['artifacts'] = {str(p.relative_to(OUT)): info(p) for p in sorted(OUT.rglob('*')) if p.is_file()}
(OUT / 'validation.json').write_text(json.dumps(summary, indent=2) + '\n')
print('Diagnostic validation complete:', PHASE, flush=True)
