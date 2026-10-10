"""Validate one frozen rule-export candidate; stop and retain any failure."""
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
OUT = Path(sys.argv[1]).resolve()
record = json.loads((OUT / 'sources.json').read_text())
SOURCE = Path(record['source_directory'])
TARGET = Path(record['target_directory'])
digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
maps = []
for kind, directory in [('source', SOURCE), ('main', ROOT)]:
    path = Path(record[kind + '_hashes_file'])
    assert digest(path) == record[kind + '_hashes_file_sha256']
    maps.append((directory, json.loads(gzip.decompress(path.read_bytes()))))
shutil.copyfile(__file__, OUT / 'reproduce-validation.py')
script_hash = digest(OUT / 'reproduce-validation.py')
env = os.environ | {
    'CARGO_HOME': '/tmp/csgrs-properties-cargo',
    'CCACHE_DIR': '/tmp/csgrs-properties-ccache',
    'CARGO_NET_OFFLINE': 'true',
    'CARGO_TARGET_DIR': str(TARGET),
    'RUSTDOCFLAGS': '-D warnings',
}
commands = []


def verify():
    for directory, values in maps:
        assert all(digest(directory / name) == expected for name, expected in values.items())
    assert digest(OUT / 'reproduce-validation.py') == script_hash


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
run('focused-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hyperdrc', '--test', 'kicad_rule_export', '--', '--test-threads=2'])
run('clippy', ['cargo', 'clippy', '--locked', '--release', '-p', 'hyperdrc', '--all-features', '--all-targets', '--', '-D', 'warnings'])
run('library-and-integration-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hyperdrc', '--lib', '--tests', '--', '--test-threads=2'])
run('doc-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hyperdrc', '--doc'])
run('docs', ['cargo', 'doc', '--locked', '--release', '-p', 'hyperdrc', '--all-features', '--lib', '--no-deps'])
run('production-build', ['cargo', 'build', '--locked', '--release', '-p', 'hyperdrc', '--lib', '--bin', 'hyperdrc'])
verify()
info = lambda p: {'bytes': p.stat().st_size, 'sha256': digest(p)}
summary = {'source_record': str(OUT / 'sources.json'), 'source_record_sha256': digest(OUT / 'sources.json'), 'all_source_and_main_hashes_unchanged': True, 'commands': commands, 'libraries': {str(TARGET / 'release/libhyperdrc.rlib'): info(TARGET / 'release/libhyperdrc.rlib')}, 'binary': {str(TARGET / 'release/hyperdrc'): info(TARGET / 'release/hyperdrc')}, 'scope': 'HyperDRC focused, library, integration, doc, strict Clippy and workspace formatting checks. Not the full workspace pre-push workflow matrix.'}
summary['artifacts'] = {str(p.relative_to(OUT)): info(p) for p in sorted(OUT.rglob('*')) if p.is_file()}
(OUT / 'validation.json').write_text(json.dumps(summary, indent=2) + '\n')
print('Candidate validation complete.', flush=True)
