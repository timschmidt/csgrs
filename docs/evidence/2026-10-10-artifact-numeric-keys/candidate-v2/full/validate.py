"""Serial validation for one frozen source candidate; no concurrent builds."""
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
candidate, phase = Path(sys.argv[1]).resolve(), sys.argv[2]
out = candidate / phase
out.mkdir()
record = json.loads((candidate / 'sources.json').read_text())
source, target = Path(record['source_directory']), Path(record['target_directory'])
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
info = lambda p: {'bytes': p.stat().st_size, 'sha256': sha(p)}
guards = []
for kind, base in [('source', source), ('main', ROOT)]:
    path = Path(record[kind + '_hashes_file'])
    assert sha(path) == record[kind + '_hashes_file_sha256']
    guards.append((base, json.loads(gzip.decompress(path.read_bytes()))))
shutil.copyfile(__file__, out / 'validate.py')
script_hash = sha(out / 'validate.py')
env = os.environ | {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache', 'CARGO_NET_OFFLINE': 'true', 'CARGO_TARGET_DIR': str(target), 'RUSTDOCFLAGS': '-D warnings'}
commands = []


def verify():
    assert sha(out / 'validate.py') == script_hash
    assert all(sha(base / name) == value for base, values in guards for name, value in values.items())


def run(name, command):
    verify()
    print('start', name, flush=True)
    start = time.monotonic()
    with (out / (name + '.stdout')).open('w') as stdout, (out / (name + '.stderr')).open('w') as stderr:
        result = subprocess.run(['timeout', '--signal=TERM', '--kill-after=10s', '1800', *command], cwd=source, env=env, stdout=stdout, stderr=stderr, check=False)
    totals = re.findall(r'test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;', (out / (name + '.stdout')).read_text())
    commands.append({'name': name, 'command': command, 'exit_code': result.returncode, 'wall_seconds': time.monotonic() - start, 'timeout_seconds': 1800, 'passed': sum(int(row[0]) for row in totals), 'ignored': sum(int(row[2]) for row in totals)})
    (out / 'commands.json').write_text(json.dumps(commands, indent=2) + '\n')
    verify()
    print('finish', name, result.returncode, flush=True)
    assert result.returncode == 0, name


run('format', ['cargo', 'fmt', '--all', '--', '--check'])
if phase == 'focused':
    run('regressions', ['cargo', 'test', '--locked', '--release', '-p', 'hyperdrc', '--test', 'artifact_numeric_keys'])
    run('artifact-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hyperdrc', '--lib', 'checks::artifacts::tests::', '--', '--test-threads=2'])
elif phase == 'full':
    run('strict-clippy', ['cargo', 'clippy', '--locked', '--release', '-p', 'hyperdrc', '--all-features', '--all-targets', '--', '-D', 'warnings'])
    run('drc-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hyperdrc', '--all-features', '--lib', '--tests', '--', '--test-threads=2'])
    run('doc-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hyperdrc', '--all-features', '--doc'])
    run('strict-docs', ['cargo', 'doc', '--locked', '--release', '-p', 'hyperdrc', '--all-features', '--lib', '--no-deps'])
else:
    assert phase == 'native'
    run('regressions', ['cargo', 'test', '--locked', '--release', '-p', 'hyperdrc', '--test', 'artifact_numeric_keys', '--test', 'native_linear_strokes', '--test', 'diagnostic_formatting'])
    run('strict-clippy', ['cargo', 'clippy', '--locked', '--release', '-p', 'hyperdrc', '--all-features', '--all-targets', '--', '-D', 'warnings'])
run('production-library', ['cargo', 'build', '--locked', '--release', '-p', 'hyperdrc', '--lib'])
summary = {'source_record': str(candidate / 'sources.json'), 'source_record_sha256': sha(candidate / 'sources.json'), 'guards_unchanged': True, 'commands': commands, 'libraries': {str(target / 'release/libhyperdrc.rlib'): info(target / 'release/libhyperdrc.rlib')}, 'scope': 'Scoped checks; not the repository-wide pre-push workflow matrix.'}
summary['artifacts'] = {str(p.relative_to(out)): info(p) for p in sorted(out.iterdir()) if p.is_file()}
(out / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
print('Validation complete:', phase, flush=True)
