"""Compare fresh physical-build CLI reports with retained complete report bytes."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import resource
import subprocess
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
OLD = E.parent / '2026-10-16-native-metadata-isolation/report-replay-v1'
WORK = Path('/dev/shm/csgrs-physical-build-requalification/report-replay-v1')
OUT = E / 'report-replay-v1'


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def dump(path, value):
    Path(path).write_text(json.dumps(value, indent=2) + '\n')


def gz(path):
    with gzip.open(path, 'rt') as stream:
        return json.load(stream)


validation = json.loads((E / 'validation-v2/result.json').read_text())
assert validation['complete']
commands = json.loads((E / 'validation-v2/commands.json').read_text())
cli = next(row for row in commands if row['name'] == 'native-cli')
assert cli['exit_code'] == 0
binary = Path('/dev/shm/csgrs-physical-build-requalification/target-v1/release/hyperdrc')
identity = cli['binaries'][str(binary)]
assert sha(binary) == identity['sha256'] == sha(identity['retained'])
old_protocol = json.loads((OLD / 'protocol.json').read_text())
assert sha(OLD / 'input-hashes.json.gz') == old_protocol['input_hashes_sha256']
inputs = gz(OLD / 'input-hashes.json.gz')
assert all(sha(path) == digest for path, digest in inputs.items())
source_inputs = gz(E / 'validation-v1/source-hashes.json.gz')
source = Path('/dev/shm/csgrs-physical-build-requalification/source-v1')
assert all(sha(source / p) == h and sha(ROOT / p) == h for p, h in source_inputs.items())
OUT.mkdir(exist_ok=False)
WORK.mkdir(exist_ok=False)
(WORK / 'tmp').mkdir()
environment = os.environ | {'TMPDIR': str(WORK / 'tmp'), 'CARGO_TERM_COLOR': 'never'}
old_rows = [row for row in json.loads((OLD / 'commands.json').read_text()) if row['name'].endswith('-baseline')]
assert len(old_rows) == 2
protocol = {'runner_sha256': sha(Path(__file__)), 'binary': str(binary), 'binary_sha256': identity['sha256'],
            'validation_sha256': sha(E / 'validation-v2/commands.json'),
            'prior_protocol_sha256': sha(OLD / 'protocol.json'),
            'input_hashes_path': str(OLD / 'input-hashes.json.gz'), 'input_hashes_sha256': old_protocol['input_hashes_sha256'],
            'environment': {'TMPDIR': str(WORK / 'tmp'), 'CARGO_TERM_COLOR': 'never'},
            'expected_cli_exit': 1, 'normalization': 'None: complete bytes and parsed JSON must match.',
            'scope': 'Fresh current-source executable, same pinned physical input paths. Prior outputs remain observations; byte equality is not an independent geometry oracle or a performance comparison.',
            'wall_limit_seconds': 1800, 'address_space_limit_bytes': 8 * 1024**3,
            'runtime_admitted': False, 'full_plan_complete': False}
dump(OUT / 'protocol.json', protocol)
rows = []


def limits():
    resource.setrlimit(resource.RLIMIT_AS, (8 * 1024**3, 8 * 1024**3))
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))


for old in old_rows:
    name = old['name'].removesuffix('-baseline')
    command = [str(binary), *old['command'][1:]]
    prior_bytes = gzip.decompress((OLD / (old['name'] + '.json.gz')).read_bytes())
    assert hashlib.sha256(prior_bytes).hexdigest() == old['stdout_sha256']
    assert Path(old['stdout_path']).read_bytes() == prior_bytes
    stdout = WORK / (name + '.json')
    stderr, resources = OUT / (name + '.stderr'), OUT / (name + '.resources')
    print('START report ' + name, flush=True)
    start = time.monotonic()
    with stdout.open('wb') as out, stderr.open('wb') as err:
        result = subprocess.run(['/usr/bin/time', '-v', '-o', str(resources),
                                 'timeout', '--signal=TERM', '--kill-after=20s', '1800s', *command],
                                cwd=source, env=environment, stdout=out, stderr=err,
                                preexec_fn=limits, start_new_session=True)
    data = stdout.read_bytes()
    with gzip.open(OUT / (name + '.json.gz'), 'wb') as stream:
        stream.write(data)
    parsed = json.loads(data)
    old_parsed = json.loads(prior_bytes)
    row = {'fixture': name, 'command': command, 'exit_code': result.returncode,
           'elapsed_seconds': time.monotonic() - start, 'stdout_path': str(stdout), 'stdout_sha256': sha(stdout),
           'stderr_sha256': sha(stderr), 'resources_sha256': sha(resources),
           'bytes': len(data), 'violation_count': parsed['violation_count'],
           'complete_bytes_equal': data == prior_bytes, 'parsed_json_equal': parsed == old_parsed,
           'finding_rows_match_count': len(parsed['violations']) == parsed['violation_count']}
    rows.append(row)
    dump(OUT / 'commands.json', rows)
    print(json.dumps(row), flush=True)
assert all(sha(path) == digest for path, digest in inputs.items())
assert sha(binary) == identity['sha256'] and sha(Path(__file__)) == protocol['runner_sha256']
assert all(sha(source / p) == h and sha(ROOT / p) == h for p, h in source_inputs.items())
passed = all(row['exit_code'] == 1 and row['complete_bytes_equal'] and row['parsed_json_equal']
             and row['finding_rows_match_count'] for row in rows)
record = {'complete': True, 'passed': passed, 'fresh_cli_processes': 2,
          'input_and_binary_bytes_unchanged': True, 'runtime_admitted': False, 'full_plan_complete': False}
dump(OUT / 'result.json', record)
print(json.dumps(record), flush=True)
if not passed:
    raise SystemExit(101)
