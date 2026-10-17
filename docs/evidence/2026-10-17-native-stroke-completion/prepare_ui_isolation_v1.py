"""Verify Cargo workspace isolation without changing any qualified input."""
import gzip, hashlib, json, os, resource, shutil, subprocess, time
from pathlib import Path

W = Path(__file__).resolve().parent
ROOT = W.parents[2]
O = W / 'ui-isolation-v1'
SOURCE = Path('/tmp/csgrs-native-stroke-ui-source-v1')
RELATIVE = Path('crates/hypercurve/examples/hypercurve_ui')

def sha(p):
    with Path(p).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()

def read(p):
    return json.loads(Path(p).read_text())

def dump(p, value):
    Path(p).write_text(json.dumps(value, indent=2) + '\n')

assert read(W / 'ui-validation-v1/result.json')['unexpected_exits'] == ['ui-tests', 'ui-clippy', 'ui-wasm']
assert read(W / 'snapshot-requalification-v1/result.json')['complete']
assert 'workspace: ' + str(ROOT / 'Cargo.toml') in (W / 'ui-validation-v1/ui-tests.stderr').read_text()
with gzip.open(W / 'candidate-preparation-v6/source-hashes.json.gz', 'rt') as f:
    inputs = json.load(f)
assert len(inputs) == 2835
assert all(sha(W / 'candidate-source-v6' / p) == h for p, h in inputs.items())
assert sha(ROOT / RELATIVE / 'Cargo.toml') == inputs[str(RELATIVE / 'Cargo.toml')]
assert sha(ROOT / RELATIVE / 'Cargo.lock') == inputs[str(RELATIVE / 'Cargo.lock')]
O.mkdir(exist_ok=False)
SOURCE.mkdir(exist_ok=False)
env = os.environ.copy()
env.update({'CARGO_HOME': '/home/tim/.cargo', 'CARGO_NET_OFFLINE': 'true',
    'CARGO_TARGET_DIR': str(W / 'ui-metadata-target-v1'), 'CARGO_TERM_COLOR': 'never'})
pins = {str(p): sha(p) for p in [Path(__file__), W / 'ui-isolation-contract-v1.md',
    W / 'candidate-preparation-v6/source-hashes.json.gz', W / 'ui-validation-v1/result.json']}
dump(O / 'protocol.json', {'input_sha256': pins, 'source': str(SOURCE),
    'timeout_seconds': 1800, 'address_space_bytes': 8 * 1024**3,
    'production_source_changes': False, 'completed_target_reopened': False})
commands = []

def limits():
    resource.setrlimit(resource.RLIMIT_AS, (8 * 1024**3, 8 * 1024**3))
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))

def metadata(name, source):
    command = ['cargo', 'metadata', '--offline', '--locked', '--no-deps',
        '--format-version', '1', '--manifest-path', str(source / RELATIVE / 'Cargo.toml')]
    begun = time.monotonic()
    with (O / (name + '.stdout')).open('wb') as out, (O / (name + '.stderr')).open('wb') as err:
        result = subprocess.run(['/usr/bin/time', '-v', '-o', str(O / (name + '.resources')),
            'timeout', '--kill-after=20s', '1800s', *command], cwd=source, env=env,
            stdout=out, stderr=err, preexec_fn=limits, start_new_session=True)
    row = {'name': name, 'command': command, 'exit_code': result.returncode,
        'elapsed_seconds': time.monotonic() - begun,
        **{s + '_sha256': sha(O / (name + '.' + s)) for s in ['stdout', 'stderr', 'resources']}}
    commands.append(row)
    dump(O / 'commands.json', commands)
    assert result.returncode == 0, row
    data = read(O / (name + '.stdout'))
    assert Path(data['workspace_root']) == source / RELATIVE, data['workspace_root']
    return data

baseline = metadata('live-demo-metadata', ROOT)
for p, h in inputs.items():
    dest = SOURCE / p
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(W / 'candidate-source-v6' / p, dest)
    assert sha(dest) == h
isolated = metadata('isolated-demo-metadata', SOURCE)
assert all(sha(SOURCE / p) == h for p, h in inputs.items())
assert all(sha(p) == h for p, h in pins.items())
assert sha(ROOT / RELATIVE / 'Cargo.toml') == inputs[str(RELATIVE / 'Cargo.toml')]
assert sha(ROOT / RELATIVE / 'Cargo.lock') == inputs[str(RELATIVE / 'Cargo.lock')]
dump(O / 'result.json', {'complete': True, 'source': str(SOURCE), 'source_inputs': len(inputs),
    'source_hashes_sha256': sha(W / 'candidate-preparation-v6/source-hashes.json.gz'),
    'live_demo_workspace_root': baseline['workspace_root'],
    'isolated_demo_workspace_root': isolated['workspace_root'],
    'all_final_v6_inputs_byte_identical': True, 'manifest_or_lock_changes': False,
    'main_sources_modified': False, 'original_failed_validation_preserved': True})
print(json.dumps({'complete': True, 'source_inputs': len(inputs), 'source': str(SOURCE)}), flush=True)
