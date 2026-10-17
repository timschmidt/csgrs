"""Reproduce the text-consumer compiler failure on the unchanged baseline."""
import gzip, hashlib, json, os, resource, subprocess, time
from pathlib import Path

W = Path(__file__).resolve().parent
ROOT = W.parents[2]
O = W / 'circuit-baseline-v1'
SOURCE = W / 'baseline-source-v1'
TARGET = W / 'circuit-baseline-target-v1'
SCRATCH = W / 'circuit-baseline-scratch-v1'
BASE = 'b137381247a03342ce5b6184710475fcc22c9d63'

def sha(p):
    with Path(p).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()

def read(p):
    return json.loads(Path(p).read_text())

def dump(p, value):
    Path(p).write_text(json.dumps(value, indent=2) + '\n')

for p in [O, TARGET, SCRATCH]:
    p.mkdir(exist_ok=False)
pins = {str(p): sha(p) for p in [Path(__file__), W / 'circuit-contract-v1.md']}
start = time.time()
dump(O / 'wait-protocol.json', {'input_sha256': pins, 'started_epoch': start,
    'wait_limit_seconds': 7200, 'wait_for': 'candidate-validation-v4/result.json'})
while not (W / 'candidate-validation-v4/result.json').exists():
    assert time.time() - start < 7200
    time.sleep(5)
prior = read(W / 'candidate-validation-v4/result.json')
assert prior['complete'] and 'circuit-all' in prior['unexpected_exits']
assert subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip() == BASE
with gzip.open(W / 'candidate-preparation-v1/baseline-source-hashes.json.gz', 'rt') as f:
    inputs = json.load(f)
assert len(inputs) == 2832
assert all(sha(SOURCE / p) == sha(ROOT / p) == h for p, h in inputs.items())
env = os.environ.copy()
for key in ['RUSTFLAGS', 'CARGO_ENCODED_RUSTFLAGS', 'RUSTC_WRAPPER', 'RUSTC_WORKSPACE_WRAPPER', 'RUSTDOCFLAGS']:
    env.pop(key, None)
env.update({'CARGO_HOME': '/home/tim/.cargo', 'CARGO_TARGET_DIR': str(TARGET),
    'CARGO_BUILD_JOBS': '2', 'CARGO_NET_OFFLINE': 'true', 'CARGO_INCREMENTAL': '0',
    'TMPDIR': str(SCRATCH), 'CARGO_TERM_COLOR': 'never'})
command = ['cargo', 'check', '--offline', '--locked', '--release', '-p', 'hypercircuit', '--all-features']
dump(O / 'protocol.json', {'base_commit': BASE, 'input_sha256': pins,
    'source_inputs_sha256': inputs, 'source': str(SOURCE), 'target': str(TARGET),
    'target_initially_empty': not list(TARGET.iterdir()),
    'rustc': subprocess.check_output(['rustc', '-vV'], text=True),
    'cargo': subprocess.check_output(['cargo', '-V'], text=True),
    'environment': {k: env[k] for k in ['CARGO_HOME', 'CARGO_TARGET_DIR', 'CARGO_BUILD_JOBS', 'CARGO_NET_OFFLINE', 'CARGO_INCREMENTAL', 'TMPDIR']},
    'command': command, 'timeout_seconds': 1800, 'address_space_bytes': 8 * 1024**3,
    'purpose': 'Current-baseline compiler reproduction; no geometry tests or performance comparison.'})

def limits():
    resource.setrlimit(resource.RLIMIT_AS, (8 * 1024**3, 8 * 1024**3))
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))

print('START baseline Hypercircuit compile reproduction', flush=True)
begun = time.monotonic()
with (O / 'check.stdout').open('wb') as out, (O / 'check.stderr').open('wb') as err:
    result = subprocess.run(['/usr/bin/time', '-v', '-o', str(O / 'check.resources'),
        'timeout', '--kill-after=20s', '1800s', *command], cwd=SOURCE, env=env,
        stdout=out, stderr=err, preexec_fn=limits, start_new_session=True)
row = {'name': 'check', 'command': command, 'exit_code': result.returncode,
    'elapsed_seconds': time.monotonic() - begun,
    **{s + '_sha256': sha(O / ('check.' + s)) for s in ['stdout', 'stderr', 'resources']}}
dump(O / 'commands.json', [row])
diagnostic = (O / 'check.stderr').read_text()
assert result.returncode == 101
assert 'error[E0308]: mismatched types' in diagnostic and 'materialize.rs:3021:17' in diagnostic
assert 'expected `&CurveRegion2`, found `&Result<CurveRegion2, ValidationError>`' in diagnostic
assert all(sha(SOURCE / p) == sha(ROOT / p) == h for p, h in inputs.items())
assert all(sha(p) == h for p, h in pins.items())
dump(O / 'result.json', {'complete': True, 'exit_code': result.returncode,
    'baseline_compile_failure_reproduced': True, 'sources_unchanged': True,
    'main_sources_modified': False, 'tests_executed': False})
print(json.dumps({'baseline_compile_failure_reproduced': True, **row}), flush=True)
