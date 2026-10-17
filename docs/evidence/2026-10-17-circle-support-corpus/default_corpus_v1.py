"""Run the entire frozen default Hypercurve library corpus without rebuilding."""
import gzip
import hashlib
import json
import os
import resource
import shutil
import signal
import subprocess
import time
from pathlib import Path

W = Path(__file__).resolve().parent
OLD = W.parent / 'circle-support-index'
O = W / 'default-corpus-v1'

def sha(p):
    with Path(p).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()

def read(p):
    return json.loads(Path(p).read_text())

def dump(p, v):
    Path(p).write_text(json.dumps(v, indent=2) + '\n')

assert read(W / 'preparation-v1/result.json')['complete']
compiled = read(OLD / 'candidate-validation-v1/binary.json')
binary = Path(compiled['retained'])
assert sha(binary) == compiled['sha256']
assert read(OLD / 'candidate-validation-v1/result.json')['cargo_target_closed']
with gzip.open(OLD / 'candidate-preparation-v1/source-hashes.json.gz', 'rt') as f:
    old_inputs = json.load(f)
with gzip.open(W / 'preparation-v1/source-hashes.json.gz', 'rt') as f:
    current = json.load(f)
decl = '\n#[cfg(test)]\n#[path = "circle_support_cost_probe.rs"]\nmod circle_support_cost_probe;\n'
old_source = OLD / 'candidate-source-v1'
curve = 'crates/hypercurve/src/curve_intersection.rs'
assert (old_source / curve).read_text().replace(decl, '') == (W / 'candidate-source-v1' / curve).read_text()
assert all(h == current[p] for p, h in old_inputs.items() if p.startswith('crates/hypercurve/src/') and p not in [curve, 'crates/hypercurve/src/circle_support_cost_probe.rs'])
assert str(old_source).encode() not in binary.read_bytes()
O.mkdir(exist_ok=False)
runtime = O / 'runtime-source-mirror'
runtime.mkdir()
mirrored = {}
for p, h in old_inputs.items():
    if not p.startswith('crates/hypercurve/'):
        continue
    assert sha(old_source / p) == h
    dest = runtime / p
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(old_source / p, dest)
    mirrored[p] = h
scratch = O / 'scratch'
scratch.mkdir()
env = os.environ | {'TMPDIR': str(scratch), 'RUST_BACKTRACE': '0'}
pins = {str(p): sha(p) for p in [Path(__file__), binary, W / 'preparation-v1/result.json', W / 'contract-v1.md']}
command = [str(binary), '--nocapture', '--test-threads=1']
dump(O / 'protocol.json', {
    'input_sha256': pins, 'binary': compiled, 'command': command, 'cwd': str(runtime),
    'source_mirror_sha256': mirrored, 'TMPDIR': str(scratch),
    'production_and_semantic_test_source_equivalence': 'Same Hypercurve source as the final proposal; the binary additionally includes the original ignored cost diagnostic module.',
    'features': [], 'default_features': [], 'selection': 'Entire library test binary; no added skips, filtering or policy changes.',
    'existing_generated_test_environment': {k: v for k, v in env.items() if k.startswith('PROPTEST_')},
    'external_timeout_seconds': 1800, 'address_space_bytes': 8 * 1024**3, 'minimum_repository_free_bytes': 768 * 1024**2,
    'cargo_target_reopened': False, 'full_crate_or_all_features_claimed': False,
})

def limits():
    resource.setrlimit(resource.RLIMIT_AS, (8 * 1024**3, 8 * 1024**3))
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))

start = time.monotonic()
space = []
reason = None
with (O / 'tests.stdout').open('wb') as out, (O / 'tests.stderr').open('wb') as err:
    proc = subprocess.Popen(['/usr/bin/time', '-v', '-o', str(O / 'tests.resources'), *command], cwd=runtime, env=env,
                            stdout=out, stderr=err, preexec_fn=limits, start_new_session=True)
    dump(O / 'process.json', {'pid': proc.pid, 'command': command, 'supervisor': '/usr/bin/time', 'started_unix': time.time()})
    print('START complete default library corpus', flush=True)
    while proc.poll() is None:
        fs = os.statvfs(W)
        available = fs.f_bavail * fs.f_frsize
        space.append({'elapsed_seconds': time.monotonic() - start, 'available_bytes': available})
        if available < 768 * 1024**2 or time.monotonic() - start > 1800:
            reason = 'repository-space-reserve' if available < 768 * 1024**2 else 'external-timeout'
            os.killpg(proc.pid, signal.SIGTERM)
            try:
                proc.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
            break
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            pass
    code = proc.wait()
assert all(sha(runtime / p) == h for p, h in mirrored.items())
assert all(sha(p) == h for p, h in pins.items())
generated = {str(p.relative_to(runtime)): sha(p) for p in runtime.rglob('*') if p.is_file() and str(p.relative_to(runtime)) not in mirrored}
row = {'name': 'tests', 'command': command, 'exit_code': code, 'elapsed_seconds': time.monotonic() - start,
       'external_stop_reason': reason, 'free_space_observations': space,
       **{s + '_sha256': sha(O / ('tests.' + s)) for s in ['stdout', 'stderr', 'resources']}}
dump(O / 'commands.json', [row])
dump(O / 'result.json', {
    'complete': True, 'command_failures': [] if code == 0 else ['tests'], 'exit_code': code,
    'source_mirror_unchanged': True, 'generated_regression_outputs': generated,
    'binary_unchanged': True, 'default_library_only': True, 'cache_admitted': False,
})
print(json.dumps({'complete': True, 'exit_code': code, 'elapsed_seconds': row['elapsed_seconds']}), flush=True)
raise SystemExit(code)
