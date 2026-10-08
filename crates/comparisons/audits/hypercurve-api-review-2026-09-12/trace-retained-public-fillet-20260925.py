from pathlib import Path
import hashlib, json, os, signal, subprocess, time

A = Path(__file__).resolve().parent
W = A.parent
prefix = 'retained-public-fillet-20260925-v48-stack1'
assert not (A / f'{prefix}-terminal.json').exists()
target = 'hypercurve_analytic_parallel_region'
binding = json.loads((A / 'local-chord-complete-replay-20260924-v48-full-terminal.json').read_text())['binaries'][target]
manifest = json.loads((A / 'local-chord-complete-replay-20260924-v48-sources.json').read_text())
current = json.loads((A / 'local-chord-complete-replay-20260924-v50-sources.json').read_text())
root = Path('/tmp/hypercurve-local-chord-complete-replay-v48-20260924')
binary = Path(binding['path'])
def verify():
    assert hashlib.sha256(binary.read_bytes()).hexdigest() == binding['sha256']
    for name, sha in manifest.items():
        assert hashlib.sha256((root/name).read_bytes()).hexdigest() == sha, name
    for name, sha in current.items():
        assert hashlib.sha256((W/name).read_bytes()).hexdigest() == sha, name

verify()
command = ['/usr/bin/gdb', '-nx', '-nh', '--batch']
for option in ['set pagination off', 'set confirm off', 'set debuginfod enabled off',
               'set print frame-arguments none', 'set print entry-values no', 'run',
               'thread apply all bt 12', 'thread apply all bt -45', 'kill', 'quit']:
    command.extend(['-ex', option])
command.extend(['--args', str(binary), '--exact', 'retained_arc_fillet_preserves_past_center_tangent_orientation', '--test-threads=1', '--nocapture'])
start = time.monotonic()
interrupted = False
with (A / f'{prefix}.log').open('w') as out:
    process = subprocess.Popen(command, cwd=root/'hypercurve', stdout=out, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        code = process.wait(timeout=20)
    except subprocess.TimeoutExpired:
        interrupted = True
        os.killpg(process.pid, signal.SIGINT)
        try:
            code = process.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
            raise
verify()
output = (A / f'{prefix}.log').read_text()
sampled = interrupted and 'received signal SIGINT' in output and '#0' in output
report = dict(sample_obtained=sampled, command=command, returncode=code, elapsed_seconds=time.monotonic()-start,
              interrupted=interrupted, binary=binding, source_manifest='local-chord-complete-replay-20260924-v48-sources.json',
              workspace_guard_manifest='local-chord-complete-replay-20260924-v50-sources.json',
              debugger_reaped=True, all_sources_unchanged=True, diagnostic_only=True)
(A / f'{prefix}-terminal.json').write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps(report), flush=True)
print(output[-18000:])
assert sampled or (not interrupted and code == 0 and '1 passed;' in output)
