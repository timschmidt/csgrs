from pathlib import Path
import hashlib, json, os, signal, subprocess, time

A = Path(__file__).resolve().parent
W = A.parent
prefix = 'one-fragment-selected-chamfer-20260924-stack3'
source_prefix = 'retained-coordinate-identity-20260924-v3'
root = Path('/tmp/hypercurve-retained-coordinate-identity-v3-20260924')
manifest = json.loads((A / f'{source_prefix}-sources.json').read_text())
parent = 'c46890e2694b50d8ed5dc0baddd43023e757dc3e'
assert subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=W/'hypercurve', text=True).strip() == parent
binary = A / f'{source_prefix}-libtest'
terminal = json.loads((A / f'{source_prefix}-terminal.json').read_text())
assert terminal['all_processes_reaped'] and terminal['all_sources_unchanged']
sha = terminal['binary_sha256']
def verify():
    assert hashlib.sha256(binary.read_bytes()).hexdigest() == sha
    for name, expected in manifest.items():
        assert hashlib.sha256((W/name).read_bytes()).hexdigest() == expected, name
        assert hashlib.sha256((root/name).read_bytes()).hexdigest() == expected, name
verify()
command = ['/usr/bin/gdb', '-nx', '-nh', '--batch']
for option in ['set pagination off', 'set confirm off', 'set debuginfod enabled off',
               'set print frame-arguments none', 'set print entry-values no', 'run',
               'thread apply all bt 12', 'thread apply all bt -40', 'kill', 'quit']:
    command.extend(['-ex', option])
command.extend(['--args', str(binary), '--exact',
                'bezier_region::tests::one_fragment_selected_loop_chamfers_from_one_interval',
                '--test-threads=1', '--nocapture'])
start = time.monotonic()
interrupted = False
with (A/f'{prefix}.log').open('w') as out:
    process = subprocess.Popen(command, cwd=root/'hypercurve', stdout=out,
                               stderr=subprocess.STDOUT, start_new_session=True)
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
output = (A/f'{prefix}.log').read_text()
sampled = interrupted and 'received signal SIGINT' in output and '#0' in output
report = dict(sample_obtained=sampled, command=command, returncode=code, elapsed_seconds=time.monotonic()-start,
              interrupted=interrupted, binary_sha256=sha, source_manifest=f'{source_prefix}-sources.json',
              parent_head=parent, uncommitted_candidate=True, debugger_reaped=True, all_sources_unchanged=True,
              diagnostic_only=True, qualification_timing_unchanged=True)
(A/f'{prefix}-terminal.json').write_text(json.dumps(report, indent=2)+'\n')
print('Diagnostic terminal:', code, 'interrupted:', interrupted, 'sources and executable unchanged', flush=True)
print(output[-24000:])
assert sampled, 'the debugger did not obtain the requested bounded stack sample'
