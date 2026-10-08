from pathlib import Path
import hashlib, json, os, signal, subprocess, time

A = Path(__file__).resolve().parent
W = A.parent
prefix = 'one-fragment-chamfer-20260924-stack2'
source_prefix = 'represented-circle-incidence-20260924-v2'
root = Path('/tmp/hypercurve-represented-circle-incidence-v2-20260924')
manifest = json.loads((A / f'{source_prefix}-sources.json').read_text())
qualification = json.loads((A / f'{source_prefix}-qualification.json').read_text())
committed = json.loads((A / f'{source_prefix}-post-commit.json').read_text())
assert qualification['all_owned_processes_reaped'] and committed['all_owned_processes_reaped']
assert subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=W/'hypercurve', text=True).strip() == committed['head']
binary = A / f'{source_prefix}-libtest'
terminal = json.loads((A / f'{source_prefix}-terminal.json').read_text())
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
                'bezier_region::tests::one_fragment_materialized_loop_chamfers_to_one_middle_interval',
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
              committed_head=committed['head'], debugger_reaped=True, all_sources_unchanged=True,
              diagnostic_only=True, qualification_timing_unchanged=True)
(A/f'{prefix}-terminal.json').write_text(json.dumps(report, indent=2)+'\n')
print('Diagnostic terminal:', code, 'interrupted:', interrupted, 'sources and executable unchanged', flush=True)
print(output[-24000:])
assert sampled, 'the debugger did not obtain the requested bounded stack sample'
