from pathlib import Path
import hashlib, json, os, signal, subprocess, sys, time

A = Path(__file__).resolve().parent
W = A.parent
version = sys.argv[1]
assert version.startswith('v') and version[1:].isdigit()
case = sys.argv[2] if len(sys.argv) > 2 else 'chamfer'
cases = {
    'chamfer': ('stack1', 'one_fragment_nonzero_parallel_loop_extends_chamfer_cuts_on_one_finite_envelope'),
    'fillet': ('stack2', 'nonrepresented_chord_parallel_corner_fillets_without_reintersection'),
    'endpoint': ('stack3', 'nonlinear_algebraic_endpoint_fillet_uses_complete_incident_domain'),
}
label, suffix = cases[case]
prefix = f'local-chord-parallel-roots-20260924-{version}-{label}'
if len(sys.argv) > 3:
    assert sys.argv[3] == 'approved'
    prefix += '-approved'
assert not (A / f'{prefix}-terminal.json').exists()
source_prefix = f'local-chord-parallel-roots-20260924-{version}'
root = Path(f'/tmp/hypercurve-local-chord-parallel-roots-{version}-20260924')
manifest = json.loads((A / f'{source_prefix}-sources.json').read_text())
base_head = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=W/'hypercurve', text=True).strip()
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
                f'bezier_region::tests::{suffix}',
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
              base_head=base_head, debugger_reaped=True, all_sources_unchanged=True,
              diagnostic_only=True, qualification_timing_unchanged=True)
(A/f'{prefix}-terminal.json').write_text(json.dumps(report, indent=2)+'\n')
print('Diagnostic terminal:', code, 'interrupted:', interrupted, 'sources and executable unchanged', flush=True)
print(output[-24000:])
assert sampled or (not interrupted and code == 0 and '1 passed;' in output), 'neither a completed case nor a bounded stack sample was obtained'
