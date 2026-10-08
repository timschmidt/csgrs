from pathlib import Path
import hashlib, json, os, shutil, subprocess, time

A = Path(__file__).resolve().parent
W = A.parent
root = Path('/tmp/hypercurve-conic-owned-contact-v1-20260924')
prefix = 'conic-owned-contact-20260924-v1'
bindings = json.loads((A/f'{prefix}-sources.json').read_text())
def verify():
    for name, sha in bindings.items():
        assert hashlib.sha256((root/name).read_bytes()).hexdigest() == sha, name
        assert hashlib.sha256((W/name).read_bytes()).hexdigest() == sha, name
verify()
env = dict(os.environ, **json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
binary = A/'conic-owned-contact-20260924-v1-libtest'
prior = json.loads((A/'conic-owned-contact-20260924-v1-terminal.json').read_text())
assert prior['all_processes_reaped']
assert hashlib.sha256(binary.read_bytes()).hexdigest() == prior['binary_sha256']
checks=[]
command=[]
prefix='conic-owned-contact-20260924-v1-remaining'
names = [line[:-6] for line in subprocess.check_output([str(binary), '--list'], text=True).splitlines() if line.endswith(': test')]
selected = [('conic_owned_endpoint_preserves_a_second_interior_contact', 60), ('conic_owned_endpoint_degree_guard_preserves_cubic_crossings', 60), ('conic_owned_endpoint_still_requires_a_pole_free_range', 60), ('nonlinear_selected_corner_chamfers_through_retained_fixed_distance_image', 75)]
cases = []
print('Candidate built; starting focused cases', flush=True)
for index, (suffix, limit) in enumerate(selected):
    matches = [name for name in names if name.rsplit('::', 1)[-1] == suffix]
    assert len(matches) == 1
    name = matches[0]
    log = A/f'{prefix}-case-{index}.log'
    start = time.monotonic()
    with log.open('w') as out:
        try:
            code = subprocess.run([str(binary), '--exact', name, '--nocapture', '--test-threads=1', '--color', 'never'], cwd=root/'hypercurve', env=env, stdout=out, stderr=subprocess.STDOUT, timeout=limit).returncode
        except subprocess.TimeoutExpired:
            code = 'timeout'
    verify()
    output = log.read_text()
    assert 'running 1 test' in output
    row = dict(name=name, returncode=code, passed=code == 0 and '1 passed;' in output, elapsed_seconds=time.monotonic()-start, limit_seconds=limit, log=log.name)
    cases.append(row)
    (A/f'{prefix}-cases.json').write_text(json.dumps(cases, indent=2)+'\n')
    print(row, output[-1400:], flush=True)
report = dict(checks=checks, command=command, cases=cases, binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(), all_sources_unchanged=True, all_processes_reaped=True)
(A/f'{prefix}-terminal.json').write_text(json.dumps(report, indent=2)+'\n')
print('Terminal; focused candidate processes reaped.', flush=True)
