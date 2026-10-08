from pathlib import Path
import hashlib, json, os, shutil, subprocess, time

A = Path(__file__).resolve().parent
W = A.parent
root = Path('/tmp/hypercurve-conic-owned-contact-v3-20260924')
prefix = 'conic-owned-contact-20260924-v3'
bindings = json.loads((A/f'{prefix}-sources.json').read_text())
def verify():
    for name, sha in bindings.items():
        assert hashlib.sha256((root/name).read_bytes()).hexdigest() == sha, name
        assert hashlib.sha256((W/name).read_bytes()).hexdigest() == sha, name
verify()
env = dict(os.environ, **json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
cargo = '/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
os.utime(root/'hypercurve/src/lib.rs', None)
checks = []
for features in ['--all-features', '--no-default-features']:
    command = [cargo, 'clippy', '--all-targets', features, '--locked', '--offline', '--', '-D', 'warnings']
    log = A/f'{prefix}-check-{len(checks)}.log'
    with log.open('w') as out:
        code = subprocess.run(command, cwd=root/'hypercurve', env=env, stdout=out, stderr=subprocess.STDOUT, timeout=1200).returncode
    verify()
    checks.append(dict(command=command, returncode=code, log=log.name))
    if code:
        (A/f'{prefix}-terminal.json').write_text(json.dumps(dict(checks=checks, tests_executed=0, all_sources_unchanged=True, all_processes_reaped=True), indent=2)+'\n')
        raise SystemExit('Clippy failed: '+log.name)
    print('Clippy passed:', features, flush=True)
command = [cargo, 'test', '--lib', '--release', '--all-features', '--no-run', '--message-format=json', '--locked', '--offline']
with (A/f'{prefix}-build.jsonl').open('w') as out, (A/f'{prefix}-build.log').open('w') as err:
    code = subprocess.run(command, cwd=root/'hypercurve', env=env, stdout=out, stderr=err, timeout=1200).returncode
verify()
assert code == 0, (A/f'{prefix}-build.log').read_text()[-3000:]
rows = [json.loads(line) for line in (A/f'{prefix}-build.jsonl').read_text().splitlines()]
artifact = next(row for row in rows if row.get('reason') == 'compiler-artifact' and row['target']['name'] == 'hypercurve' and row.get('executable'))
assert not artifact['fresh']
binary = A/f'{prefix}-libtest'
shutil.copy2(artifact['executable'], binary)
names = [line[:-6] for line in subprocess.check_output([str(binary), '--list'], text=True).splitlines() if line.endswith(': test')]
selected = [('conic_owned_endpoint_reuses_local_evidence_on_finite_ranges', 60), ('conic_owned_endpoint_preserves_a_second_interior_contact', 60), ('conic_owned_endpoint_degree_guard_preserves_cubic_crossings', 60), ('conic_owned_endpoint_still_requires_a_pole_free_range', 60), ('chord_rational_contact_ownership_preserves_selected_fiber_parameters', 60), ('selected_fiber_points_reuse_projective_roots_without_global_promotion', 60), ('monotone_parallel_ranges_remove_only_proven_unary_pairs', 60), ('nonlinear_selected_corner_chamfers_through_retained_fixed_distance_image', 75), ('extended_fillet_region_classifies_both_sides_of_its_companion', 75)]
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
