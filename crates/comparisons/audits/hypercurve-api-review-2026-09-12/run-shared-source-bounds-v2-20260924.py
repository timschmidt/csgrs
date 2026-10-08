from pathlib import Path
import hashlib, json, os, shutil, subprocess, time

A = Path(__file__).resolve().parent
W = A.parent
root = Path('/tmp/hypercurve-shared-source-bounds-v2-20260924')
prefix = 'shared-source-bounds-20260924-v2'
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
subprocess.run(['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustfmt', '--edition', '2024', '--config', 'skip_children=true', '--check', 'src/bezier_offset.rs'], cwd=root/'hypercurve', check=True, timeout=60)
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
selected = [('recursive_bounds_refine_each_shared_source_once', 60), ('recursive_sign_replays_correlated_real_witness_before_deep_intervals', 60), ('recursive_projective_bounds_enclose_scaled_nested_radicals', 60), ('recursive_projective_bounds_reuse_positive_denominator_certificate', 60), ('recursive_real_witnesses_do_not_require_unused_selected_generators', 60), ('retained_real_witnesses_collapse_recursive_source_box_coefficients', 60), ('selected_fiber_points_reuse_projective_roots_without_global_promotion', 60), ('nonlinear_selected_corner_chamfers_through_retained_fixed_distance_image', 75), ('recursive_selected_radial_projective_chamfer_reenters_corner_kernel', 75), ('extended_fillet_region_classifies_both_sides_of_its_companion', 75), ('pair_native_boolean_algebraic_chord_corner_publishes_a_third_generation_fillet', 75)]
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
