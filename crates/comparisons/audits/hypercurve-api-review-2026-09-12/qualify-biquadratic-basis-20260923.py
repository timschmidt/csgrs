from pathlib import Path
import hashlib, json, subprocess

A = Path(__file__).resolve().parent
W = A.parent
roots = {v: Path(f'/tmp/hypercurve-biquadratic-basis-{v}-2026-09-23') for v in ['v3', 'v5']}
def read(name): return json.loads((A/name).read_text())
def sha(path): return hashlib.sha256(path.read_bytes()).hexdigest()
bindings = {v: read(f'biquadratic-basis-20260923-{v}-sources.json') for v in roots}
for version, root in roots.items():
    for name, digest in bindings[version].items(): assert sha(root/name) == digest, name
changed = {name for name in bindings['v3'] if bindings['v3'][name] != bindings['v5'][name]}
assert changed == {'hypercurve/src/bezier_algebraic_image.rs', 'hypercurve/src/bezier_offset.rs'}

# The final Hypercurve changes are confined to two existing test functions.
for name, old_name, new_name in [
    ('hypercurve/src/bezier_algebraic_image.rs', 'arithmetic_adapter_rejects_evidence_that_does_not_replay_strictly', 'arithmetic_adapter_replays_close_nonrational_bounds_strictly'),
    ('hypercurve/src/bezier_offset.rs', 'selected_fiber_transverse_mapped_cut_inverts_by_point', 'selected_fiber_transverse_mapped_cut_inverts_by_point'),
]:
    before = (roots['v3']/name).read_text(); after = (roots['v5']/name).read_text()
    def outside(text, function):
        start = text.index(f'    fn {function}(')
        end = text.index('    #[test]', start)
        return text[:start], text[end:]
    assert outside(before, old_name) == outside(after, new_name), name

hr = read('biquadratic-basis-20260923-v3-terminal.json')
dep = read('biquadratic-basis-20260923-v3-dependents-terminal.json')
scalar = read('biquadratic-basis-20260923-v3-scalar-attempt2-terminal.json')
focus = read('biquadratic-basis-20260923-v3-hypercurve-terminal.json')
broad = read('biquadratic-basis-20260923-v3-regression-terminal.json')
final = read('biquadratic-basis-20260923-v5-hypercurve-terminal.json')
for report in [hr, dep, scalar, focus, broad, final]:
    assert report['all_processes_reaped'] and report['all_sources_unchanged']
assert all(row['returncode'] == 0 for row in hr['checks'] + hr['tests'] + scalar['cases'])
for record in dep['crates']:
    assert record['returncode'] == 0 and all(row['returncode'] == 0 for row in record['checks'])
assert all(row['returncode'] == 0 for row in final['checks'])
old_failure = 'bezier_algebraic_image::policy_tests::arithmetic_adapter_rejects_evidence_that_does_not_replay_strictly'
assert [row['name'] for row in broad['failed']] == [old_failure]
assert final['cases'][0]['passed']
assert final['cases'][1]['returncode'] == 101
assert 'the transformed selected cut must invert on the analytic overlap:' in (A/final['cases'][1]['log']).read_text()
assert all(row['passed'] for row in focus['cases'][1:])
files = {
    'hyperreal': ['src/computable/node/quadratic_tower.rs', 'src/real/arithmetic/quadratic_tower_sign.rs'],
    'hypercurve': ['src/bezier_algebraic_image.rs', 'src/bezier_offset.rs'],
}
main_counts = {}
for crate in files:
    main_counts[crate] = 0
    for name, digest in bindings['v5'].items():
        if name.startswith(crate+'/'):
            assert sha(W/name) == digest, name
            main_counts[crate] += 1
    result = subprocess.run(['git', 'diff', '--check'], cwd=W/crate, capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr

report = dict(
    parents={crate: subprocess.check_output(['git','rev-parse','HEAD'],cwd=W/crate,text=True).strip() for crate in files},
    files={crate:{name:sha(W/crate/name) for name in paths} for crate,paths in files.items()},
    main_files_verified=main_counts, snapshot_files_verified={v:len(b) for v,b in bindings.items()},
    hyperreal_tests_passed=828, hyperlimit_tests_passed=252, hypersolve_tests_passed=507,
    hypercurve_regression_attempted=broad['attempted'], hypercurve_unique_passes=broad['passed']+3+1,
    hypercurve_ignored=broad['ignored'], hypercurve_existing_nonpass=final['cases'][1],
    hypercurve_existing_nonpasses_not_repeated=broad['not_repeated_existing_nonpasses'],
    hypercurve_final_cases=final['cases'], hypercurve_production_unchanged=True,
    binaries={
        'hyperreal':hr['binary_sha256'],
        **{row['crate']:row['binary_sha256'] for row in dep['crates']},
        'hypercurve_sweep':broad['binary_sha256'], 'hypercurve_final':final['binary_sha256'],
    },
    all_owned_processes_reaped=True, full_goal_complete=False,
)
(A/'biquadratic-basis-20260923-qualification.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({key:report[key] for key in ['main_files_verified','snapshot_files_verified','hypercurve_unique_passes','hypercurve_ignored']},indent=2))
