from pathlib import Path
import hashlib, json, subprocess, sys
A = Path(__file__).resolve().parent
W = A.parent
version = sys.argv[1]
assert version.startswith('v') and version[1:].isdigit()
prefix = f'local-chord-parallel-roots-20260924-{version}'
root = Path(f'/tmp/hypercurve-local-chord-parallel-roots-{version}-20260924')
manifest = json.loads((A / f'{prefix}-sources.json').read_text())
focused = json.loads((A / f'{prefix}-terminal.json').read_text())
full = json.loads((A / f'{prefix}-full-terminal.json').read_text())
cases = json.loads((A / f'{prefix}-full-cases.json').read_text())
selection = json.loads((A / f'{prefix}-full-selection.json').read_text())
baseline = json.loads((A / 'shared-parallel-expression-20260924-v1-full-cases.json').read_text())
for result in [focused, full]:
    assert result['all_sources_unchanged'] and result['all_processes_reaped']
    assert len(result['checks']) == 3
    assert all(row['returncode'] == 0 for row in result['checks'])
assert len(focused['cases']) == 45
assert all(row['returncode'] == 0 for row in focused['cases'])
assert len(manifest) == 2044
for name, sha in manifest.items():
    assert hashlib.sha256((root / name).read_bytes()).hexdigest() == sha, name
    assert hashlib.sha256((W / name).read_bytes()).hexdigest() == sha, name
for artifact in full['binaries'].values():
    assert hashlib.sha256(Path(artifact['path']).read_bytes()).hexdigest() == artifact['sha256']
assert full['binaries']['hypercurve']['sha256'] == focused['binary_sha256']
assert len(cases) == full['attempted'] == len(selection['jobs'])
assert len([row for row in cases if row['target'] == 'hypercurve']) == 1250
assert len({(row['target'], row['name']) for row in cases}) == len(cases)
assert {(row['target'], row['name']) for row in cases} == {tuple(job) for job in selection['jobs']}
public_targets = {row['target'] for row in cases if row['target'] != 'hypercurve'}
assert public_targets == {'hypercurve_path_closure', 'hypercurve_curve_region_boolean', 'hypercurve_bezier_arrangement', 'hypercurve_curve_region_boolean_fuzz', 'hypercurve_pcb_boolean_regressions', 'hypercurve_curve', 'hypercurve_curve_intersection', 'hypercurve_curve_point', 'hypercurve_curve_range'}
public_count = sum(row['target'] != 'hypercurve' for row in cases)
assert public_count == 225
assert all(row['passed'] for row in cases if row['target'] != 'hypercurve')
previous = {(row['target'], row['name']): row for row in baseline}
resolved = {'bezier_offset::conversion_tests::recursive_parameter_projection_preserves_roots_through_refinement_and_charts', 'bezier_parameter::conversion_tests::independent_singleton_charts_reuse_nested_root_identity', 'bezier_offset::conversion_tests::retained_parallel_contact_keeps_local_root_through_angular_and_point_queries', 'bezier_offset::conversion_tests::recursive_chord_parallel_local_roots_reject_conjugates_and_clip_exactly', 'bezier_offset::conversion_tests::recursive_chord_parallel_zero_norm_midpoint_is_not_a_component', 'bezier_region::tests::nonlinear_algebraic_endpoint_fillet_uses_complete_incident_domain'}
assert all(row['passed'] for row in cases if row['target'] == 'hypercurve' and row['name'] in resolved)
assert sum(row['target'] == 'hypercurve' and row['name'] in resolved for row in cases) == len(resolved)
for row in cases:
    key = row['target'], row['name']
    if key not in previous:
        assert row['target'] == 'hypercurve' and row['name'] in resolved and row['passed'], row
        continue
    prior = previous[key]
    if prior['passed']:
        assert row['passed'], row
    elif prior['ignored']:
        assert row['passed'] or row['ignored'], row
    elif not row['passed'] and not row['ignored']:
        assert prior['returncode'] == row['returncode'], row
        assert prior['limit_seconds'] == row['limit_seconds'], row
files = ['src/bezier_offset.rs', 'src/bezier_parameter.rs', 'src/bezier_region.rs', 'src/bezier_split.rs', 'src/curve.rs', 'src/curve_corner_chain.rs', 'src/curve_region_boolean.rs', 'src/curve_region_trim.rs', 'src/curve_subdivision.rs', 'src/curve_support_intersection.rs']
parent = '9945504d571c15316b9fbca22aa689a63b6dddb2'
repositories = []
for repo in sorted(W.iterdir()):
    if not (repo / '.git').exists():
        continue
    head = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=repo, text=True).strip()
    paths = subprocess.check_output(['git', 'diff', '--name-only'], cwd=repo, text=True).splitlines()
    assert paths == (files if repo.name == 'hypercurve' else []), (repo.name, paths)
    assert not subprocess.check_output(['git', 'diff', '--cached', '--name-only'], cwd=repo, text=True)
    assert not subprocess.check_output(['git', 'ls-files', '--others', '--exclude-standard'], cwd=repo, text=True)
    subprocess.run(['git', 'diff', '--check'], cwd=repo, check=True)
    if repo.name == 'hypercurve':
        assert head == parent
    repositories.append(dict(name=repo.name, head=head))
newly_passing = [row['name'] for row in cases
                if row['passed'] and (row['target'], row['name']) in previous
                and not previous[row['target'], row['name']]['passed']]
report = dict(parent=parent, files={name: manifest['hypercurve/' + name] for name in files},
              repositories=repositories, manifest=f'{prefix}-sources.json',
              all_2044_inputs_match=True, all_owned_processes_reaped=True,
              focused=f'{prefix}-terminal.json', full=f'{prefix}-full-terminal.json',
              attempted=full['attempted'], passed=full['passed'], ignored=full['ignored'],
              unchanged_nonpasses=full['nonpasses'], newly_passing=newly_passing,
              required_new_regressions=sorted(resolved), public_integration_passes=public_count, full_goal_complete=False)
(A / f'{prefix}-qualification.json').write_text(json.dumps(report, indent=2)+'\n')
print('Qualified:', report['attempted'], 'attempted,', report['passed'], 'passed,',
      report['ignored'], 'ignored,', len(report['unchanged_nonpasses']), 'unchanged nonpasses')
