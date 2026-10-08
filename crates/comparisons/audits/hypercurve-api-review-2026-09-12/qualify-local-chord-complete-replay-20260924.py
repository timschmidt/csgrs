from pathlib import Path
import hashlib, json, subprocess, sys
A = Path(__file__).resolve().parent
W = A.parent
version = sys.argv[1]
assert version.startswith('v') and version[1:].isdigit()
prefix = f'local-chord-complete-replay-20260924-{version}'
root = Path(f'/tmp/hypercurve-local-chord-complete-replay-{version}-20260924')
manifest = json.loads((A / f'{prefix}-sources.json').read_text())
focused = json.loads((A / f'{prefix}-terminal.json').read_text())
full = json.loads((A / f'{prefix}-full-terminal.json').read_text())
cases = json.loads((A / f'{prefix}-full-cases.json').read_text())
selection = json.loads((A / f'{prefix}-full-selection.json').read_text())
baseline = json.loads((A / 'local-chord-parallel-roots-20260924-v22-full-cases.json').read_text())
for result in [focused, full]:
    assert result['all_sources_unchanged'] and result['all_processes_reaped']
    assert len(result['checks']) == 3
    assert all(row['returncode'] == 0 for row in result['checks'])
assert len(focused['cases']) == 68
assert all(row['returncode'] == 0 for row in focused['cases'])
assert len(manifest) == 2044
for name, sha in manifest.items():
    assert hashlib.sha256((root / name).read_bytes()).hexdigest() == sha, name
    assert hashlib.sha256((W / name).read_bytes()).hexdigest() == sha, name
for artifact in full['binaries'].values():
    assert hashlib.sha256(Path(artifact['path']).read_bytes()).hexdigest() == artifact['sha256']
assert full['binaries']['hypercurve']['sha256'] == focused['binary_sha256']
assert len(cases) == full['attempted'] == len(selection['jobs'])
assert len([row for row in cases if row['target'] == 'hypercurve']) == 1259
assert len({(row['target'], row['name']) for row in cases}) == len(cases)
assert {(row['target'], row['name']) for row in cases} == {tuple(job) for job in selection['jobs']}
public_targets = {row['target'] for row in cases if row['target'] != 'hypercurve'}
assert public_targets == {'hypercurve_path_closure', 'hypercurve_curve_region_boolean', 'hypercurve_bezier_arrangement', 'hypercurve_curve_region_boolean_fuzz', 'hypercurve_pcb_boolean_regressions', 'hypercurve_curve', 'hypercurve_curve_intersection', 'hypercurve_curve_point', 'hypercurve_curve_range', 'hypercurve_bezier_algebraic_parameter', 'hypercurve_bezier_fit_offset', 'hypercurve_analytic_parallel_region'}
public_count = sum(row['target'] != 'hypercurve' for row in cases)
assert public_count == 376
assert sum(row['target'] == 'hypercurve_bezier_fit_offset' for row in cases) == 107
assert sum(row['target'] == 'hypercurve_analytic_parallel_region' for row in cases) == 16
assert sum(row['target'] == 'hypercurve_bezier_algebraic_parameter' for row in cases) == 28
assert all(row['passed'] for row in cases if row['target'] != 'hypercurve')
previous = {(row['target'], row['name']): row for row in baseline}
old_name = 'bezier_offset::conversion_tests::selected_affine_tangent_source_retains_incident_endpoint_and_contacts_locally'
new_name = 'bezier_offset::conversion_tests::selected_source_range_retains_incident_endpoint_and_contacts_locally'
previous['hypercurve', new_name] = previous.pop(('hypercurve', old_name))
resolved = {'bezier_offset::conversion_tests::closed_parallel_secants_retain_endpoints_and_both_seam_parameters', 'bezier_offset::conversion_tests::closed_parallel_turn_certificate_retains_only_the_seam', 'bezier_offset::conversion_tests::closed_parallel_turn_certificate_rejects_multiple_traversal', 'bezier_region::tests::one_fragment_nonzero_parallel_loop_extends_chamfer_cuts_on_one_finite_envelope', 'bezier_offset::conversion_tests::local_parallel_endpoint_clipping_reuses_a_coefficient_root', 'bezier_parameter::conversion_tests::isolation_retains_large_rational_roots_until_projection_is_requested'}
resolved.add('bezier_offset::conversion_tests::recursive_local_isolation_reuses_exact_signs_below_interval_precision')
resolved.add('bezier_offset::conversion_tests::unused_positive_radicals_do_not_block_recursive_enclosures')
resolved.add('bezier_offset::conversion_tests::regular_parallel_orientation_reuse_rejects_ranges_crossing_cusps')
resolved.add('bezier_parameter::conversion_tests::solver_owned_root_refinement_excludes_repeated_lower_endpoints')
assert all(row['passed'] for row in cases if row['target'] == 'hypercurve' and row['name'] in resolved)
assert sum(row['target'] == 'hypercurve' and row['name'] in resolved for row in cases) == len(resolved)
for row in cases:
    key = row['target'], row['name']
    if key not in previous:
        if row['target'] in {'hypercurve_bezier_algebraic_parameter', 'hypercurve_bezier_fit_offset', 'hypercurve_analytic_parallel_region'}:
            assert row['passed'], row
            continue
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
files = ['src/bezier_offset.rs', 'src/bezier_parameter.rs', 'src/bezier_region.rs', 'src/curve.rs', 'src/curve_corner_chain.rs']
parent = 'c8c978a3e7e208fe8b17185a4e719b5c35392d4b'
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
