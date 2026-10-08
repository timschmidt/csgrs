from pathlib import Path
import hashlib, json, subprocess, sys

A = Path(__file__).resolve().parent
W = A.parent
version = sys.argv[1]
assert version.startswith('v') and version[1:].isdigit()
prefix = f'represented-circle-incidence-20260924-{version}'
root = Path(f'/tmp/hypercurve-represented-circle-incidence-{version}-20260924')
manifest = json.loads((A / f'{prefix}-sources.json').read_text())
focused = json.loads((A / f'{prefix}-terminal.json').read_text())
full = json.loads((A / f'{prefix}-full-terminal.json').read_text())
cases = json.loads((A / f'{prefix}-full-cases.json').read_text())
selection = json.loads((A / f'{prefix}-full-selection.json').read_text())
baseline = json.loads((A / 'chord-point-containment-20260924-v3-full-cases.json').read_text())
baseline_manifest = json.loads((A / 'chord-point-containment-20260924-v3-sources.json').read_text())
for result in [focused, full]:
    assert result['all_sources_unchanged'] and result['all_processes_reaped']
    assert len(result['checks']) == 3
    assert all(row['returncode'] == 0 for row in result['checks'])
assert len(focused['cases']) == 8
assert all(row['returncode'] == 0 for row in focused['cases'])
assert len(manifest) == 2044
assert set(manifest) == set(baseline_manifest)
assert [name for name in manifest if manifest[name] != baseline_manifest[name]] == ['hypercurve/src/bezier_offset.rs']
for name, sha in manifest.items():
    assert hashlib.sha256((root / name).read_bytes()).hexdigest() == sha, name
    assert hashlib.sha256((W / name).read_bytes()).hexdigest() == sha, name
for artifact in full['binaries'].values():
    assert hashlib.sha256(Path(artifact['path']).read_bytes()).hexdigest() == artifact['sha256']
assert full['binaries']['hypercurve']['sha256'] == focused['binary_sha256']
assert len(cases) == full['attempted'] == len(selection['jobs'])
assert len({(row['target'], row['name']) for row in cases}) == len(cases)
assert {(row['target'], row['name']) for row in cases} == {tuple(job) for job in selection['jobs']}
new_tests = {
    'bezier_offset::conversion_tests::represented_circle_incidence_replays_retained_similarity_points',
    'bezier_offset::conversion_tests::represented_circle_retained_point_incidence_observes_requested_policy',
}
focused_names = {row['label'] for row in focused['cases']}
def related(row):
    return row['target'] != 'hypercurve' or any(word in row['name'] for word in ['fillet', 'circle', 'cusp', 'chord', 'retained_point', 'corner']) or row['name'].rsplit('::', 1)[-1] in focused_names
expected = {(row['target'], row['name']) for row in baseline if related(row)} | {('hypercurve', name) for name in new_tests}
assert {(row['target'], row['name']) for row in cases} == expected
public_targets = {row['target'] for row in cases if row['target'] != 'hypercurve'}
assert public_targets == {'hypercurve_path_closure', 'hypercurve_curve_region_boolean', 'hypercurve_bezier_arrangement', 'hypercurve_curve_region_boolean_fuzz', 'hypercurve_pcb_boolean_regressions', 'hypercurve_curve', 'hypercurve_curve_intersection', 'hypercurve_curve_point', 'hypercurve_curve_range'}
public_count = sum(row['target'] != 'hypercurve' for row in cases)
assert public_count == 225
assert all(row['passed'] for row in cases if row['target'] != 'hypercurve')
previous = {(row['target'], row['name']): row for row in baseline}
for row in cases:
    if row['target'] == 'hypercurve' and row['name'] in new_tests:
        assert row['passed'], row
        continue
    prior = previous[row['target'], row['name']]
    if prior['passed']:
        assert row['passed'], row
    elif prior['ignored']:
        assert row['passed'] or row['ignored'], row
    elif not row['passed'] and not row['ignored']:
        assert prior['returncode'] == row['returncode'], row
        assert prior['limit_seconds'] == row['limit_seconds'], row
files = ['src/bezier_offset.rs']
parent = '1cf070a5d1ec2ebf2796177da784a2d5a0494ee2'
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
newly_passing = [row['name'] for row in cases if row['passed'] and (row['target'], row['name']) in previous and not previous[row['target'], row['name']]['passed']]
report = dict(parent=parent, files={name: manifest['hypercurve/' + name] for name in files},
              repositories=repositories, manifest=f'{prefix}-sources.json',
              all_2044_inputs_match=True, all_owned_processes_reaped=True,
              focused=f'{prefix}-terminal.json', affected=f'{prefix}-full-terminal.json',
              unchanged_code_full_qualification='chord-point-containment-20260924-v3-qualification.json',
              attempted=full['attempted'], passed=full['passed'], ignored=full['ignored'],
              unchanged_nonpasses=full['nonpasses'], newly_passing=newly_passing,
              public_integration_passes=public_count, full_goal_complete=False)
(A / f'{prefix}-qualification.json').write_text(json.dumps(report, indent=2)+'\n')
print('Qualified:', report['attempted'], 'attempted,', report['passed'], 'passed,',
      report['ignored'], 'ignored,', len(report['unchanged_nonpasses']), 'unchanged nonpasses')
