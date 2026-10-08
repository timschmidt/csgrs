from pathlib import Path
import hashlib, json, subprocess

A = Path(__file__).resolve().parent
W = A.parent
prefix = 'general-vector-tangents-20260924-v1'
root = Path('/tmp/hypercurve-general-vector-tangents-v1-20260924')
manifest = json.loads((A / f'{prefix}-sources.json').read_text())
focused = json.loads((A / f'{prefix}-terminal.json').read_text())
broad = json.loads((A / f'{prefix}-broad-resumed-terminal.json').read_text())
cases = json.loads((A / f'{prefix}-broad-resumed-cases.json').read_text())
selection = json.loads((A / f'{prefix}-broad-resumed-selection.json').read_text())
supplement = json.loads((A / f'{prefix}-corners-terminal.json').read_text())
assert supplement['all_sources_unchanged'] and supplement['all_processes_reaped']
assert supplement['binary'] == broad['binaries']['hypercurve']
assert len(supplement['cases']) == 12
baseline = json.loads((A / 'rational-polynomial-20260924-v8-full-cases.json').read_text())
for result in [focused, broad]:
    assert result['all_sources_unchanged'] and result['all_processes_reaped']
    assert len(result['checks']) == 3
    assert all(row['returncode'] == 0 for row in result['checks'])
assert len(focused['cases']) == 8
assert all(row['returncode'] == 0 for row in focused['cases'])
assert len(manifest) == 2044
for name, sha in manifest.items():
    assert hashlib.sha256((root / name).read_bytes()).hexdigest() == sha, name
    assert hashlib.sha256((W / name).read_bytes()).hexdigest() == sha, name
for artifact in broad['binaries'].values():
    assert hashlib.sha256(Path(artifact['path']).read_bytes()).hexdigest() == artifact['sha256']
assert broad['binaries']['hypercurve']['sha256'] == focused['binary_sha256']
assert len(cases) == broad['attempted'] == len(selection['jobs'])
cases += supplement['cases']
selection['jobs'] += supplement['jobs']
assert len({(row['target'], row['name']) for row in cases}) == len(cases)
assert {(row['target'], row['name']) for row in cases} == {tuple(job) for job in selection['jobs']}
assert len([row for row in cases if row['target'] != 'hypercurve']) == 121
assert all(row['passed'] for row in cases if row['target'] != 'hypercurve')
previous = {(row['target'], row['name']): row for row in baseline}
resolved_ph = 'bezier_region::tests::closed_ph_corner_edits_preserve_both_normalized_source_lobes'
new_tests = {
    'bezier_offset::conversion_tests::vector_tangent_predicates_replay_general_parameter_authorities',
    'bezier_offset::conversion_tests::regularized_vector_tangent_owns_selected_stationary_endpoints',
}
for row in cases:
    if row['target'] == 'hypercurve' and row['name'] in new_tests | {resolved_ph}:
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
files = ['src/bezier_offset.rs', 'src/bezier_region.rs', 'src/curve.rs',
         'src/curve_corner_chain.rs', 'src/curve_region_boolean.rs']
parent = '8de08b874db388e25b2dae69cf93622cded79006'
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
              focused=f'{prefix}-terminal.json', broad=f'{prefix}-broad-resumed-terminal.json',
              corner_supplement=f'{prefix}-corners-terminal.json',
              attempted=len(cases), passed=sum(row['passed'] for row in cases), ignored=sum(row['ignored'] for row in cases),
              unchanged_nonpasses=[row for row in cases if not row['passed'] and not row['ignored']], newly_passing=newly_passing,
              public_integration_passes=121, full_goal_complete=False)
(A / f'{prefix}-qualification.json').write_text(json.dumps(report, indent=2) + '\n')
print('Qualified:', report['attempted'], 'attempted,', report['passed'], 'passed,',
      report['ignored'], 'ignored,', len(report['unchanged_nonpasses']), 'unchanged nonpasses')
