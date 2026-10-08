from pathlib import Path
import hashlib, json, subprocess, sys
A = Path(__file__).resolve().parent
W = A.parent
version = sys.argv[1]
assert version.startswith('v') and version[1:].isdigit()
prefix = f'independent-field-corners-20260924-{version}'
root = Path(f'/tmp/hypercurve-independent-field-corners-{version}-20260924')
manifest = json.loads((A / f'{prefix}-sources.json').read_text())
focused = json.loads((A / f'{prefix}-terminal.json').read_text())
full = json.loads((A / f'{prefix}-full-resumed-terminal.json').read_text())
cases = json.loads((A / f'{prefix}-full-resumed-cases.json').read_text())
selection = json.loads((A / f'{prefix}-full-resumed-selection.json').read_text())
baseline = json.loads((A / 'rational-polynomial-20260924-v8-full-cases.json').read_text())
baseline += json.loads((A / 'general-vector-tangents-20260924-v1-broad-resumed-cases.json').read_text())
baseline += json.loads((A / 'general-vector-tangents-20260924-v1-corners-terminal.json').read_text())['cases']
for result in [focused, full]:
    assert result['all_sources_unchanged'] and result['all_processes_reaped']
    assert len(result['checks']) == 3
    assert all(row['returncode'] == 0 for row in result['checks'])
assert len(focused['cases']) == 10
assert all(row['returncode'] == 0 for row in focused['cases'])
assert len(manifest) == 2044
for name, sha in manifest.items():
    assert hashlib.sha256((root / name).read_bytes()).hexdigest() == sha, name
    assert hashlib.sha256((W / name).read_bytes()).hexdigest() == sha, name
for artifact in full['binaries'].values():
    assert hashlib.sha256(Path(artifact['path']).read_bytes()).hexdigest() == artifact['sha256']
assert full['binaries']['hypercurve']['sha256'] == focused['binary_sha256']
assert len(cases) == full['attempted'] == len(selection['jobs']) == 1361
assert len({(row['target'], row['name']) for row in cases}) == len(cases)
assert {(row['target'], row['name']) for row in cases} == {tuple(job) for job in selection['jobs']}
assert len([row for row in cases if row['target'] != 'hypercurve']) == 121
assert all(row['passed'] for row in cases if row['target'] != 'hypercurve')
previous = {(row['target'], row['name']): row for row in baseline}
resolved = 'bezier_region::tests::independent_field_corner_edits_preserve_normalized_sets'
for row in cases:
    if row['target'] == 'hypercurve' and row['name'] == resolved:
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
files = ['src/bezier_offset.rs', 'src/bezier_region.rs', 'src/curve.rs', 'src/curve_region_boolean.rs']
parent = 'c13778eb15662d3b12093a54e518e9fb137c39d1'
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
              focused=f'{prefix}-terminal.json', full=f'{prefix}-full-resumed-terminal.json',
              attempted=full['attempted'], passed=full['passed'], ignored=full['ignored'],
              unchanged_nonpasses=full['nonpasses'], newly_passing=newly_passing,
              resolved_regression=resolved, public_integration_passes=121, full_goal_complete=False)
(A / f'{prefix}-qualification.json').write_text(json.dumps(report, indent=2)+'\n')
print('Qualified:', report['attempted'], 'attempted,', report['passed'], 'passed,',
      report['ignored'], 'ignored,', len(report['unchanged_nonpasses']), 'unchanged nonpasses')
