from pathlib import Path
import hashlib, json, subprocess, sys

A = Path(__file__).resolve().parent
W = A.parent
version, phase = sys.argv[1:]
assert phase in {'qualified', 'staged', 'committed'}
prefix = f'incident-ray-sectors-20260925-{version}'
candidate = json.loads((A / f'{prefix}-terminal.json').read_text())
control_prefix = f'incident-ray-sectors-parent-20260925-{version}'
control = json.loads((A / f'{control_prefix}-terminal.json').read_text())
assert candidate['parent'] == control['parent']
for label, report in [(prefix, candidate), (control_prefix, control)]:
    assert report['all_processes_reaped'] and report['all_sources_unchanged']
    assert report['build_returncode'] == 0
    manifest = json.loads((A / report['source_manifest']).read_text())
    assert len(manifest) == 2044
    for name, expected in manifest.items():
        assert hashlib.sha256((A / 'source-archives' / label / name).read_bytes()).hexdigest() == expected, name
    for binary in report['binaries'].values():
        assert hashlib.sha256(Path(binary['path']).read_bytes()).hexdigest() == binary['sha256']
assert len(candidate['checks']) == 3 and all(row['returncode'] == 0 for row in candidate['checks'])
assert len(candidate['cases']) == len(candidate['selection']) == 171
assert len(control['cases']) == len(control['selection']) == 3
assert not any(row['ignored'] for row in candidate['cases'] + control['cases'])
by_name = {row['name']: row for row in candidate['cases']}
regression = 'curve_region_boolean::certified_successor_tests::regularization_orders_all_branches_at_a_pinched_algebraic_corner'
assert by_name[regression]['returncode'] == 0
prior = {row['name']: row for row in control['cases']}
assert prior[regression]['returncode'] == 101
nonpasses = {row['name'] for row in candidate['cases'] if row['returncode'] != 0}
assert nonpasses == set(prior) - {regression}
for name in nonpasses:
    assert by_name[name]['returncode'] == prior[name]['returncode'] == 101
assert all(row['returncode'] == 0 for row in candidate['cases'] if row['target'] != 'hypercurve')
guard = json.loads((A / candidate['workspace_guard']).read_text())
for name, expected in guard.items():
    assert hashlib.sha256((W / name).read_bytes()).hexdigest() == expected, name
source_manifest = json.loads((A / candidate['source_manifest']).read_text())
selected = candidate['selected_files']
assert selected == ['src/curve_region_boolean.rs']
expected_files = sorted(selected + candidate['restored_to_parent'])
heads = []
for repo in sorted(W.iterdir()):
    if not (repo / '.git').exists():
        continue
    head = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=repo, text=True).strip()
    paths = subprocess.check_output(['git', 'diff', 'HEAD', '--name-only'], cwd=repo, text=True).splitlines()
    staged = subprocess.check_output(['git', 'diff', '--cached', '--name-only'], cwd=repo, text=True).splitlines()
    untracked = subprocess.check_output(['git', 'ls-files', '--others', '--exclude-standard'], cwd=repo, text=True).splitlines()
    assert not untracked, (repo.name, untracked)
    if repo.name == 'hypercurve':
        assert paths == (sorted(candidate['restored_to_parent']) if phase == 'committed' else expected_files), paths
        assert staged == (selected if phase == 'staged' else []), staged
        if phase != 'committed':
            assert head == candidate['parent']
        else:
            assert subprocess.check_output(['git', 'rev-parse', 'HEAD^'], cwd=repo, text=True).strip() == candidate['parent']
            assert subprocess.check_output(['git', 'diff-tree', '--no-commit-id', '--name-only', '-r', 'HEAD'], cwd=repo, text=True).splitlines() == selected
        for name in selected:
            expected = source_manifest['hypercurve/' + name]
            assert hashlib.sha256((repo / name).read_bytes()).hexdigest() == expected
            if phase != 'qualified':
                revision = ':' if phase == 'staged' else 'HEAD:'
                content = subprocess.check_output(['git', 'show', revision + name], cwd=repo)
                assert hashlib.sha256(content).hexdigest() == expected
    else:
        assert not paths and not staged, (repo.name, paths, staged)
    subprocess.run(['git', 'diff', '--check'], cwd=repo, check=True)
    subprocess.run(['git', 'diff', '--cached', '--check'], cwd=repo, check=True)
    heads.append(dict(name=repo.name, head=head))
assert len(heads) == 30
report = dict(phase=phase, parent=candidate['parent'], files={name: source_manifest['hypercurve/' + name] for name in selected},
              repositories=heads, source_manifest=candidate['source_manifest'], workspace_guard=candidate['workspace_guard'],
              candidate=f'{prefix}-terminal.json', parent_control=f'{control_prefix}-terminal.json',
              checks=3, attempted=171, passed=169, public_passes=109, ignored=0, unchanged_parent_failures=sorted(nonpasses),
              newly_passing=regression, all_inputs_and_executables_match=True, all_owned_processes_reaped=True,
              full_goal_complete=False)
(A / f'{prefix}-{phase}.json').write_text(json.dumps(report, indent=2) + '\n')
print(phase, ': 169 passing cases, 109 public passes, two parent failures reproduced, 30 repositories bound')
