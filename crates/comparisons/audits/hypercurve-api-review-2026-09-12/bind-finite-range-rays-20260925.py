from pathlib import Path
import hashlib, json, subprocess, sys

A = Path(__file__).resolve().parent
W = A.parent
phase = sys.argv[1]
assert phase in {'qualified', 'staged', 'committed'}
prefix = 'finite-range-rays-20260925-v63-candidate'
control_prefix = 'finite-range-rays-20260925-v62-parent'
candidate = json.loads((A / f'{prefix}-terminal.json').read_text())
control = json.loads((A / f'{control_prefix}-terminal.json').read_text())
assert candidate['parent'] == control['parent']
manifests = {}
for label, report in [(prefix, candidate), (control_prefix, control)]:
    assert report['all_processes_reaped'] and report['all_sources_unchanged']
    assert report['build_returncode'] == 0
    manifest = json.loads((A / report['source_manifest']).read_text())
    assert len(manifest) == 2044
    manifests[label] = manifest
    for name, expected in manifest.items():
        assert hashlib.sha256((A / 'source-archives' / label / name).read_bytes()).hexdigest() == expected, name
    for binary in report['binaries'].values():
        assert hashlib.sha256(Path(binary['path']).read_bytes()).hexdigest() == binary['sha256']
assert len(candidate['checks']) == 3 and all(row['returncode'] == 0 for row in candidate['checks'])
assert len(candidate['cases']) == len(candidate['selection']) == 185
assert len(control['cases']) == len(control['selection']) == 4
assert not any(row['ignored'] for row in candidate['cases'] + control['cases'])
assert all(row['returncode'] == 0 for row in candidate['cases'])
assert all(row['returncode'] == 101 for row in control['cases'])
assert sum(row['target'] != 'hypercurve' for row in candidate['cases']) == 109
by_name = {row['name']: row for row in candidate['cases']}
newly_passing = sorted(row['name'] for row in control['cases'])
assert all(by_name[name]['returncode'] == 0 for name in newly_passing)
guard = json.loads((A / candidate['workspace_guard']).read_text())
for name, expected in guard.items():
    assert hashlib.sha256((W / name).read_bytes()).hexdigest() == expected, name
source_manifest = manifests[prefix]
selected = candidate['selected_files']
assert selected == ['src/bezier_offset.rs', 'src/bezier_region.rs', 'src/curve_region_boolean.rs']
for name in source_manifest:
    if name.removeprefix('hypercurve/') not in selected:
        assert source_manifest[name] == manifests[control_prefix][name], name
remaining = sorted(name.removeprefix('hypercurve/') for name in guard
                   if name.startswith('hypercurve/') and guard[name] != source_manifest[name])
assert len(remaining) == 6
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
        assert paths == (remaining if phase == 'committed' else candidate['workspace_changed']), paths
        assert staged == (selected if phase == 'staged' else []), staged
        if phase != 'committed':
            assert head == candidate['parent']
        else:
            assert subprocess.check_output(['git', 'rev-parse', 'HEAD^'], cwd=repo, text=True).strip() == candidate['parent']
            assert subprocess.check_output(['git', 'diff-tree', '--no-commit-id', '--name-only', '-r', 'HEAD'], cwd=repo, text=True).splitlines() == selected
        if phase != 'qualified':
            for name in selected:
                revision = ':' if phase == 'staged' else 'HEAD:'
                content = subprocess.check_output(['git', 'show', revision + name], cwd=repo)
                assert hashlib.sha256(content).hexdigest() == source_manifest['hypercurve/' + name], name
    else:
        assert not paths and not staged, (repo.name, paths, staged)
    subprocess.run(['git', 'diff', '--check'], cwd=repo, check=True)
    subprocess.run(['git', 'diff', '--cached', '--check'], cwd=repo, check=True)
    heads.append(dict(name=repo.name, head=head))
assert len(heads) == 30
report = dict(phase=phase, parent=candidate['parent'], files={name: source_manifest['hypercurve/' + name] for name in selected},
              repositories=heads, source_manifest=candidate['source_manifest'], workspace_guard=candidate['workspace_guard'],
              candidate=f'{prefix}-terminal.json', parent_control=f'{control_prefix}-terminal.json',
              checks=3, attempted=185, passed=185, public_passes=109, ignored=0, newly_passing=newly_passing,
              remaining_uncommitted=remaining, all_inputs_and_executables_match=True,
              all_owned_processes_reaped=True, full_goal_complete=False)
(A / f'finite-range-rays-20260925-v63-{phase}.json').write_text(json.dumps(report, indent=2) + '\n')
print(phase, ': 185 passing cases, 109 public passes, four parent failures repaired, 30 repositories bound')
