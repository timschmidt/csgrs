from pathlib import Path
import hashlib, json, subprocess, sys

A = Path(__file__).resolve().parent
W = A.parent
phase = sys.argv[1]
assert phase in {'qualified', 'staged', 'committed'}
prefix = 'native-root-ownership-20260925-v67-candidate'
baseline_prefix = 'native-root-ownership-20260925-v64'
candidate = json.loads((A / f'{prefix}-terminal.json').read_text())
baseline = json.loads((A / f'{baseline_prefix}-terminal.json').read_text())
manifest = json.loads((A / candidate['source_manifest']).read_text())
baseline_manifest = json.loads((A / 'local-chord-complete-replay-20260924-v64-sources.json').read_text())
candidate_root = A / 'source-archives' / prefix
baseline_root = A / 'source-archives' / 'hypercurve-local-chord-complete-replay-v64-20260924'
for report, root, sources in [(candidate, candidate_root, manifest), (baseline, baseline_root, baseline_manifest)]:
    assert report['all_processes_reaped'] and report['all_sources_unchanged']
    assert report['build_returncode'] == 0
    assert len(sources) == 2044
    for name, expected in sources.items():
        assert hashlib.sha256((root / name).read_bytes()).hexdigest() == expected, name
    for binary in report['binaries'].values():
        assert hashlib.sha256(Path(binary['path']).read_bytes()).hexdigest() == binary['sha256']
    assert len(report['checks']) == 3 and all(row['returncode'] == 0 for row in report['checks'])
assert len(candidate['cases']) == len(candidate['selection']) == 226
assert all(row['returncode'] == 0 and not row['ignored'] for row in candidate['cases'])
assert sum(row['target'] != 'hypercurve' for row in candidate['cases']) == 109
assert len(baseline['cases']) == 7
regressions = {'imported_root_comparisons_preserve_owned_and_excluded_endpoints',
               'decreasing_root_charts_preserve_owned_and_excluded_endpoints'}
assert {row['label'] for row in baseline['cases'] if row['returncode'] != 0} == regressions
for row in baseline['cases']:
    assert row['returncode'] == (101 if row['label'] in regressions else 0)
    assert '0 ignored' in (A / row['log']).read_text()
assert all(any(row['name'].endswith('::' + name) for row in candidate['cases']) for name in regressions)
for name in manifest:
    if not name.startswith('hypercurve/'):
        assert manifest[name] == baseline_manifest[name], name

local = 'src/bezier_parameter.rs'
old = (baseline_root / 'hypercurve' / local).read_text()
new = (candidate_root / 'hypercurve' / local).read_text()
start = '    fn imported_parameters_with_excluded_endpoint_roots('
old_end = '    #[test]\n    fn retained_refinement_matches_square_free_reference_and_shares_sturm_work()'
new_end = '    #[test]\n    fn collapsed_exact_real_representation_imports_as_an_exact_parameter()'
assert old[old.index(start):old.index(old_end)] == new[new.index(start):new.index(new_end)]
parent = subprocess.check_output(['git', 'show', candidate['parent'] + ':' + local], cwd=W/'hypercurve', text=True)
for begin, end in [
    ('    fn from_algebraic_root_representation_with_domain(', '    pub(crate) fn refined_isolating_interval('),
    ('    pub(crate) fn same_value(', '\nfn disjoint_parameter_interval_order('),
]:
    assert old[old.index(begin):old.index(end, old.index(begin))] == parent[parent.index(begin):parent.index(end, parent.index(begin))]

guard = json.loads((A / candidate['workspace_guard']).read_text())
for name, expected in guard.items():
    assert hashlib.sha256((W / name).read_bytes()).hexdigest() == expected, name
selected = candidate['selected_files']
assert selected == [local]
remaining = sorted(name.removeprefix('hypercurve/') for name in guard
                   if name.startswith('hypercurve/') and guard[name] != manifest[name])
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
            content = subprocess.check_output(['git', 'show', (':' if phase == 'staged' else 'HEAD:') + local], cwd=repo)
            assert hashlib.sha256(content).hexdigest() == manifest['hypercurve/' + local]
    else:
        assert not paths and not staged, (repo.name, paths, staged)
    subprocess.run(['git', 'diff', '--check'], cwd=repo, check=True)
    subprocess.run(['git', 'diff', '--cached', '--check'], cwd=repo, check=True)
    heads.append(dict(name=repo.name, head=head))
assert len(heads) == 30
receipt = dict(phase=phase, parent=candidate['parent'], files={local: manifest['hypercurve/' + local]},
               candidate=f'{prefix}-terminal.json', pre_change_working_tree=f'{baseline_prefix}-terminal.json',
               source_manifest=candidate['source_manifest'], workspace_guard=candidate['workspace_guard'],
               checks=3, attempted=226, passed=226, public_passes=109, ignored=0,
               newly_passing=sorted(regressions), remaining_uncommitted=remaining, repositories=heads,
               all_inputs_and_executables_match=True, all_owned_processes_reaped=True, full_goal_complete=False)
(A / f'native-root-ownership-20260925-v67-{phase}.json').write_text(json.dumps(receipt, indent=2) + '\n')
print(phase, ': 226 passing cases, 109 public passes, exact pre-change regressions bound, 30 repositories audited')
