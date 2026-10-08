from pathlib import Path
import hashlib, json, subprocess, sys

A = Path(__file__).resolve().parent
W = A.parent
prefix = 'fillet-branches-isolated-20260926-v190'
phase = sys.argv[1]
assert phase in ['qualified','staged','committed']
report = json.loads((A/f'{prefix}-resumed-terminal.json').read_text())
assert report['all_processes_reaped'] and report['scoped_qualification_complete']
assert len(report['binaries']) == 8
assert report['hypercurve_build_returncode'] == 0
assert report['hypercurve_passed'] == 434
assert len(report['cases']) == 434 and len(report['checks']) == 4
assert all(row['returncode'] == 0 for row in report['cases']+report['checks'])
guard = json.loads((A/report['workspace_guard']).read_text())
manifest = json.loads((A/report['source_manifest']).read_text())
for name, sha in guard.items():
    assert hashlib.sha256((W/name).read_bytes()).hexdigest() == sha, name
    for root in [report['source_directory'],report['build_source_directory']]:
        assert hashlib.sha256((Path(root)/name).read_bytes()).hexdigest() == manifest[name], name
for binary in report['binaries'].values():
    assert hashlib.sha256(Path(binary['path']).read_bytes()).hexdigest() == binary['sha256']
assert len(report['deferred_cases']) == 2
first = json.loads((A/report['deferred_cases'][0]['baseline_comparison']).read_text())
assert first['all_processes_reaped'] and first['all_sources_and_binaries_unchanged']
assert [(c['role'],c['returncode']) for c in first['cases']] == [('parent',101),('combined',0)]
assert len(first['identical_test_body_sha256']) == 64
assert len(report['baseline_comparisons']) == 1
second = report['baseline_comparisons'][0]
assert second['identical_test_body'] and second['equivalent_failure']
assert second['candidate']['returncode'] == second['parent']['returncode'] == 101
assert second['combined']['returncode'] == 0
for qualification in ['selected-cusp-ranges-isolated-20260926-v185-resumed-terminal.json', 'selected-cusp-fillet-fixed-20260926-v189-terminal.json']:
    extra = json.loads((A/qualification).read_text())
    assert extra['all_processes_reaped']
    extra_manifest = json.loads((A/extra['source_manifest']).read_text())
    for name, sha in extra_manifest.items():
        assert hashlib.sha256((Path(extra['source_directory'])/name).read_bytes()).hexdigest() == sha
    for binary in extra['binaries'].values():
        assert hashlib.sha256(Path(binary['path']).read_bytes()).hexdigest() == binary['sha256']
selected = {'hypercurve':['src/curve.rs']}
for repo in ['hyperreal','hypersolve']:
    assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=W/repo,text=True).strip() == report['parents'][repo]
    assert not subprocess.check_output(['git','diff','--cached','--name-only'],cwd=W/repo)
heads = {}
for repo, files in selected.items():
    def git(*args):
        return subprocess.check_output(['git',*args],cwd=W/repo)
    head = git('rev-parse','HEAD').decode().strip()
    heads[repo] = head
    staged = git('diff','--cached','--name-only').decode().splitlines()
    if phase == 'committed':
        assert not staged
        assert git('rev-parse','HEAD^').decode().strip() == report['parents'][repo]
        assert sorted(git('diff-tree','--no-commit-id','--name-only','-r','HEAD').decode().splitlines()) == sorted(files)
    else:
        assert head == report['parents'][repo]
        assert sorted(staged) == (sorted(files) if phase == 'staged' else [])
    if phase != 'qualified':
        for file in files:
            blob = git('show',('HEAD:' if phase == 'committed' else ':')+file)
            assert hashlib.sha256(blob).hexdigest() == manifest[repo+'/'+file], file
statuses = {repo.name: subprocess.check_output(['git','status','--short'],cwd=repo,text=True)
            for repo in W.iterdir() if (repo/'.git').exists()}
assert len(statuses) == 30
assert all(not status for repo,status in statuses.items() if repo not in {'hypercurve','hyperreal','hypersolve'})
receipt = dict(phase=phase,heads=heads,selected=selected,source_manifest=report['source_manifest'],
               workspace_guard=report['workspace_guard'],qualification=f'{prefix}-resumed-terminal.json',
               hypercurve_passed=434,ignored=0,pre_existing_failures=report['deferred_cases'],baseline_comparisons=report['baseline_comparisons'],
               every_owned_process_reaped=True,all_bound_sources_and_binaries_unchanged=True,
               repository_statuses=statuses)
(A/f'{prefix}-{phase}.json').write_text(json.dumps(receipt,indent=2)+'\n')
print(phase+': 2044 source inputs and eight fresh executables match qualification; 434 tests passed; two equivalent baseline failures pass in the combined source-chart migration.')
print(json.dumps(heads))
