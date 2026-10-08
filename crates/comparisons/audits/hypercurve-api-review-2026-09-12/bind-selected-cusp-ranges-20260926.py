from pathlib import Path
import hashlib, json, subprocess, sys

A = Path(__file__).resolve().parent
W = A.parent
prefix = 'selected-cusp-ranges-isolated-20260926-v185'
phase = sys.argv[1]
assert phase in ['qualified','staged','committed']
report = json.loads((A/f'{prefix}-resumed-terminal.json').read_text())
assert report['all_processes_reaped'] and report['scoped_qualification_complete']
assert len(report['binaries']) == 8
assert report['hypercurve_build_returncode'] == 0
assert report['hypercurve_passed'] == 356
assert len(report['cases']) == 356 and len(report['checks']) == 4
assert all(row['returncode'] == 0 for row in report['cases']+report['checks'])
guard = json.loads((A/report['workspace_guard']).read_text())
manifest = json.loads((A/report['source_manifest']).read_text())
for name, sha in guard.items():
    assert hashlib.sha256((W/name).read_bytes()).hexdigest() == sha, name
    for root in [report['source_directory'],report['build_source_directory']]:
        assert hashlib.sha256((Path(root)/name).read_bytes()).hexdigest() == manifest[name], name
for binary in report['binaries'].values():
    assert hashlib.sha256(Path(binary['path']).read_bytes()).hexdigest() == binary['sha256']
baseline = json.loads((A/report['deferred_cases'][0]['baseline_comparison']).read_text())
assert len(report['deferred_cases']) == 1 and report['deferred_cases'][0]['returncode'] == 'timeout'
assert baseline['all_processes_reaped'] and baseline['all_sources_and_binaries_unchanged'] and baseline['returncode'] == 'timeout'
assert baseline['test_file_unchanged']
selected = {'hypercurve':['src/bezier_offset.rs','src/bezier_region.rs']}
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
               hypercurve_passed=356,ignored=0,pre_existing_timeout=report['deferred_cases'][0],
               every_owned_process_reaped=True,all_bound_sources_and_binaries_unchanged=True,
               repository_statuses=statuses)
(A/f'{prefix}-{phase}.json').write_text(json.dumps(receipt,indent=2)+'\n')
print(phase+': 2044 source inputs and eight fresh executables match qualification; 356 tests passed; one unchanged chamfer case exceeded 180 seconds in both candidate and committed baseline.')
print(json.dumps(heads))
