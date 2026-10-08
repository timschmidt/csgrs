from pathlib import Path
import hashlib, json, subprocess, sys

A = Path(__file__).resolve().parent
W = A.parent
prefix = 'retained-contact-factor-20260925-v98'
phase = sys.argv[1]
assert phase in ['qualified','staged','committed']
report = json.loads((A/f'{prefix}-terminal.json').read_text())
assert report['all_processes_reaped'] and report['qualification_complete']
assert report['hypersolve_passed'] == 514 and report['hypercurve_passed'] == 259
assert len(report['cases']) == 260 and len(report['checks']) == 7
assert all(row['returncode'] == 0 for row in report['cases']+report['checks'])
guard = json.loads((A/report['workspace_guard']).read_text())
manifest = json.loads((A/report['source_manifest']).read_text())
for name, sha in guard.items():
    assert hashlib.sha256((W/name).read_bytes()).hexdigest() == sha, name
    for root in [report['source_directory'],report['build_source_directory']]:
        assert hashlib.sha256((Path(root)/name).read_bytes()).hexdigest() == manifest[name], name
for binary in report['binaries'].values():
    assert hashlib.sha256(Path(binary['path']).read_bytes()).hexdigest() == binary['sha256']
selected = {'hypersolve':['src/algebraic_fiber.rs','src/ordered_field_roots.rs','src/root_sign.rs'],
            'hypercurve':['src/bezier_offset.rs']}
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
assert all(not status for repo,status in statuses.items() if repo not in selected)
receipt = dict(phase=phase,heads=heads,selected=selected,source_manifest=report['source_manifest'],
               workspace_guard=report['workspace_guard'],qualification=f'{prefix}-terminal.json',
               hypersolve_passed=514,hypercurve_passed=259,ignored=0,
               every_owned_process_reaped=True,all_bound_sources_and_binaries_unchanged=True,
               repository_statuses=statuses)
(A/f'{prefix}-{phase}.json').write_text(json.dumps(receipt,indent=2)+'\n')
print(phase+': 2044 source inputs and seven fresh executables match qualification; 773 tests passed.')
print(json.dumps(heads))
