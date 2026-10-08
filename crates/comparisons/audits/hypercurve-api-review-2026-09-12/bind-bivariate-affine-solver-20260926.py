from pathlib import Path
import hashlib,json,subprocess,sys
A=Path(__file__).resolve().parent
W=A.parent
phase=sys.argv[1]
assert phase in ['qualified','staged','committed']
prefix='bivariate-affine-solver-20260926-v119'
r=json.loads((A/'bivariate-affine-20260925-v119-terminal.json').read_text())
assert r['all_processes_reaped'] and r['all_sources_unchanged']
assert r['hypersolve_passed']==516 and r['hypersolve_build_returncode']==0
assert all(row['returncode']==0 for row in r['checks'] if row['repo']=='hypersolve')
assert all(row['returncode']==0 for row in r['cases'] if row['repo']=='hypersolve')
manifest=json.loads((A/r['source_manifest']).read_text())
for name,sha in manifest.items():
    for root in [W,Path(r['source_directory']),Path(r['build_source_directory'])]:
        assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,name
for binary in r['binaries'].values():
    assert hashlib.sha256(Path(binary['path']).read_bytes()).hexdigest()==binary['sha256']
def git(repo,*args):
    return subprocess.check_output(['git',*args],cwd=W/repo)
repo='hypersolve'
files=['src/curve_resultant.rs']
head=git(repo,'rev-parse','HEAD').decode().strip()
staged=git(repo,'diff','--cached','--name-only').decode().splitlines()
if phase=='committed':
    assert not staged
    assert git(repo,'rev-parse','HEAD^').decode().strip()==r['parents'][repo]
    assert git(repo,'diff-tree','--no-commit-id','--name-only','-r','HEAD').decode().splitlines()==files
else:
    assert head==r['parents'][repo]
    assert staged==(files if phase=='staged' else [])
if phase!='qualified':
    for file in files:
        blob=git(repo,'show',('HEAD:' if phase=='committed' else ':')+file)
        assert hashlib.sha256(blob).hexdigest()==manifest[repo+'/'+file]
for other in ['hypercurve','hyperreal']:
    assert git(other,'rev-parse','HEAD').decode().strip()==r['parents'][other]
    assert not git(other,'diff','--cached','--name-only')
statuses={repo.name:git(repo.name,'status','--short').decode() for repo in W.iterdir() if (repo/'.git').exists()}
assert len(statuses)==30
assert all(not status for repo,status in statuses.items() if repo not in ['hypercurve','hypersolve'])
receipt=dict(phase=phase,head=head,files=files,qualification='bivariate-affine-20260925-v119-terminal.json',
    source_manifest=r['source_manifest'],hypersolve_passed=516,ignored=0,
    outer_session=62814,outer_exit_code=1,
    scope='Hypersolve primitive; two pre-existing combined Hypercurve workloads remain timed out',
    all_sources_and_executables_unchanged=True,all_owned_processes_reaped=True,repository_statuses=statuses)
(A/f'{prefix}-{phase}.json').write_text(json.dumps(receipt,indent=2)+'\n')
print(phase,head,'516 solver tests passed; source, executable and index/HEAD binding verified.')
