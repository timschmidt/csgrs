from pathlib import Path
import hashlib,json,subprocess,sys
A=Path(__file__).resolve().parent
W=A.parent
phase=sys.argv[1]
assert phase in ['qualified','staged','committed']
prefix='packed-integer-product-solver-20260926-v140'
r=json.loads((A/'corner-source-chart-complete-20260926-v140-terminal.json').read_text())
assert r['all_processes_reaped'] and r['hypersolve_passed']==520
assert r['hypersolve_build_returncode']==0 and r['hypercurve_build_returncode']==0
assert len(r['checks'])==7 and all(row['returncode']==0 for row in r['checks'])
assert len(r['binaries'])==8
assert all(row['returncode']==0 for row in r['cases'] if row['repo']=='hypersolve')
curve=[row for row in r['cases'] if row['repo']=='hypercurve']
assert len(curve)==143 and sum(row['returncode']==0 for row in curve)==142
assert curve[-1]['returncode']==101
failure=json.loads((A/'corner-source-chart-complete-20260926-v140-prepacking-failure.json').read_text())
assert failure['all_processes_reaped'] and failure['all_sources_unchanged']
assert failure['test_body_identical'] and failure['returncode']==101
manifest=json.loads((A/r['source_manifest']).read_text())
for name,sha in manifest.items():
    for root in [W,Path(r['source_directory']),Path(r['build_source_directory'])]:
        assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,name
for binary in r['binaries'].values():
    assert hashlib.sha256(Path(binary['path']).read_bytes()).hexdigest()==binary['sha256']
def git(repo,*args):
    return subprocess.check_output(['git',*args],cwd=W/repo)
repo='hypersolve'
files=['src/bareiss.rs','src/curve_resultant.rs','src/integer_interpolation.rs']
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
        assert hashlib.sha256(git(repo,'show',('HEAD:' if phase=='committed' else ':')+file)).hexdigest()==manifest[repo+'/'+file]
for other in ['hypercurve','hyperreal']:
    assert git(other,'rev-parse','HEAD').decode().strip()==r['parents'][other]
    assert not git(other,'diff','--cached','--name-only')
statuses={p.name:git(p.name,'status','--short').decode() for p in W.iterdir() if (p/'.git').exists()}
assert len(statuses)==30
assert all(not status for repo,status in statuses.items() if repo not in ['hypersolve','hypercurve'])
receipt=dict(phase=phase,head=head,files=files,qualification='corner-source-chart-complete-20260926-v140-terminal.json',
             source_manifest=r['source_manifest'],hypersolve_passed=520,hypercurve_passed=142,ignored=0,
             outer_session=36665,outer_exit_code=1,
             scope='Hypersolve determinant arithmetic; broader Hypercurve migration has a matched pre-packing chamfer failure and remains pending',
             all_sources_and_executables_unchanged=True,all_owned_processes_reaped=True,repository_statuses=statuses)
(A/f'{prefix}-{phase}.json').write_text(json.dumps(receipt,indent=2)+'\n')
print(phase,head,'520 solver tests passed; eight fresh executables and all source/index/HEAD bindings verified.')
