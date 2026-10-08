from pathlib import Path
import hashlib,json,re,subprocess
A=Path(__file__).resolve().parent;W=A.parent;prefix='initial-endpoint-root-replay-20260927-v362';r=json.loads((A/f'{prefix}-terminal.json').read_text());assert r['qualification_complete'] and r['all_processes_reaped']
m=json.loads((A/r['source_manifest']).read_text())
for n,h in m.items():
 for root in [W,Path(r['source_directory']),A/'build-workspace-20260925']:assert hashlib.sha256((root/n).read_bytes()).hexdigest()==h,(root,n)
for b in r['builds']:
 assert b['returncode']==0;assert hashlib.sha256(Path(b['binary']['path']).read_bytes()).hexdigest()==b['binary']['sha256']
assert len(r['builds'])==2 and len(r['checks'])==7 and all(c['returncode']==0 for c in r['checks'])
cases=r['cases'];assert all(c['returncode']==0 for c in cases);assert [c['name']for c in cases[1:]]==r['expected_geometry_cases']
counts={}
for label in ['hypersolve-full-lib','hypersolve-integration']:
 counts[label]=[int(n)for n in re.findall(r'test result: ok\. (\d+) passed;', (A/next(c['log']for c in r['checks']if c['label']==label)).read_text())]
repo=W/'hypersolve'
def git(*args,cwd=repo):return subprocess.check_output(['git',*args],cwd=cwd)
parent='539eb997d21c2193b759172862667f4bb4e555bd';assert git('rev-parse','HEAD').decode().strip()==parent
assert git('status','--porcelain=v1').decode()==' M src/root_sign.rs\n';assert not git('diff','--cached','--name-only').strip()
previous=json.loads((A/'cached-tower-approximation-20260927-v357-repositories-after.json').read_text());repositories={}
for name,old in previous.items():
 head=git('rev-parse','HEAD',cwd=W/name).decode().strip();status=git('status','--porcelain=v1',cwd=W/name).decode();assert head==old['head'],name
 if name!='hypersolve':assert not status,name
 repositories[name]=dict(head=head,status=status)
(A/f'{prefix}-repositories-before.json').write_text(json.dumps(repositories,indent=2)+'\n')
record=dict(qualification=f'{prefix}-terminal.json',source_manifest=r['source_manifest'],outer_session_reaped=75007,reused_runtime_outer_session_reaped=11942,runtime_qualification_reused_from=r["runtime_qualification_reused_from"],runtime_source_difference=r["runtime_source_difference"],validated_files=len(m),selected_geometry_tests=len(r['expected_geometry_cases']),suite_pass_counts=counts,checks=len(r['checks']),repos={'hypersolve':dict(parent=parent,paths={'src/root_sign.rs':m['hypersolve/src/root_sign.rs']})},known_unresolved=r['known_unresolved'])
(A/f'{prefix}-reviewed.json').write_text(json.dumps(record,indent=2)+'\n')
(A/f'{prefix}-commit.txt').write_text('Preserve exact root-sign replay for narrow isolators\n\nAn undecided initial endpoint sign no longer bypasses the existing certified\ncoarser-singleton replay. Build the signed query chain once and reuse it only\nafter proving an enclosing bracket owns the same root. Exact endpoint roots\nstill decline explicitly and the retained equation and isolator are unchanged.\n\nCover 600- and 1200-bit isolators over pi and exp(1/3), both defining gauges,\npositive/negative queries, exact equality and the zero query. Existing foreign\nroot exclusion and endpoint ownership tests remain passing.\n\nValidation: 528 solver unit tests, 368 solver integration tests, '+str(len(r['expected_geometry_cases']))+' selected\ngeometry tests, all-target Clippy, formatting and denied-warning solver docs.\n')
print('Reviewed',len(m),'source inputs;',counts,'and',len(r['expected_geometry_cases']),'geometry tests')
