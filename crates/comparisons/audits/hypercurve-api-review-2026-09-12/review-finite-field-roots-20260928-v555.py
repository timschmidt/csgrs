from pathlib import Path
import hashlib,json,re,subprocess,sys
A=Path(__file__).resolve().parent;W=A.parent;prefix='finite-field-roots-20260928-v555'
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def git(repo,*args):return subprocess.check_output(['git',*args],cwd=W/repo)
r=json.loads((A/f'{prefix}-terminal.json').read_text());assert r['qualification_complete'] and r['probe_complete'] and r['all_processes_reaped']
manifest=json.loads((A/r['source_manifest']).read_text())
for name,sha in manifest.items():
 for root in [W,Path(r['source_directory']),A/'build-workspace-20260925']:assert digest(root/name)==sha,(root,name)
assert len(r['builds'])==1 and len(r['checks'])==6 and len(manifest)==2047
for b in r['builds']:
 assert b['returncode']==0
 for artifact in b['binaries'].values():assert digest(Path(artifact['path']))==artifact['sha256']
expected={(target,name) for target,names in r['expected_cases'].items() for name in names}
assert len(r['cases'])==len(expected)==449
assert {(c['target'],c['name']) for c in r['cases']}==expected
for c in r['cases']:assert c['passed'] and c['returncode']==0 and re.search(r'test result: ok\. 1 passed;',(A/c['log']).read_text())
assert all(c['returncode']==0 for c in r['checks'])
prior=json.loads((A/'fillet-endpoint-bridge-20260927-v548-terminal.json').read_text());assert prior['all_processes_reaped'] and prior['qualification_complete']
old_manifest=json.loads((A/prior['source_manifest']).read_text());paths={'hypercurve':['src/bezier_parameter.rs','src/curve_fillet.rs']}
assert {n for n,h in manifest.items()if h!=old_manifest.get(n)}=={'hypercurve/'+p for p in paths['hypercurve']}
for n,h in old_manifest.items():assert digest(Path(prior['source_directory'])/n)==h,n
assert len(r['builds'][0]['binaries'])==8
old_times={}
for baseline in ['retained-tangent-pair-broad-20260927-v540','fillet-endpoint-bridge-20260927-v548']:
 for case in json.loads((A/f'{baseline}-terminal.json').read_text())['cases']:old_times[case['name']]=case['elapsed_seconds']
regressions=[];workload_changes=[]
for case in r['cases']:
 old=old_times.get(case['name'])
 if old is not None and old>.1 and case['elapsed_seconds']>old*1.25+.2:regressions.append(dict(name=case['name'],before=old,after=case['elapsed_seconds']))
previous=json.loads((A/'fillet-endpoint-bridge-20260927-v548-repositories-after.json').read_text());repositories={};repos={}
for name,old in previous.items():
 head=git(name,'rev-parse','HEAD').decode().strip();status=git(name,'status','--porcelain=v1').decode();assert head==old['head'],name
 if name in paths:
  assert status==''.join(' M '+p+'\n'for p in paths[name]),(name,status)
  assert not git(name,'diff','--cached','--name-only').strip()
  subprocess.run(['git','diff','--check'],cwd=W/name,check=True)
  repos[name]=dict(parent=head,paths={p:manifest[name+'/'+p]for p in paths[name]})
 else:assert not status,name
 repositories[name]=dict(head=head,status=status)
(A/f'{prefix}-repositories-before.json').write_text(json.dumps(repositories,indent=2)+'\n')
review=dict(qualification=f'{prefix}-terminal.json',source_manifest=r['source_manifest'],outer_session_reaped=int(sys.argv[1]),validated_files=len(manifest),selected_geometry_tests=len(r['cases']),checks=len(r['checks']),repos=repos,scope='Finite nonrational polynomial isolation reuses the Hypersolve division-free authority, strict scalar signs, original root evidence and Sturm fallback; exact stationary family region compositions close; full implementation goal remains active',performance_changes=regressions,workload_changes=workload_changes)
(A/f'{prefix}-reviewed.json').write_text(json.dumps(review,indent=2)+'\n')
message="Certify finite exact parameter intervals without field division\n\nFinite nonrational polynomial queries now share Hypersolve's ordered-field Bernstein isolator before Sturm replay. Keep the original polynomial and simple-root certificate, require strict scalar decisions, and retain the complete fallback for unresolved and repeated roots. This avoids an unnecessary degree-20 algebraic Sturm chain that blocked a representable stationary-parameter fillet/chamfer/offset composition.\n\nValidation: 449 release regressions across eight binaries and six static/downstream checks. Direct regressions cover irrational finite bounds, nonrational coefficient scales, original-root evidence reuse, repeated roots and closed endpoints. The stationary quartic family now continues through constrained filleting, normalization, Boolean comparison, chamfer, round offset and nontrivial Boolean clipping under both exact policies.\n"
(A/f'{prefix}-hypercurve-commit.txt').write_text(message)
print('Reviewed',len(manifest),'sources;',len(r['cases']),'domain/family tests;6 checks;',len(repositories),'repositories; performance changes',regressions)
