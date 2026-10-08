from pathlib import Path
import hashlib,json,re,subprocess,sys
A=Path(__file__).resolve().parent;W=A.parent;prefix='common-point-incidence-broad-20260928-v593'
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
assert len(r['cases'])==len(expected)==489
assert {(c['target'],c['name']) for c in r['cases']}==expected
for c in r['cases']:assert c['passed'] and c['returncode']==0 and re.search(r'test result: ok\. 1 passed;',(A/c['log']).read_text())
assert all(c['returncode']==0 for c in r['checks'])
focus=json.loads((A/'common-point-incidence-20260928-v592-terminal.json').read_text());assert focus['all_processes_reaped'] and focus['probe_complete']
assert json.loads((A/focus['source_manifest']).read_text())==manifest
assert focus['builds'][0]['binaries']['hypercurve']['sha256']==r['builds'][0]['binaries']['hypercurve']['sha256']
reused=[c for c in r['cases']if 'reused_from'in c];assert len(reused)==33
for c in reused:
 assert c['reused_from']=='common-point-incidence-20260928-v592-terminal.json'
 assert any((old['target'],old['name'],old['log'],old['elapsed_seconds'])==(c['target'],c['name'],c['log'],c['elapsed_seconds'])and old['passed']for old in focus['cases'])
prior=json.loads((A/'recursive-point-incidence-broad-20260928-v587-terminal.json').read_text());assert prior['all_processes_reaped'] and prior['qualification_complete']
old_manifest=json.loads((A/prior['source_manifest']).read_text());paths={'hypercurve': ['src/bezier_offset.rs']}
assert {n for n,h in manifest.items()if h!=old_manifest.get(n)}=={'hypercurve/'+p for p in paths['hypercurve']}
for n,h in old_manifest.items():assert digest(Path(prior['source_directory'])/n)==h,n
assert len(r['builds'][0]['binaries'])==9
for target,listing in r['test_listings'].items():
 assert listing['returncode']==0
 available={line.removesuffix(': test')for line in(A/listing['log']).read_text().splitlines()if line.endswith(': test')}
 assert set(r['expected_cases'][target])<=available
assert len(r['test_listings'])==9

old_times={}
for baseline in ['recursive-point-incidence-broad-20260928-v587']:
 for case in json.loads((A/f'{baseline}-terminal.json').read_text())['cases']:old_times[case['name']]=case['elapsed_seconds']
regressions=[];workload_changes=[]
for case in r['cases']:
 old=old_times.get(case['name'])
 if old is not None and old>.1 and case['elapsed_seconds']>old*1.25+.2:regressions.append(dict(name=case['name'],before=old,after=case['elapsed_seconds']))
previous=json.loads((A/'recursive-point-incidence-broad-20260928-v587-repositories-after.json').read_text());repositories={};repos={}
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
review=dict(qualification=f'{prefix}-terminal.json',source_manifest=r['source_manifest'],outer_session_reaped=int(sys.argv[1]),validated_files=len(manifest),selected_geometry_tests=len(r['cases']),checks=len(r['checks']),repos=repos,scope='One retained-field point enumerator serves generic algebraic images, generated points and winding membership; winding owns its ordered crossings; full goal remains active',performance_changes=regressions,workload_changes=workload_changes)
(A/f'{prefix}-reviewed.json').write_text(json.dumps(review,indent=2)+'\n')
message=f"""Unify exact point membership without losing collapsed domains

An independently retained algebraic image of a circle center previously returned Boundary when the inward parallel collapsed to that center. Route algebraic-image and winding membership through the shared retained-field incidence solver and remove the duplicated bivariate point enumerator. Preserve deferred-image and denominator admission, selected root evidence, and winding's separate crossing order and endpoint ownership.

Validation: {len(r['cases'])} release regressions across nine binaries and six static/downstream checks, including an independent collapsed-circle point oracle under both policies and normal sheets. Thirty-three focused results are reused only after matching every source hash and the exact library executable hash. The common path preserves generated point constraints, selected quadratic/quintic fields, stationary contacts and pole barriers.
"""
(A/f'{prefix}-hypercurve-commit.txt').write_text(message)
print('Reviewed',len(manifest),'sources;',len(r['cases']),'domain/family tests;6 checks;',len(repositories),'repositories; performance changes',regressions)
