from pathlib import Path
import hashlib,json,re,subprocess,sys
A=Path(__file__).resolve().parent;W=A.parent;prefix='retained-tangent-pair-broad-20260927-v540'
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
assert len(r['cases'])==len(expected)==396
assert {(c['target'],c['name']) for c in r['cases']}==expected
for c in r['cases']:assert c['passed'] and c['returncode']==0 and re.search(r'test result: ok\. 1 passed;',(A/c['log']).read_text())
assert all(c['returncode']==0 for c in r['checks'])
prior=json.loads((A/'stationary-pair-cells-broad-20260927-v536-terminal.json').read_text());assert prior['all_processes_reaped'] and prior['qualification_complete']
old_manifest=json.loads((A/prior['source_manifest']).read_text());paths={'hypercurve':['src/bezier_offset.rs','src/curve_fillet.rs']}
assert {n for n,h in manifest.items()if h!=old_manifest.get(n)}=={'hypercurve/'+p for p in paths['hypercurve']}
for n,h in old_manifest.items():assert digest(Path(prior['source_directory'])/n)==h,n
checked=json.loads((A/'retained-tangent-pair-replay-20260927-v539-terminal.json').read_text())
assert checked['qualification_complete'] and checked['all_processes_reaped']
assert json.loads((A/checked['source_manifest']).read_text())==manifest
assert len(r['builds'][0]['binaries'])==8 and r['static_check_sources_identical']
assert all(row['reused_from']=='retained-tangent-pair-replay-20260927-v539-terminal.json' for row in r['checks'])
assert [{key:value for key,value in row.items() if key!='reused_from'} for row in r['checks']]==checked['checks']
old_times={}
for baseline in ['stationary-pair-cells-broad-20260927-v536']:
 for case in json.loads((A/f'{baseline}-terminal.json').read_text())['cases']:old_times[case['name']]=case['elapsed_seconds']
regressions=[];workload_changes=[]
for case in r['cases']:
 old=old_times.get(case['name'])
 if case['name'].endswith('::regular_source_frames_retain_unprojectable_selected_parameters'):
  workload_changes.append(dict(name=case['name'],before=old,after=case['elapsed_seconds'],change='also checks cross/dot of independently anchored opposite tangents without global parameter promotion'))
  continue
 if old is not None and old>.1 and case['elapsed_seconds']>old*1.25+.2:regressions.append(dict(name=case['name'],before=old,after=case['elapsed_seconds']))
previous=json.loads((A/'stationary-pair-cells-broad-20260927-v536-repositories-after.json').read_text());repositories={};repos={}
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
review=dict(qualification=f'{prefix}-terminal.json',source_manifest=r['source_manifest'],outer_session_reaped=int(sys.argv[1]),validated_files=len(manifest),selected_geometry_tests=len(r['cases']),checks=len(r['checks']),repos=repos,scope='Retained analytic tangent displacements replay polynomial cross/dot relations in shared parameter evidence before independent endpoint refinement; stationary continuous-family constraint regression; full implementation goal remains active',performance_changes=regressions,workload_changes=workload_changes)
(A/f'{prefix}-reviewed.json').write_text(json.dumps(review,indent=2)+'\n')
message='Replay analytic tangent relations in their retained parameter fields\n\nA stationary reparameterization of the continuous fillet family selected its exact center and contacts, then failed to certify the zero cross product of their tangent witnesses. Cancel shared tangent-displacement anchors and positive speed denominators before signing the source polynomials. Reuse equal retained parameter evidence or the existing native parameter-pair authority, leaving unavailable optional proofs to the general tangent predicate. Share the same frame/displacement check with linear-form replay.\n\nValidation: 396 release regressions across eight binaries and six static/downstream checks. The independent stationary continuous-family fixture covers radius-only constraint requests and center/point/parameter selections under both policies and path directions, with exact contact/center/radius witnesses. A selected-parameter regression verifies opposite displaced tangents without global root promotion.\n'
(A/f'{prefix}-hypercurve-commit.txt').write_text(message)
print('Reviewed',len(manifest),'sources;',len(r['cases']),'domain/family tests;6 checks;',len(repositories),'repositories; performance changes',regressions)
