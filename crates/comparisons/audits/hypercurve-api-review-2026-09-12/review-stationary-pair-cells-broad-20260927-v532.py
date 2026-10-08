from pathlib import Path
import hashlib,json,re,subprocess,sys
A=Path(__file__).resolve().parent;W=A.parent;prefix='stationary-pair-cells-broad-20260927-v532'
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
assert len(r['cases'])==len(expected)==394
assert {(c['target'],c['name']) for c in r['cases']}==expected
for c in r['cases']:assert c['passed'] and c['returncode']==0 and re.search(r'test result: ok\. 1 passed;',(A/c['log']).read_text())
assert all(c['returncode']==0 for c in r['checks'])
prior=json.loads((A/'regular-cell-ordering-20260927-v528-terminal.json').read_text());assert prior['all_processes_reaped'] and prior['qualification_complete']
old_manifest=json.loads((A/prior['source_manifest']).read_text());paths={'hypercurve':['src/bezier_offset.rs','src/curve.rs','src/curve_fillet.rs','src/curve_parameter_component.rs','tests/hypercurve_stationary_fillets.rs']}
assert {n for n,h in manifest.items()if h!=old_manifest.get(n)}=={'hypercurve/'+p for p in paths['hypercurve']}
for n,h in old_manifest.items():assert digest(Path(prior['source_directory'])/n)==h,n
checked=json.loads((A/'stationary-pair-cell-orientation-20260927-v531-terminal.json').read_text())
assert checked['qualification_complete'] and checked['all_processes_reaped']
assert json.loads((A/checked['source_manifest']).read_text())==manifest
assert len(r['builds'][0]['binaries'])==8 and r['static_check_sources_identical']
assert all(row['reused_from']=='stationary-pair-cell-orientation-20260927-v531-terminal.json' for row in r['checks'])
assert [{key:value for key,value in row.items() if key!='reused_from'} for row in r['checks']]==checked['checks']
old_times={}
for baseline in ['regular-pair-endpoint-tangents-20260927-v518','finite-domain-ownership-20260927-v527','regular-cell-ordering-20260927-v528']:
 for case in json.loads((A/f'{baseline}-terminal.json').read_text())['cases']:old_times[case['name']]=case['elapsed_seconds']
regressions=[];workload_changes=[]
for case in r['cases']:
 old=old_times.get(case['name'])
 if old is not None and old>.1 and case['elapsed_seconds']>old*1.25+.2:regressions.append(dict(name=case['name'],before=old,after=case['elapsed_seconds']))
previous=json.loads((A/'regular-cell-ordering-20260927-v528-repositories-after.json').read_text());repositories={};repos={}
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
review=dict(qualification=f'{prefix}-terminal.json',source_manifest=r['source_manifest'],outer_session_reaped=int(sys.argv[1]),validated_files=len(manifest),selected_geometry_tests=len(r['cases']),checks=len(r['checks']),repos=repos,scope='Owned regular source cells for stationary pair fillets, shared isolated/family frame replay, branch-aware represented constraints, and finite-cell normal proof reuse; full implementation goal and additional generic/ray closure cases remain open',performance_changes=regressions,workload_changes=workload_changes)
(A/f'{prefix}-reviewed.json').write_text(json.dumps(review,indent=2)+'\n')
message='Keep stationary pair fillets on their owned source cells\n\nPartition pair queries at source singularities and original offset cusps while retaining the original curves for cuts. Carry seam ownership and outward extension rays through the shared domain solver. Reuse certified finite-cell normal orientation and one shared source-frame reconstruction for isolated centers and constrained families. Point constraints replay the same branch frame; continuous families continue to require an exact center or contact constraint.\n\nValidation: 394 release regressions across eight binaries, including strict and approximate stationary pair contacts under both trimming modes and path reversals, all four constraint forms, opposite-normal rejection, continuous-family selection, and region/offset compositions. Six static/downstream checks passed on identical inputs.\n'
(A/f'{prefix}-hypercurve-commit.txt').write_text(message)
print('Reviewed',len(manifest),'sources;',len(r['cases']),'domain/family tests;6 checks;',len(repositories),'repositories; performance changes',regressions)
