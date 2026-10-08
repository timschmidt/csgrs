from pathlib import Path
import hashlib,json,re,subprocess,sys
A=Path(__file__).resolve().parent;W=A.parent;prefix='stationary-fillet-public-20260927-v525'
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def git(repo,*args):return subprocess.check_output(['git',*args],cwd=W/repo)
r=json.loads((A/f'{prefix}-terminal.json').read_text());assert r['qualification_complete'] and r['probe_complete'] and r['all_processes_reaped']
manifest=json.loads((A/r['source_manifest']).read_text())
for name,sha in manifest.items():
 for root in [W,Path(r['source_directory']),A/'build-workspace-20260925']:assert digest(root/name)==sha,(root,name)
assert len(r['builds'])==1 and len(r['checks'])==3 and len(manifest)==2047
for b in r['builds']:assert b['returncode']==0 and digest(Path(b['binary']['path']))==b['binary']['sha256']
assert len(r['cases'])==len(set(r['expected_cases']))==30
assert set(c['name']for c in r['cases'])==set(r['expected_cases'])
for c in r['cases']:assert c['passed'] and c['returncode']==0 and re.search(r'test result: ok\. 1 passed;',(A/c['log']).read_text())
assert all(c['returncode']==0 for c in r['checks'])
prior=json.loads((A/'empty-incident-components-20260927-v523-terminal.json').read_text());assert prior['all_processes_reaped'] and prior['qualification_complete']
old_manifest=json.loads((A/prior['source_manifest']).read_text());paths={'hypercurve':['tests/hypercurve_stationary_fillets.rs']}
assert {n for n,h in manifest.items()if h!=old_manifest.get(n)}=={'hypercurve/'+p for p in paths['hypercurve']}
for n,h in old_manifest.items():assert digest(Path(prior['source_directory'])/n)==h,n
broad=json.loads((A/'regular-pair-endpoint-tangents-20260927-v518-terminal.json').read_text())
old_times={c['name']:c['elapsed_seconds']for c in broad['cases']+prior['cases']};regressions=[]
for case in r['cases']:
 old=old_times.get(case['name'])
 if old is not None and old>.1 and case['elapsed_seconds']>old*1.25+.2:regressions.append(dict(name=case['name'],before=old,after=case['elapsed_seconds']))
previous=json.loads((A/'empty-incident-components-20260927-v523-repositories-after.json').read_text());repositories={};repos={}
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
review=dict(qualification=f'{prefix}-terminal.json',source_manifest=r['source_manifest'],outer_session_reaped=int(sys.argv[1]),validated_files=len(manifest),selected_geometry_tests=len(r['cases']),checks=len(r['checks']),repos=repos,scope='Public stationary fillet contact and composition regressions; overall implementation goal remains active',performance_changes=regressions)
(A/f'{prefix}-reviewed.json').write_text(json.dumps(review,indent=2)+'\n')
message='Cover stationary pair fillets through quadratic line charts\n\nKeep the repaired public TrimOrExtend cases as permanent regressions. An affine quadratic line and a stationary polynomial or cusp chart must produce the independently specified exact tangent circle in either path orientation. Reuse the same contact checks as native-line fillets.\n\nValidation: all 30 stationary-fillet release tests, including region/offset compositions and both policies; Clippy with all and no default features; formatting. TrimOnly and interior-cusp parallel-pair closure remain separate open work.\n'
(A/f'{prefix}-hypercurve-commit.txt').write_text(message)
print('Reviewed',len(manifest),'sources;',len(r['cases']),'stationary-fillet tests;3 checks;',len(repositories),'repositories; performance changes',regressions)
