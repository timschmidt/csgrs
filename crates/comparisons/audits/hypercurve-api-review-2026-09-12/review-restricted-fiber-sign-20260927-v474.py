from pathlib import Path
import hashlib,json,re,subprocess,sys
A=Path(__file__).resolve().parent;W=A.parent;prefix='restricted-fiber-sign-20260927-v474'
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def git(repo,*args):return subprocess.check_output(['git',*args],cwd=W/repo)
r=json.loads((A/f'{prefix}-terminal.json').read_text());assert r['qualification_complete'] and r['all_processes_reaped']
manifest=json.loads((A/r['source_manifest']).read_text())
for name,sha in manifest.items():
 for root in [W,Path(r['source_directory']),A/'build-workspace-20260925']:assert digest(root/name)==sha,(root,name)
targets={}
for b in r['builds']:
 assert b['returncode']==0 and digest(Path(b['binary']['path']))==b['binary']['sha256']
 target=b['crate'] if '--lib' in b['command'] else b['command'][b['command'].index('--test')+1]
 targets[Path(b['binary']['path']).name]=target
actual=set()
for c in r['cases']:
 assert c['returncode']==0 and re.search(r'test result: ok\. 1 passed;', (A/c['log']).read_text())
 actual.add((targets[c['binary']],c['name']))
expected={(target,name)for target,names in r['expected_cases'].items()for name in names}
assert actual==expected and len(actual)==len(r['cases'])==325
assert len(r['builds'])==7 and len(r['checks'])==6
assert all(c['returncode']==0 for c in r['checks'])
prior=json.loads((A/'rational-root-dispatch-20260927-v468-terminal.json').read_text());assert prior['all_processes_reaped'] and prior['qualification_complete']
old_manifest=json.loads((A/prior['source_manifest']).read_text())
assert {n for n,h in manifest.items()if h!=old_manifest[n]}=={'hypercurve/src/bezier_offset.rs'}
for n,h in old_manifest.items():assert digest(Path(prior['source_directory'])/n)==h,n
oracle=json.loads((A/'restricted-fiber-final-oracle-20260927-v475-terminal.json').read_text())
assert oracle['qualification_complete'] and oracle['all_processes_reaped'] and len(oracle['cases'])==1
assert json.loads((A/oracle['source_manifest']).read_text())==manifest
assert set(oracle['trial'])=={'hypercurve/src/bezier_offset.rs'}
for name,sha in manifest.items():assert digest(Path(oracle['source_directory'])/name)==oracle['trial'].get(name,sha),name
captured=(Path(oracle['source_directory'])/'hypercurve/src/bezier_offset.rs').read_bytes()
marker=b'\n#[cfg(test)]\nmod captured_fiber_interval_query {'
assert captured.count(marker)==1 and hashlib.sha256(captured.split(marker)[0]).hexdigest()==manifest['hypercurve/src/bezier_offset.rs']
for b in oracle['builds']:assert b['returncode']==0 and digest(Path(b['binary']['path']))==b['binary']['sha256']
for case in oracle['cases']:assert case['returncode']==0 and re.search(r'test result: ok\. 1 passed;', (A/case['log']).read_text())
paired=json.loads((A/'newton-fillet-paired-20260927-v467-terminal.json').read_text());assert paired['complete'] and paired['all_processes_reaped'] and len(paired['cases'])==12
for case in paired['cases']:assert case['returncode']==0 and re.search(r'test result: ok\. 1 passed;', (A/case['log']).read_text())
assert r['initial_dispatch_check']
for name,medians in paired['medians'].items():
 case=next(c for c in r['cases']if c['name']==name)
 assert case['elapsed_seconds']<=medians['before']*1.1+0.1
paths={'hypercurve':['src/bezier_offset.rs']}
previous=json.loads((A/'rational-root-dispatch-20260927-v468-repositories-after.json').read_text());repositories={};repos={}
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
tests={c['label']:c['passed_tests']for c in r['checks']if 'passed_tests'in c}
review=dict(qualification=f'{prefix}-terminal.json',source_manifest=r['source_manifest'],outer_session_reaped=int(sys.argv[1]),validated_files=len(manifest),selected_geometry_tests=len(actual),checks=len(r['checks']),integration_tests=tests,repos=repos,known_unresolved=r['known_unresolved'],captured_fiber_oracle='restricted-fiber-final-oracle-20260927-v475-terminal.json',captured_oracle_outer_session_reaped=int(sys.argv[2]))
(A/f'{prefix}-reviewed.json').write_text(json.dumps(review,indent=2)+'\n')
assert not r['known_unresolved']
validation="Validation: 325 selected Hypercurve release tests, one independently proved captured fiber-box query, Clippy with all and no default features, formatting, denied-warning docs, editing fuzz compilation, and Hyperbrep compilation. Both complete STRICT and APPROXIMATE_512 extended-fillet re-offset regressions now pass; the earlier STRICT run timed out at 360 seconds.\n"
message='Reuse restricted boxes for exact selected-fiber signs\n\nA predicate can be strictly positive throughout a retained fiber box while direct power-basis interval evaluation still contains zero. Evaluate the polynomial already restricted to that box on the unit square before constructing the local-field GCD. This retains cancellations exposed by the exact affine substitution and reuses the existing interval machinery. Keep the cheap speculative fact checks, original root evidence, exact equality replay, and public API unchanged.\n\n'
(A/f'{prefix}-hypercurve-commit.txt').write_text(message+validation)
print('Reviewed',len(manifest),'sources;',len(actual),'geometry tests;6 checks;',len(repositories),'repositories')
