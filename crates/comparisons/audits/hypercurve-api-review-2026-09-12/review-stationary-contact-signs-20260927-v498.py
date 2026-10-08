from pathlib import Path
import hashlib,json,re,subprocess,sys
A=Path(__file__).resolve().parent;W=A.parent;prefix='stationary-contact-signs-20260927-v498'
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
assert actual==expected and len(actual)==len(r['cases'])==357
assert len(r['builds'])==8 and len(r['checks'])==6 and len(manifest)==2047
assert all(c['returncode']==0 for c in r['checks'])
prior=json.loads((A/'fillet-retained-domains-20260927-v495-terminal.json').read_text());assert prior['all_processes_reaped'] and prior['qualification_complete']
old_manifest=json.loads((A/prior['source_manifest']).read_text())
paths={'hypercurve':['src/bezier_offset.rs']}
assert {n for n,h in manifest.items()if h!=old_manifest.get(n)}=={'hypercurve/'+p for p in paths['hypercurve']}
for n,h in old_manifest.items():assert digest(Path(prior['source_directory'])/n)==h,n
assert r['initial_dispatch_check']
old_times={c['name']:c['elapsed_seconds']for c in prior['cases']};regressions=[]
for case in r['cases']:
 old=old_times.get(case['name'])
 if old is not None and old>.1 and case['elapsed_seconds']>old*1.25+.2:regressions.append(dict(name=case['name'],before=old,after=case['elapsed_seconds']))
previous=json.loads((A/'fillet-retained-domains-20260927-v495-repositories-after.json').read_text());repositories={};repos={}
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
review=dict(qualification=f'{prefix}-terminal.json',source_manifest=r['source_manifest'],outer_session_reaped=int(sys.argv[1]),validated_files=len(manifest),selected_geometry_tests=len(actual),checks=len(r['checks']),repos=repos,scope='selected regression suite; overall implementation goal remains active',performance_changes=regressions)
(A/f'{prefix}-reviewed.json').write_text(json.dumps(review,indent=2)+'\n')
validation='Validation: 357 selected Hypercurve release tests across eight binaries, all-target Clippy with all and no default features, formatting, denied-warning documentation, editing fuzz compilation, and Hyperbrep compilation. Four new geometric regressions exercise stationary source contacts, a contact beyond a center-locus cusp, and owned endpoint limits from both sides under both policies. Both full extended-fillet re-offset regressions remain passing.\n'
message='Replay parallel tangent orientation at each exact contact\n\nA regular source range can cross a cusp of its parallel. Reusing the derivative sign at its midpoint certified the wrong tangent orientation at an exact contact beyond that cusp. Remove the range-wide sign argument from rational and general pair replay, and share contact-local derivative evidence with vector tangent queries. At stationary range endpoints, replay the first nonzero polynomial Taylor coefficient toward the owned side without projecting another scalar image or isolating another root.\n\n'
(A/f'{prefix}-hypercurve-commit.txt').write_text(message+validation)
print('Reviewed',len(manifest),'sources;',len(actual),'geometry tests;6 checks;',len(repositories),'repositories; performance changes',regressions)
