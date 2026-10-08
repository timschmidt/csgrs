from pathlib import Path
import hashlib,json,re,subprocess,sys
A=Path(__file__).resolve().parent;W=A.parent;prefix='independent-oblique-fillet-20260927-v436'
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def verify_run(prefix,current):
 r=json.loads((A/f'{prefix}-terminal.json').read_text());assert r['qualification_complete'] and r['all_processes_reaped']
 manifest=json.loads((A/r['source_manifest']).read_text())
 roots=[Path(r['source_directory'])]
 if current:roots += [W,A/'build-workspace-20260925']
 for n,h in manifest.items():
  for root in roots:assert digest(root/n)==h,(root,n)
 targets={}
 for b in r['builds']:
  assert b['returncode']==0;binary=Path(b['binary']['path']);assert digest(binary)==b['binary']['sha256']
  cmd=b['command'];target=b['crate'] if '--lib' in cmd else cmd[cmd.index('--test')+1];targets[binary.name]=target
 actual=set()
 for c in r['cases']:
  assert c['returncode']==0;actual.add((targets[c['binary']],c['name']))
  assert re.search(r'test result: ok\. 1 passed;', (A/c['log']).read_text()),c['name']
 expected={(target,name)for target,names in r['expected_cases'].items()for name in names}
 assert actual==expected and len(actual)==len(r['cases'])
 assert len(r['builds'])==len(r['expected_cases']) and len(r['checks'])==6
 assert all(c['returncode']==0 for c in r['checks'])
 return r,manifest,len(actual)
r,manifest,current_count=verify_run(prefix,True)
assert current_count==323
for prior_prefix in ['independent-oblique-fillet-20260927-v435']:
 prior=json.loads((A/f'{prior_prefix}-terminal.json').read_text());assert prior['all_processes_reaped'];assert manifest==json.loads((A/prior['source_manifest']).read_text())
 for name,sha in manifest.items():assert digest(Path(prior['source_directory'])/name)==sha,(prior_prefix,name)
 assert digest(Path(prior['builds'][0]['binary']['path']))==prior['builds'][0]['binary']['sha256']==r['builds'][0]['binary']['sha256']
 for case in r['cases']:
  if case.get('reused_from')==f'{prior_prefix}-terminal.json':assert any(old['name']==case['name'] and old['log']==case['log'] and old['returncode']==0 for old in prior['cases'])
 for check in r['checks']:
  if check.get('reused_from')==f'{prior_prefix}-terminal.json':assert any({k:v for k,v in check.items()if k!='reused_from'}==old for old in prior['checks'])
repo=W/'hypercurve';paths=['src/bezier_offset.rs','src/bezier_region.rs','src/curve.rs','src/curve_fillet.rs','src/curve_region_boolean.rs','src/curve_support_intersection.rs']
def git(*args,cwd=repo):return subprocess.check_output(['git',*args],cwd=cwd)
assert git('rev-parse','HEAD').decode().strip()=='922a155e059ce12feb5042ed8b65a4c2e26cafe4'
assert git('diff','--name-only').decode().splitlines()==paths
assert not git('diff','--cached','--name-only').strip()
assert git('status','--porcelain=v1').decode()==''.join(' M '+p+'\n'for p in paths)
subprocess.run(['git','diff','--check'],cwd=repo,check=True)
previous=json.loads((A/'point-similarity-composition-20260927-v410-repositories-after.json').read_text());repositories={}
for name,old in previous.items():
 head=git('rev-parse','HEAD',cwd=W/name).decode().strip();status=git('status','--porcelain=v1',cwd=W/name).decode();assert head==old['head'],name
 if name!='hypercurve':assert not status,name
 repositories[name]=dict(head=head,status=status)
(A/f'{prefix}-repositories-before.json').write_text(json.dumps(repositories,indent=2)+'\n')
record=dict(qualification=f'{prefix}-terminal.json',source_manifest=r['source_manifest'],outer_session_reaped=int(sys.argv[1]),validated_files=len(manifest),selected_geometry_tests=current_count,checks=len(r['checks']),reused_runs=['independent-oblique-fillet-20260927-v435-terminal.json'],repos={'hypercurve':dict(parent=repositories['hypercurve']['head'],paths={p:manifest['hypercurve/'+p]for p in paths})},known_unresolved=r['known_unresolved'])
(A/f'{prefix}-reviewed.json').write_text(json.dumps(record,indent=2)+'\n')
(A/f'{prefix}-commit.txt').write_text('Replay independent oblique chord fillet families through exact charts\n\nMixed chord/parallel coincidence discarded its regular sample and rejected\nretained chords without a represented affine support. Retain that witness\nand reuse the existing component solver through an exact source chart,\nthen restore original chord locations and recheck all supplied constraints.\nRadius-only requests for a surviving family require an exact center/contact.\n\nReuse domain-certified rational PH images with original parameter charts\nand normal constraints before constructing radical pair equations. Unresolved\ntangent forms and displaced coordinates reuse existing projective predicates;\ncertified native interval decisions remain first. Angular ordering reuses\nreference collinearity rather than recomputing candidate determinants.\nAdmit regular incident bridges outside [0,1] using the existing ordered-interval constructor, with\nunchanged source-pole and normal-regularity checks. No public curve variant\nor compatibility interface is added.\n\nValidation: 323 selected release tests across seven binaries and six checks:\nClippy in both feature modes, formatting, editing fuzz compilation,\ndenied-warning documentation and Hyperbrep compilation. New regressions\ncover 80 constrained fillet edits with Boolean/offset reentry, exact signs\nbelow 2^-512, swapped normal constraints and exterior retained endpoints.\nThe nonlinear selected-corner chamfer passes in 4.82 seconds (4.72 prior).\nThe quartic parallel chamfer passes in 81.02 seconds (111.46 prior).\nTwo previously recorded extended-fillet re-offset performance failures\nremain open and excluded from this passing scope.\n')
print('Reviewed:',len(manifest),'sources;',current_count,'tests; 6 checks;',len(repositories),'repositories')
