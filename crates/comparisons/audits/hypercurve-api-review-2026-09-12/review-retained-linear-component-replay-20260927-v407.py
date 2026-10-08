from pathlib import Path
import hashlib,json,re,subprocess
A=Path(__file__).resolve().parent;W=A.parent;prefix='retained-linear-component-replay-20260927-v407'
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
assert current_count==239
for prior_prefix in ['retained-linear-component-replay-20260927-v406']:
 prior=json.loads((A/f'{prior_prefix}-terminal.json').read_text());assert prior['all_processes_reaped'];assert manifest==json.loads((A/prior['source_manifest']).read_text())
 for name,sha in manifest.items():assert digest(Path(prior['source_directory'])/name)==sha,(prior_prefix,name)
 assert digest(Path(prior['builds'][0]['binary']['path']))==prior['builds'][0]['binary']['sha256']==r['builds'][0]['binary']['sha256']
 for case in r['cases']:
  if case.get('reused_from')==f'{prior_prefix}-terminal.json':assert any(old['name']==case['name'] and old['log']==case['log'] and old['returncode']==0 for old in prior['cases'])
 for check in r['checks']:
  if check.get('reused_from')==f'{prior_prefix}-terminal.json':assert any({k:v for k,v in check.items()if k!='reused_from'}==old for old in prior['checks'])
repo=W/'hypercurve';paths=['src/bezier_offset.rs','src/curve.rs','src/curve_fillet.rs']
def git(*args,cwd=repo):return subprocess.check_output(['git',*args],cwd=cwd)
assert git('rev-parse','HEAD').decode().strip()=='46f278dacb5b8556b4196a09e22d1721b20695cb'
assert git('diff','--name-only').decode().splitlines()==paths
assert not git('diff','--cached','--name-only').strip()
assert git('status','--porcelain=v1').decode()==''.join(' M '+p+'\n'for p in paths)
subprocess.run(['git','diff','--check'],cwd=repo,check=True)
previous=json.loads((A/'fillet-component-witnesses-20260927-v401-repositories-after.json').read_text());repositories={}
for name,old in previous.items():
 head=git('rev-parse','HEAD',cwd=W/name).decode().strip();status=git('status','--porcelain=v1',cwd=W/name).decode();assert head==old['head'],name
 if name!='hypercurve':assert not status,name
 repositories[name]=dict(head=head,status=status)
(A/f'{prefix}-repositories-before.json').write_text(json.dumps(repositories,indent=2)+'\n')
record=dict(qualification=f'{prefix}-terminal.json',source_manifest=r['source_manifest'],outer_session_reaped=20622,validated_files=len(manifest),selected_geometry_tests=current_count,checks=len(r['checks']),reused_runs=['retained-linear-component-replay-20260927-v406-terminal.json'],repos={'hypercurve':dict(parent=repositories['hypercurve']['head'],paths={p:manifest['hypercurve/'+p]for p in paths})},known_unresolved=r['known_unresolved'])
(A/f'{prefix}-reviewed.json').write_text(json.dumps(record,indent=2)+'\n')
(A/f'{prefix}-commit.txt').write_text("""Replay constrained straight fillet families through retained chord charts

A retained chord coincident with a nonlinear straight parallel was rejected
although its native-line counterpart admitted an exact constrained fillet.
Reuse the shared component solver with an affine chart over the chord's
actual endpoints. Preserve the nonlinear source's parameters and publish
cuts in the original chord's location representation. Transport a requested
chord parameter through its exact point and recheck all original constraints.

Empty polynomial coefficient rows are valid zeros. Handle their products
before computing coefficient dimensions, including the allocation-fallible
and one-parameter multiplication paths; zero times zero no longer underflows
into a capacity-overflow panic.

Extend the independent semicircle regression across native lines, retained
chords, and offset-returned chords with non-Cartesian endpoints. Cover exact
rotation, reversal, both policies and trim modes, center/point/parameter
selection, connectivity and Boolean equality. Radius alone still requires
an additional exact constraint for a continuous family.

Validation: 239 selected release tests across seven binaries; Clippy with
and without default features, formatting, editing fuzz compilation,
denied-warning docs and Hyperbrep compilation. The two previously recorded
extended-fillet re-offset performance failures remain open and excluded.
""")
print('Reviewed:',len(manifest),'sources;',current_count,'tests; 6 checks;',len(repositories),'repositories')
