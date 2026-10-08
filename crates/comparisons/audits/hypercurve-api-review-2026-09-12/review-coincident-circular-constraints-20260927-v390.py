from pathlib import Path
import hashlib,json,re,subprocess
A=Path(__file__).resolve().parent;W=A.parent;prefix='coincident-circular-constraints-20260927-v390'
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
broad,broad_manifest,broad_count=verify_run('coincident-circular-constraints-20260927-v389',False)
assert current_count==86 and broad_count==234
assert [n for n,h in manifest.items()if h!=broad_manifest[n]]==['hypercurve/src/curve_fillet.rs']
repo=W/'hypercurve';paths=['src/bezier_offset.rs','src/curve.rs','src/curve_corner_chain.rs','src/curve_fillet.rs']
def git(*args,cwd=repo):return subprocess.check_output(['git',*args],cwd=cwd)
assert git('rev-parse','HEAD').decode().strip()=='1bd0348bb7b848e85f9085932a30033675f59e2f'
assert git('diff','--name-only').decode().splitlines()==paths
assert not git('diff','--cached','--name-only').strip()
assert git('status','--porcelain=v1').decode()==''.join(' M '+p+'\n'for p in paths)
subprocess.run(['git','diff','--check'],cwd=repo,check=True)
previous=json.loads((A/'retained-wide-field-proofs-20260927-v374-repositories-after.json').read_text());repositories={}
for name,old in previous.items():
 head=git('rev-parse','HEAD',cwd=W/name).decode().strip();status=git('status','--porcelain=v1',cwd=W/name).decode();assert head==old['head'],name
 if name!='hypercurve':assert not status,name
 repositories[name]=dict(head=head,status=status)
(A/f'{prefix}-repositories-before.json').write_text(json.dumps(repositories,indent=2)+'\n')
record=dict(qualification=f'{prefix}-terminal.json',source_manifest=r['source_manifest'],outer_session_reaped=67326,validated_files=len(manifest),selected_geometry_tests=current_count,checks=len(r['checks']),prior_broad_qualification=dict(receipt='coincident-circular-constraints-20260927-v389-terminal.json',outer_session_reaped=31836,tests=broad_count,checks=6,subsequent_changes=['hypercurve/src/curve_fillet.rs']),repos={'hypercurve':dict(parent=repositories['hypercurve']['head'],paths={p:manifest['hypercurve/'+p]for p in paths})},known_unresolved=r['known_unresolved'])
(A/f'{prefix}-reviewed.json').write_text(json.dumps(record,indent=2)+'\n')
(A/f'{prefix}-commit.txt').write_text('Retain exact constrained fillets on coincident circular supports\n\nRequire a center or contact when coincident circular offsets leave a\nnonzero fillet family. Replay center, point and authored-parameter\nconstraints through the common cut and reconstruction authority.\nClassify actual surviving rational intervals, including full/major native\nsweeps, signed-radius sheets and isolated source-chart endpoints.\n\nPreserve the opposed-tangent proof instead of reconstructing a semicircle\nthrough general incidence. Extend the shared concentric-normal constructor\nto arbitrary retained centers using the existing chord-normal frame.\nCarry each solved fragment index with its authored domain so reconstruction\ncannot evaluate a circular contact on a neighboring NURBS line span.\n\nCover both policies, reversal, trim/extension, retained centers and contacts,\nirrational interval bounds, exact contact positions, independent-region\nBoolean equality and re-offset. No new public type or compatibility layer.\n\nValidation: 234 selected release tests and six checks for the shared fixes;\nafter the final surviving-domain correction, 86 affected tests and all six\nchecks pass again (Clippy both feature modes, formatting, editing fuzz,\ndenied-warning docs and Hyperbrep). The two previously recorded full\nextended-fillet re-offset performance failures remain unresolved.\n')
print('Reviewed:',len(manifest),'sources;',current_count,'final tests;',broad_count,'prior broad tests; 6 final checks;',len(repositories),'repositories')
