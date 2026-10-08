from pathlib import Path
import hashlib,json,re,subprocess,sys
A=Path(__file__).resolve().parent;W=A.parent;prefix='normalized-root-replay-20260927-v454'
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
assert actual==expected and len(actual)==len(r['cases'])==323
assert len(r['builds'])==7 and len(r['checks'])==14
assert all(c['returncode']==0 for c in r['checks'])
prior=json.loads((A/'normalized-root-replay-20260927-v453-terminal.json').read_text());assert prior['all_processes_reaped']
old_manifest=json.loads((A/prior['source_manifest']).read_text())
assert {n for n,h in manifest.items()if h!=old_manifest[n]}=={'hypersolve/src/root_isolation_monic_tests.rs'}
for n,h in old_manifest.items():assert digest(Path(prior['source_directory'])/n)==h,n
for check in r['checks']:
 if check.get('reused_from'):
  assert check['label'].startswith('hyperreal-')
  assert any({k:v for k,v in check.items()if k!='reused_from'}==old for old in prior['checks'])
oracle=json.loads((A/'normalized-root-oracles-20260927-v455-terminal.json').read_text())
assert oracle['qualification_complete'] and oracle['all_processes_reaped'] and len(oracle['cases'])==3
assert json.loads((A/oracle['source_manifest']).read_text())==manifest
assert set(oracle['trial'])=={'hypersolve/src/root_sign.rs'}
for name,sha in manifest.items():assert digest(Path(oracle['source_directory'])/name)==oracle['trial'].get(name,sha),name
captured=(Path(oracle['source_directory'])/'hypersolve/src/root_sign.rs').read_bytes()
marker=b'\n#[cfg(test)]\nmod captured_quadratic_field_queries {'
assert captured.count(marker)==1 and hashlib.sha256(captured.split(marker)[0]).hexdigest()==manifest['hypersolve/src/root_sign.rs']
for b in oracle['builds']:assert b['returncode']==0 and digest(Path(b['binary']['path']))==b['binary']['sha256']
for case in oracle['cases']:assert case['returncode']==0 and re.search(r'test result: ok\. 1 passed;', (A/case['log']).read_text())
paths={'hyperreal':['src/computable/node/quadratic_tower.rs'],'hypersolve':['src/root_isolation.rs','src/root_isolation_monic_tests.rs','src/root_sign.rs'],'hypercurve':['tests/hypercurve_curve_region_promotion.rs']}
previous=json.loads((A/'independent-oblique-fillet-20260927-v436-repositories-after.json').read_text());repositories={};repos={}
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
review=dict(qualification=f'{prefix}-terminal.json',source_manifest=r['source_manifest'],outer_session_reaped=int(sys.argv[1]),validated_files=len(manifest),selected_geometry_tests=len(actual),checks=len(r['checks']),integration_tests=tests,repos=repos,known_unresolved=r['known_unresolved'],captured_root_oracles='normalized-root-oracles-20260927-v455-terminal.json',captured_oracle_outer_session_reaped=int(sys.argv[2]))
(A/f'{prefix}-reviewed.json').write_text(json.dumps(review,indent=2)+'\n')
validation=f"Validation: {tests['hyperreal-integration']} Hyperreal and {tests['hypersolve-integration']} Hypersolve tests, 323 selected Hypercurve release tests, three independently proved captured root queries, Clippy, formatting, denied-warning docs, editing fuzz compilation, and Hyperbrep compilation. The two original extended-fillet re-offset tests remain unresolved and are excluded from the passing geometry scope.\n"
messages={
'hyperreal':'Fuse exact quadratic arithmetic and reuse signed branches\n\nUse the existing fused rational product sums for quadratic products and norms. Decide signs from equal coefficient signs before constructing a larger norm, and compare opposed rational products without materializing their difference. Preserve principal-root branches, exact cancellation, and general scalar fallback.\n\n',
'hypersolve':'Normalize selected-root remainder chains without losing signs\n\nRemove the initial polynomial quotient and normalize subsequent remainders by positive leading units, reusing compact exact scalar proofs. Trim zero queries before requesting a defining-degree decision. Reuse the existing monic normalization kernel under its general name and update every caller directly. Original root equations, isolators, endpoint ownership and coarser certified replay remain authoritative.\n\n',
'hypercurve':'Check retained fillet charts across all regularized loops\n\nAn extended fillet can regularize into a separate loop. The reflected candidates put that loop on opposite sides of the boundary ordering, so counting only the first loop falsely rejects one result. Check the inserted circle charts across the full normalized region.\n\n'}
for repo,message in messages.items():(A/f'{prefix}-{repo}-commit.txt').write_text(message+validation)
print('Reviewed',len(manifest),'sources;',len(actual),'geometry tests;',tests,';14 checks;',len(repositories),'repositories')
