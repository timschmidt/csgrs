from pathlib import Path
import hashlib,json,re,subprocess
A=Path(__file__).resolve().parent;W=A.parent;prefix='endpoint-derivative-retention-v852'
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def git(repo,*args):return subprocess.check_output(['git',*args],cwd=W/repo)
reaped=json.loads((A/f'{prefix}-reaped.json').read_text());assert reaped['outer_exit_code']==0
r=json.loads((A/f'{prefix}-terminal.json').read_text());assert r['normal_production_build']and r['qualification_complete']and r['probe_complete']and r['all_processes_reaped']
manifest=json.loads((A/r['source_manifest']).read_text());assert len(manifest)==2048
for name,sha in manifest.items():
 for root in [W,Path(r['source_directory']),A/'build-workspace-20260925']:assert digest(root/name)==sha,(root,name)
assert len(r['builds'])==1 and len(r['test_listings'])==12 and len(r['checks'])==7 and all(c['returncode']==0 for c in r['checks'])
binaries={}
for build in r['builds']:
 assert build['returncode']==0
 artifacts={}
 for line in (A/build['log']).read_text().splitlines():
  try:m=json.loads(line)
  except ValueError:continue
  if m.get('reason')=='compiler-artifact'and m.get('target',{}).get('name')in build['binaries']and m.get('executable'):artifacts[m['target']['name']]=m
 assert set(artifacts)==set(build['binaries'])
 for target,binary in build['binaries'].items():
  assert digest(Path(artifacts[target]['executable']))==binary['sha256']==digest(Path(binary['path']))
  assert target not in binaries;binaries[target]=binary
expected=json.loads((A/'endpoint-derivative-retention-cases-v852.json').read_text());expected_set=set()
for target,listing in r['test_listings'].items():
 assert listing['returncode']==0
 available={s.removesuffix(': test')for s in(A/listing['log']).read_text().splitlines()if s.endswith(': test')}
 selected=set(expected[target])if expected[target]is not None else available
 assert selected<=available and selected==set(r['expected_cases'][target]);expected_set.update((target,name)for name in selected)
assert len(r['cases'])==len(expected_set)
assert expected_set=={(c['target'],c['name'])for c in r['cases']}and len({c['log']for c in r['cases']})==len(expected_set)
for c in r['cases']:
 assert c['passed']and c['returncode']==0 and 'test result: ok. 1 passed;'in(A/c['log']).read_text()
 assert c['command'][0]==binaries[c['target']]['path']and '--include-ignored'in c['command']
promotion=json.loads((A/'endpoint-derivative-retention-promotion-v852.json').read_text());changed=set(promotion['promoted']);assert len(changed)==2
old_manifest=json.loads((A/'derivative-jet-v841-sources.json').read_text());assert {n for n in manifest if manifest[n]!=old_manifest[n]}==changed
for name in changed:
 assert digest(Path(promotion['candidates'][name]))==manifest[name]==promotion['promoted'][name]
 assert promotion['base'][name]==old_manifest[name]
old_root=A/'source-archives/derivative-jet-v841'
name='hypercurve/src/bezier_arrangement.rs';before=(old_root/name).read_text();after=(W/name).read_text()
a0=before.index('        let second_derivative = match image.second_derivative() {');b0=before.index('        return Classification::Decided(Some(RetainedEndpointSideData {',a0)
a1=after.index('        // Higher derivatives are optional projected coordinates here.');b1=after.index('        return Classification::Decided(Some(RetainedEndpointSideData {',a1)
assert before[:a0]==after[:a1]
tail=after[b1:];test_start=tail.index('    #[test]\n    fn optional_unprojected_derivatives_do_not_block_endpoint_setup()');test_end=tail.index('    #[test]\n    fn lazy_polynomial_endpoint_derivatives_match_eager_images()',test_start)
assert before[b0:]==tail[:test_start]+tail[test_end:]
assert after[a1:b1].count('.and_then(retained_algebraic_tangent)')==2
name='hypercurve/src/bezier_split_endpoint.rs';before=(old_root/name).read_text();after=(W/name).read_text()
assert 'transformed_rational_derivative'not in after
assert after.count('let second_derivative = derivatives.next();')==2
assert after.count('let third_derivative = derivatives.next();')==2
assert 'fn lazy_endpoint_images_preserve_affine_domain_blockers()'in after
assert before[before.index('    pub fn is_exact('):before.index('\nfn transformed_rational_derivative(')]==after[after.index('    pub fn is_exact('):after.index('\n#[cfg(test)]')]
assert len(expected)==12 and len(expected_set)==327
assert ('hypercurve','bezier_split_endpoint::tests::rational_endpoints_retain_nonrational_higher_derivatives')in expected_set
assert ('hypercurve','bezier_arrangement::endpoint_adjacency_tests::optional_unprojected_derivatives_do_not_block_endpoint_setup')in expected_set
replay='endpoint-derivative-retention-replay-v853';previous_probe='endpoint-derivative-retention-probe-v847';source='endpoint-derivative-retention-probe-v847.rs'
p=json.loads((A/f'{replay}-terminal.json').read_text());reap=json.loads((A/f'{replay}-reaped.json').read_text());old_probe=json.loads((A/f'{previous_probe}-terminal.json').read_text())
assert reap['outer_exit_code']==0 and p['probe_complete']and p['all_processes_reaped']
assert p['baseline']==r['source_manifest']and all(v['returncode']==0 for v in p['processes'])
assert digest(A/source)==p['source_sha256']==old_probe['source_sha256']
assert digest(Path(p['rlib']))==p['rlib_sha256']
assert digest(A/f'{replay}-binary')==p['binary_sha256']
assert digest(A/f'{previous_probe}-binary')==old_probe['binary_sha256']
def probe_rows(prefix):
 result={}
 for line in(A/f'{prefix}-run.log').read_text().splitlines():
  if not line.startswith('policy='):continue
  row=dict(piece.split('=',1)for piece in line.split());key=tuple(int(row[k])for k in ['policy','family','order']);assert key not in result;result[key]=row
 return result
old_rows=probe_rows(previous_probe);new_rows=probe_rows(replay);assert set(old_rows)==set(new_rows)and len(new_rows)==12
assert sum(row['endpoint_present']=='false'for row in old_rows.values())==8
for key,row in new_rows.items():
 assert row['direct_retained']==old_rows[key]['direct_retained']=='true'
 assert row['endpoint_present']==row['same_image']=='true'
previous=json.loads((A/'derivative-jet-v841-repositories-after.json').read_text());repositories={};repos={}
for name,old in previous.items():
 head=git(name,'rev-parse','HEAD').decode().strip();status=git(name,'status','--porcelain=v1').decode();assert head==old['head'],name
 paths=sorted(p.removeprefix(name+'/')for p in changed if p.startswith(name+'/'))
 if paths:
  assert status==''.join(' M '+p+'\n'for p in paths),status
  assert not git(name,'diff','--cached','--name-only').strip();subprocess.run(['git','diff','--check'],cwd=W/name,check=True)
  repos[name]=dict(parent=head,paths={p:manifest[name+'/'+p]for p in paths})
 else:assert not status,name
 repositories[name]=dict(head=head,status=status)
(A/f'{prefix}-repositories-before.json').write_text(json.dumps(repositories,indent=2)+'\n')
review=dict(qualification=f'{prefix}-terminal.json',source_manifest=r['source_manifest'],outer_session_reaped=reaped['outer_session'],validated_files=len(manifest),tests=len(expected_set),checks=7,repos=repos,scope='Preserve all exact rational endpoint higher derivatives instead of dropping retained expressions. Make optional coordinate extraction nonblocking during endpoint setup; source-based derivative demand, all actual ordering predicates, original pole checks and success-only caches are unchanged. Verify independent analytic signs and a stationary endpoint requiring an unprojected second derivative.',unresolved='The full exact-geometry implementation goal remains active; this migration does not establish universal composition closure or a measured speedup.')
(A/f'{prefix}-reviewed.json').write_text(json.dumps(review,indent=2)+'\n')
print('Reviewed',len(expected_set),'release cases,twelve pinned binaries,seven checks,2048sources and30repositories')
