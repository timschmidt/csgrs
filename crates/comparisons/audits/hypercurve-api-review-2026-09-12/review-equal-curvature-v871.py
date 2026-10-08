from pathlib import Path
import hashlib,json,re,subprocess
A=Path(__file__).resolve().parent;W=A.parent;prefix='equal-curvature-v871'
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
expected=json.loads((A/'equal-curvature-cases-v871.json').read_text());expected_set=set()
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
promotion=json.loads((A/'equal-curvature-promotion-v871.json').read_text());changed=set(promotion['promoted']);assert len(changed)==4
old_manifest=json.loads((A/'same-side-reflection-v867-sources.json').read_text());assert {n for n in manifest if manifest[n]!=old_manifest[n]}==changed
for name in changed:
 assert digest(Path(promotion['candidates'][name]))==manifest[name]==promotion['promoted'][name]
 assert promotion['base'][name]==old_manifest[name]
old_root=A/'source-archives/same-side-reflection-v867'
def rename_fields(text):
 for old,new in [('first_curvature_cross','first_side_witness'),('second_curvature_cross','second_side_witness'),('magnitude_difference','normalized_difference')]:text=re.sub(r'\b'+old+r'\b',new,text)
 return text
name='hypercurve/src/bezier_tangent_order.rs';before=rename_fields((old_root/name).read_text());after=(W/name).read_text()
before=before.replace("First candidate `cross(B'(t), B''(t))` sign evidence.","First candidate's signed normal derivative witness at the compared order.").replace("Second candidate `cross(B'(t), B''(t))` sign evidence.","Second candidate's signed normal derivative witness at the compared order.").replace('Same-side curvature-magnitude difference after clearing speed\n    /// denominators.','Difference of normalized derivative witnesses after clearing positive\n    /// speed denominators, squared when comparing curvature magnitudes.')
a=after.index('/// Compares regular branches whose common nonzero signed curvature');b=after.index('#[derive(Clone, Copy, Debug, Eq, PartialEq)]\nenum ScalarSignStatus',a)
assert after[:a]+after[b:]==before
new=after[a:b]
for text in ['norm_squared_sign(first[0], policy)','norm_squared_sign(second[0], policy)','ScalarSignStatus::Positive => continue','cross_sign(tangent, jerk, policy, true)','cross_sign(tangent, acceleration, policy, true)','dot_sign(tangent, acceleration, policy, true)','Some(&Real::from(3))','power_representation(first_speed, 3, policy)','power_representation(second_speed, 3, policy)']:assert text in new,text
assert 'Rational::' not in new and 'to_rational' not in new
# Every pre-existing construction/sign/arithmetic kernel remains unchanged;
# the new graph numerator delegates all scalar operations to that authority.
name='hypercurve/src/bezier_arrangement.rs';before=rename_fields((old_root/name).read_text());after=(W/name).read_text()
for start,end in [('fn retained_algebraic_derivative(','fn retained_algebraic_same_tangent_evidence_to_turn('),('fn compare_same_tangent_third_order(','fn turn_half('),('fn retained_endpoint_side_data(','#[cfg(test)]')]:
 a=before.index(start);b=before.index(end,a);c=after.index(start);d=after.index(end,c);assert before[a:b]==after[c:d],start
start=after.index('                            let zero_curvature');end=after.index('                            retained_algebraic_same_tangent_evidence_to_turn(',start);guard=after[start:end]
assert 'normalized_difference'in guard and guard.count('Ordering::Equal')==3
assert 'zero_curvature ||'in ' '.join(guard.split()) and 'BezierAlgebraicSameTangentOrderStatus::SameDirection'in guard
start=after.index('fn compare_equal_curvature_third_order(');end=after.index('fn compare_same_tangent_third_order(',start);new=after[start:end]
for text in ['Some(RealSign::Positive)','Real::from(3)','dot_vectors(tangent, acceleration)','cube(&second_speed)','cube(&first_speed)']:assert text in new,text
name='hypercurve/tests/hypercurve_bezier_tangent_order.rs';before=rename_fields((old_root/name).read_text());after=(W/name).read_text();assert ''.join(before.split())==''.join(after.split())
name='hypercurve/tests/hypercurve_bezier_arrangement.rs';before=(old_root/name).read_text();after=(W/name).read_text();a=after.index('#[test]\nfn tangent_ordered_traversal_resolves_equal_nonzero_curvature()');b=after.index('#[test]',a+len('#[test]'))
assert after[:a]+after[b:]==before
assert len(expected_set)==337
assert ('hypercurve','bezier_arrangement::endpoint_adjacency_tests::equal_curvature_graph_jets_ignore_source_acceleration')in expected_set
assert ('hypercurve_bezier_arrangement','tangent_ordered_traversal_resolves_equal_nonzero_curvature')in expected_set
for probe,blocked in [('equal-curvature-probe-v870',4),('equal-curvature-replay-v872',0)]:
 assert json.loads((A/f'{probe}-reaped.json').read_text())['outer_exit_code']==0
 data=json.loads((A/f'{probe}-terminal.json').read_text());assert data['probe_complete']and data['all_processes_reaped']and all(p['returncode']==0 for p in data['processes'])
 assert f'complete arrangement_requests=4 blocked={blocked} certified_booleans=8'in data['output']
 assert data['source_sha256']==digest(A/'equal-curvature-probe-v870.rs')
 if blocked==0:assert data['baseline']==r['source_manifest']
previous=json.loads((A/'same-side-reflection-v867-repositories-after.json').read_text());repositories={};repos={}
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
review=dict(qualification=f'{prefix}-terminal.json',source_manifest=r['source_manifest'],outer_session_reaped=reaped['outer_session'],validated_files=len(manifest),tests=len(expected_set),checks=7,repos=repos,scope='Extend native and represented third-order branch ordering to certified equal nonzero curvature using the graph derivative J/S^3, including tangential acceleration and positive source-speed normalization. Preserve lazy higher-derivative demand and all existing zero-curvature/arithmetic kernels. Rename ambiguous curvature-only report fields directly. Independent exact polynomial and pi-coefficient graph oracles; unchanged public replay repairs four arrangement blockers while preserving eight certified Booleans.',unresolved='The full exact-geometry implementation goal remains active; this migration does not establish universal composition closure or a measured speedup.')
(A/f'{prefix}-reviewed.json').write_text(json.dumps(review,indent=2)+'\n')
print('Reviewed',len(expected_set),'release cases,twelve pinned binaries,seven checks,2048sources and30repositories')
