from pathlib import Path
import hashlib,json,re,subprocess
A=Path(__file__).resolve().parent;W=A.parent;prefix='retained-vector-order-v884'
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
expected=json.loads((A/'retained-vector-order-cases-v884.json').read_text());expected_set=set()
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
promotion=json.loads((A/'retained-vector-order-promotion-v884.json').read_text());changed=set(promotion['promoted']);assert len(changed)==7
old_manifest=json.loads((A/'mixed-endpoint-order-v877-sources.json').read_text());assert {n for n in manifest if manifest[n]!=old_manifest[n]}==changed
for name in changed:
 assert digest(Path(promotion['candidates'][name]))==manifest[name]==promotion['promoted'][name]
 assert promotion['base'][name]==old_manifest[name]
old_root=A/'source-archives/mixed-endpoint-order-v877'
def function(source,name):
 match=re.search(r'(?m)^([ \t]*)(?:pub(?:\(crate\))? )?fn '+re.escape(name)+r'[(<]',source);assert match,name
 end=re.search(r'(?m)^'+match[1]+r'}',source[match.end():]);assert end,name
 return source[match.start():match.end()+end.end()]
def compact(value):return re.sub(r'\s+','',value)
old_arr=(old_root/'hypercurve/src/bezier_arrangement.rs').read_text();arr=(W/'hypercurve/src/bezier_arrangement.rs').read_text()
old_tangent=(old_root/'hypercurve/src/bezier_tangent_order.rs').read_text();tangent=(W/'hypercurve/src/bezier_tangent_order.rs').read_text()
old_image=(old_root/'hypercurve/src/bezier_algebraic_image.rs').read_text();image=(W/'hypercurve/src/bezier_algebraic_image.rs').read_text()
for fn in ['compare_retained_turn_from_base','compare_retained_same_tangent_second_order','compare_retained_algebraic_same_tangent_third_order','compare_same_tangent_second_order','compare_equal_curvature_third_order','compare_same_tangent_third_order','retained_tangent_adjacency']:
 assert function(arr,fn)==function(old_arr,fn),fn
assert function(tangent,'negate_algebraic_root').replace('pub(crate) ','',1)==function(old_arr,'negate_algebraic_root')
assert arr.count('type EndpointAdjacency =')==1 and 'type EndpointAdjacency'not in tangent
assert '.and_then(retained_algebraic_tangent)'not in arr
assert '-> RetainedTangentVector'in function(arr,'retained_algebraic_tangent')
# The existing coordinate arithmetic and interval kernels remain unchanged.
for fn,guard_end,renames in [('cross_sign','    if !retain_scalar',{'left_x':'left.dx()','left_y':'left.dy()','right_x':'right.dx()','right_y':'right.dy()'}),('dot_sign','    if !retain_scalar',{'left_x':'left.dx()','left_y':'left.dy()','right_x':'right.dx()','right_y':'right.dy()'}),('norm_squared_sign','    let dx_squared',{'x':'vector.dx()','y':'vector.dy()'})]:
 actual=function(tangent,fn);old=function(old_tangent,fn)
 a=actual.index('    let ');b=actual.index(guard_end,a);actual=actual[:a]+actual[b:]
 for token,value in renames.items():actual=re.sub(r'\b'+token+r'\b',value,actual)
 assert compact(actual)==compact(old),fn
for fn in ['compare_algebraic_tangent_turn_from_base_impl','compare_algebraic_same_tangent_second_order','compare_algebraic_same_tangent_third_order','compare_algebraic_equal_curvature_third_order','graph_third_derivative_witness','normalized_graph_third_difference','same_side_magnitude_difference','power_representation','binary_from_evidence_values','interval_bilinear_sign']:
 assert function(tangent,fn)==function(old_tangent,fn),fn
assert 'pub fn from_image(image: &RationalBezierAlgebraicTangentImage2) -> Self'in tangent
assert 'pub fn represented_coordinates('in tangent
assert not re.search(r'pub (?:const )?fn (?:dx|dy)\(',tangent)
assert 'source: Option<Box<RetainedTangentBilinear>>'in tangent
assert 'source: Some(Box::new(RetainedTangentBilinear'in tangent
assert 'None if evidence.scalar.is_none() && evidence.source.is_none()'in tangent
assert 'Coordinates(Arc<'in tangent
# Common selected-root identity and denominator certification are unchanged.
old_common=function(old_image,'shared_parameter_cross_sign');common=function(image,'shared_parameter_bilinear_sign')
old_guard=old_common[:old_common.index('        let determinant')]
guard=common[:common.index('        let determinant')].replace('shared_parameter_bilinear_sign','shared_parameter_cross_sign').replace('        dot: bool,\n','')
assert guard==old_guard
for fn in ['shared_image_parameter','coordinate_sign','rational_coordinate_image_pair','reduce_algebraic_image_polynomial','multiply_polynomials','subtract_polynomials']:
 assert function(image,fn)==function(old_image,fn),fn
negated=function(image,'negated_retained_expression')
assert 'denominator: expression.denominator.clone()'in negated and negated.count('.map(|value| -value)')==2
name='hypercurve/tests/hypercurve_bezier_arrangement.rs';before=(old_root/name).read_text();after=(W/name).read_text();assert after.startswith(before.rstrip())
assert len(expected_set)==342
for target,name in [('hypercurve','bezier_tangent_order::exact_real_status_tests::source_tangent_angles_retain_replay_and_reversal'),('hypercurve_bezier_arrangement','retained_source_tangents_order_without_coordinate_projection')]:assert(target,name)in expected_set
for probe,blocked,source in [('retained-vector-order-probe-v881',2,'retained-vector-order-probe-v881.rs'),('retained-vector-order-replay-v885',0,'retained-vector-order-probe-v883.rs')]:
 assert json.loads((A/f'{probe}-reaped.json').read_text())['outer_exit_code']==0
 data=json.loads((A/f'{probe}-terminal.json').read_text());assert data['probe_complete']and data['all_processes_reaped']and all(p['returncode']==0 for p in data['processes'])
 assert f'complete requests=2 blocked={blocked}'in data['output']
 assert data['output'].count('point_represented=true retained_tangent=true vector_represented=false')==2
 assert data['source_sha256']==digest(A/source)
 if blocked==0:assert data['baseline']==r['source_manifest']
assert (A/'retained-vector-order-probe-v883.rs').read_text()==(A/'retained-vector-order-probe-v881.rs').read_text().replace('BezierAlgebraicTangentVector2::from_image(tangent).is_some()', 'BezierAlgebraicTangentVector2::from_image(tangent).represented_coordinates().is_some()')
previous=json.loads((A/'mixed-endpoint-order-v877-repositories-after.json').read_text());repositories={};repos={}
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
review=dict(qualification=f'{prefix}-terminal.json',source_manifest=r['source_manifest'],outer_session_reaped=reaped['outer_session'],validated_files=len(manifest),tests=len(expected_set),checks=7,repos=repos,scope='Retain exact derivative images in the tangent vector, expose optional represented coordinate pairs, and migrate all callers directly. Extend the existing selected-source determinant authority to dot/norm and arbitrary exact constant combinations, retaining sign replay operands. Preserve represented-coordinate arithmetic, selected-root identity, denominator guards, demand-driven derivatives and source reversal. The API-observer-only migrated public probe repairs both retained-expression traversal blockers.',unresolved='The full exact-geometry implementation goal remains active; this migration does not establish universal composition closure or a measured speedup.')
(A/f'{prefix}-reviewed.json').write_text(json.dumps(review,indent=2)+'\n')
print('Reviewed',len(expected_set),'release cases,twelve pinned binaries,seven checks,2048sources and30repositories')
