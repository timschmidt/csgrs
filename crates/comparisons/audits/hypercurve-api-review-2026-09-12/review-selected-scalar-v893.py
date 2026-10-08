from pathlib import Path
import hashlib,json,re,subprocess
A=Path(__file__).resolve().parent;W=A.parent;prefix='selected-scalar-v893'
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def git(repo,*args):return subprocess.check_output(['git',*args],cwd=W/repo)
reaped=json.loads((A/f'{prefix}-reaped.json').read_text());assert reaped['outer_exit_code']==0
r=json.loads((A/f'{prefix}-terminal.json').read_text());assert r['normal_production_build']and r['qualification_complete']and r['probe_complete']and r['all_processes_reaped']
manifest=json.loads((A/r['source_manifest']).read_text());assert len(manifest)==2048
for name,sha in manifest.items():
 for root in [W,Path(r['source_directory']),A/'build-workspace-20260925']:assert digest(root/name)==sha,(root,name)
assert len(r['builds'])==2 and len(r['test_listings'])==13 and len(r['checks'])==9 and all(c['returncode']==0 for c in r['checks'])
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
expected=json.loads((A/'selected-scalar-cases-v893.json').read_text());expected_set=set()
for target,listing in r['test_listings'].items():
 assert listing['returncode']==0
 available={line.removesuffix(': test')for line in(A/listing['log']).read_text().splitlines()if line.endswith(': test')}
 selected=set(expected[target])if expected[target]is not None else {n for n in available if target!='hypersolve' or n.startswith('algebraic_fiber::')}
 assert selected<=available and selected==set(r['expected_cases'][target]);expected_set.update((target,name)for name in selected)
assert len(r['cases'])==len(expected_set)==409
assert expected_set=={(c['target'],c['name'])for c in r['cases']}and len({c['log']for c in r['cases']})==len(expected_set)
for c in r['cases']:
 assert c['passed']and c['returncode']==0 and 'test result: ok. 1 passed;'in(A/c['log']).read_text()
 assert c['command'][0]==binaries[c['target']]['path']and '--include-ignored'in c['command']
promotion=json.loads((A/'selected-scalar-promotion-v893.json').read_text());changed=set(promotion['promoted']);assert len(changed)==6
old_manifest=json.loads((A/'retained-vector-order-v886-sources.json').read_text());assert {n for n in manifest if manifest[n]!=old_manifest[n]}==changed
for name in changed:
 assert digest(Path(promotion['candidates'][name]))==manifest[name]==promotion['promoted'][name]
 assert promotion['base'][name]==old_manifest[name]
old_root=A/'source-archives/retained-vector-order-v886'
def function(source,name):
 match=re.search(r'(?m)^([ \t]*)(?:pub(?:\(crate\))? )?fn '+re.escape(name)+r'[(<]',source);assert match,name
 end=re.search(r'(?m)^'+match[1]+r'}',source[match.end():]);assert end,name
 return source[match.start():match.end()+end.end()]
def compact(value):return re.sub(r',([)\]}])',r'\1',re.sub(r'\s+','',value))
old_tangent=(old_root/'hypercurve/src/bezier_tangent_order.rs').read_text();tangent=(W/'hypercurve/src/bezier_tangent_order.rs').read_text()
for fn in ['cross_sign','dot_sign','norm_squared_sign','compare_algebraic_tangent_turn_from_base_impl','compare_algebraic_same_tangent_second_order','compare_algebraic_same_tangent_third_order','compare_algebraic_equal_curvature_third_order','compare_algebraic_same_side_magnitude','power_representation','binary_from_evidence_values','interval_bilinear_sign','represented_sign','refined_represented_sign']:
 assert function(tangent,fn)==function(old_tangent,fn),fn
for fn,end in [('graph_third_derivative_witness','    let normal = binary_from_evidence_values'),('normalized_graph_third_difference','    let (Some(first), Some(second), Some(first_speed), Some(second_speed))'),('same_side_magnitude_difference','    let Some(first_cross_scalar)')]:
 new=function(tangent,fn);start=new.index('    if let Some(evidence) = selected_scalar_calculation(');finish=new.index(end,start);new=new[:start]+new[finish:]
 old=re.sub(r'(\w+)\.scalar\.as_ref\(\)',r'\1.represented_scalar()',function(old_tangent,fn))
 assert compact(new)==compact(old),fn
assert 'pub scalar:'not in tangent and 'source: Option<Box<RetainedTangentBilinear>>'not in tangent
assert 'value: Option<TangentScalar>'in tangent and 'Selected(AlgebraicFieldValue)'in tangent
assert 'pub fn represented_scalar('in tangent and 'pub fn selected_scalar('in tangent
old_image=(old_root/'hypercurve/src/bezier_algebraic_image.rs').read_text();image=(W/'hypercurve/src/bezier_algebraic_image.rs').read_text()
assert image.replace('pub(crate) fn coordinate_polynomials(&self)','fn coordinate_polynomials(&self)')==old_image
# All existing private field arithmetic, fiber algorithms, and proof replay remain unchanged.
old_field=(old_root/'hypersolve/src/algebraic_fiber.rs').read_text();field=(W/'hypersolve/src/algebraic_fiber.rs').read_text()
marker='const LOCAL_FIELD_INTERVAL_SIGN_REFINEMENT_ROUNDS';test_marker='#[cfg(test)]'
old_core=old_field[old_field.index(marker):old_field.index(test_marker)]
core=field[field.index(marker):field.index(test_marker)]
start=core.index('    fn new(',core.index('impl LocalAlgebraicField'));end=core.index('    fn modulus(',start)
a=old_core.index('    fn new(',old_core.index('impl LocalAlgebraicField'));b=old_core.index('    fn modulus(',a)
assert core[:start]+old_core[a:b]+core[end:]==old_core
init=core[start:end]
assert 'is_valid_local_algebraic_field_evidence(root)'in init and 'Ok(Self::from_validated_root(root, policy))'in init
for value in ['root: root.clone()','signed_polynomials: Vec::new()','inverse_polynomials: Vec::new()','certainty: Certainty::Exact','refinement_steps: 0']:assert value in init,value
value_definition=field[field.index('pub struct AlgebraicFieldValue {'):field.index('/// A construction or decision')]
assert value_definition.count('Arc<')==3 and 'LocalAlgebraicField'not in value_definition
assert 'value.exact_value().is_none() && !Arc::ptr_eq(&self.source, &value.source)'in field
admission=function(field,'rational_function');assert admission.index('denominator.is_zero')<admission.index('local_field_element_is_structurally_zero')
assert 'value = self.reduced_value(value)?'in function(field,'finish')
assert 'pub fn rational_coefficients('not in field and 'pub fn coefficients('in field
assert 'compare_algebraic_root_representations_by_difference'in function(field,'require_selected_root')
for probe,blocked in [('source-scalar-order-probe-v889',2),('selected-scalar-replay-v894',0)]:
 assert json.loads((A/f'{probe}-reaped.json').read_text())['outer_exit_code']==0
 data=json.loads((A/f'{probe}-terminal.json').read_text());assert data['probe_complete']and data['all_processes_reaped']and all(p['returncode']==0 for p in data['processes'])
 assert f'complete requests=2 blocked={blocked}'in data['output']
 assert data['output'].count('point_represented=true retained_tangent=true vector_represented=false')==4
 assert data['source_sha256']==digest(A/'source-scalar-order-probe-v889.rs')
 if blocked==0:assert data['baseline']==r['source_manifest']
previous=json.loads((A/'retained-vector-order-v886-repositories-after.json').read_text());repositories={};repos={}
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
review=dict(qualification=f'{prefix}-terminal.json',source_manifest=r['source_manifest'],outer_session_reaped=reaped['outer_session'],validated_files=len(manifest),tests=len(expected_set),checks=9,repos=repos,scope='Own finite selected-field values through existing Hypersolve arithmetic with scoped caches, strict pole/root identity guards, parameter-independent exact constants, reduced retained coefficients and root refinement. Hypercurve retains one scalar definition and uses field values for curvature and both third-order normalization paths; callers migrate directly. The unchanged public V889 probe repairs both traversal blockers without scalar projection.',unresolved='The full exact-geometry implementation goal remains active. Independent selected parameters still need certified transport or extension arithmetic; universal Boolean/offset/fillet/chamfer composition closure and a measured speedup are not established by this change.')
(A/f'{prefix}-reviewed.json').write_text(json.dumps(review,indent=2)+'\n')
print('Reviewed',len(expected_set),'release cases,thirteen pinned binaries,nine checks,2048sources and30repositories')
