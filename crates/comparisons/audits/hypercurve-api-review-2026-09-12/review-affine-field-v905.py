from pathlib import Path
import hashlib,json,re,subprocess
A=Path(__file__).resolve().parent;W=A.parent;prefix='affine-field-v905'
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
expected=json.loads((A/'affine-field-cases-v905.json').read_text());expected_set=set()
for target,listing in r['test_listings'].items():
 assert listing['returncode']==0
 available={line.removesuffix(': test')for line in(A/listing['log']).read_text().splitlines()if line.endswith(': test')}
 selected=set(expected[target])if expected[target]is not None else {n for n in available if target!='hypersolve' or n.startswith(('algebraic::','algebraic_fiber::','algebraic_tensor_image::'))}
 assert selected<=available and selected==set(r['expected_cases'][target]);expected_set.update((target,name)for name in selected)
assert len(r['cases'])==len(expected_set)==503
assert expected_set=={(c['target'],c['name'])for c in r['cases']}and len({c['log']for c in r['cases']})==len(expected_set)
for c in r['cases']:
 assert c['passed']and c['returncode']==0 and 'test result: ok. 1 passed;'in(A/c['log']).read_text()
 assert c['command'][0]==binaries[c['target']]['path']and '--include-ignored'in c['command']
promotion=json.loads((A/'affine-field-promotion-v905.json').read_text());changed=set(promotion['promoted']);assert len(changed)==6
old_manifest=json.loads((A/'selected-scalar-v893-sources.json').read_text());assert {n for n in manifest if manifest[n]!=old_manifest[n]}==changed
for name in changed:
 assert digest(Path(promotion['candidates'][name]))==manifest[name]==promotion['promoted'][name]
 assert promotion['base'][name]==old_manifest[name]
old_root=A/'source-archives/selected-scalar-v893'
def function(source,name):
 match=re.search(r'(?m)^([ \t]*)(?:pub(?:\(crate\))? )?fn '+re.escape(name)+r'[(<]',source);assert match,name
 end=re.search(r'(?m)^'+match[1]+r'}',source[match.end():]);assert end,name
 return source[match.start():match.end()+end.end()]
old_tangent=(old_root/'hypercurve/src/bezier_tangent_order.rs').read_text();tangent=(W/'hypercurve/src/bezier_tangent_order.rs').read_text()
for fn in ['cross_sign','dot_sign','norm_squared_sign','compare_algebraic_tangent_turn_from_base_impl','compare_algebraic_same_tangent_second_order','compare_algebraic_same_tangent_third_order','compare_algebraic_equal_curvature_third_order','compare_algebraic_same_side_magnitude','power_representation','binary_from_evidence_values','interval_bilinear_sign','represented_sign','refined_represented_sign','graph_third_derivative_witness','normalized_graph_third_difference','same_side_magnitude_difference','selected_scalar_calculation','scalar_in_field','strict_exact_witness','vector_in_field']:
 assert function(tangent,fn)==function(old_tangent,fn),fn
bilinear=function(tangent,'source_bilinear_evidence')
assert 'evidence.sign.is_none()'in bilinear and 'transported.value.is_some()'in bilinear
old_field=(old_root/'hypersolve/src/algebraic_fiber.rs').read_text();field=(W/'hypersolve/src/algebraic_fiber.rs').read_text()
marker='const LOCAL_FIELD_INTERVAL_SIGN_REFINEMENT_ROUNDS';test_marker='#[cfg(test)]'
assert field[field.index(marker):field.index(test_marker)]==old_field[old_field.index(marker):old_field.index(test_marker)]
value_definition=field[field.index('pub struct AlgebraicFieldValue {'):field.index('/// A construction or decision')]
assert value_definition==old_field[old_field.index('pub struct AlgebraicFieldValue {'):old_field.index('/// A construction or decision')]
assert value_definition.count('Arc<')==3 and 'LocalAlgebraicField'not in value_definition
assert 'DifferentSelectedRoot'not in field and 'fn require_selected_root('not in field
admission=function(field,'rational_function');assert admission.index('parameter_transport')<admission.index('denominator.is_zero')<admission.index('local_field_element_is_structurally_zero')
assert 'value = self.reduced_value(value)?'in function(field,'finish')
assert 'if let Cow::Owned(element)'in function(field,'finish')
transport=function(field,'parameter_transport')
for text in ['compare_algebraic_root_representations_by_difference','PredicatePolicy::STRICT','AlgebraicRootComparisonStatus::InvalidEvidence','algebraic_root_affine_relation(&self.source, root)','algebraic_root_affine_relation(&self.state.root, root)','self.transports','relation.clone()']:assert text in transport,text
horner=function(field,'substitute_parameter')
assert 'self.state.multiply_polynomials(&result, &linear)?'in horner and '.add_polynomials('in horner
root=(W/'hypersolve/src/algebraic.rs').read_text();affine=function(root,'algebraic_root_affine_relation')
assert 'let policy = PredicatePolicy::STRICT;'in affine and 'policy: PredicatePolicy'not in affine
assert 'exact_rational_ref()'not in affine and '.exact_rational_normal_form()'in affine and '.unwrap_or(power)'in affine
assert 'admitted_represented_roots_share_isolated_common_root'in affine and 'Some(true)'in affine
tensor=(W/'hypersolve/src/algebraic_tensor_image.rs').read_text()
construction=function(tensor,'represent_algebraic_tensor_image')
assert construction.index('normalize_tensor_relation(relation.clone())')<construction.index('substitute_affine_axis(')
for probe,blocked in [('affine-source-scalar-probe-v895',2),('affine-field-replay-v906',0)]:
 assert json.loads((A/f'{probe}-reaped.json').read_text())['outer_exit_code']==0
 data=json.loads((A/f'{probe}-terminal.json').read_text());assert data['probe_complete']and data['all_processes_reaped']and all(p['returncode']==0 for p in data['processes'])
 assert f'complete requests=2 blocked={blocked}'in data['output']
 assert data['output'].count('point_represented=true retained_tangent=true vector_represented=false')==4
 assert data['source_sha256']==digest(A/'affine-source-scalar-probe-v895.rs')
 if blocked==0:assert data['baseline']==r['source_manifest']
previous=json.loads((A/'selected-scalar-v893-repositories-after.json').read_text());repositories={};repos={}
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
review=dict(qualification=f'{prefix}-terminal.json',source_manifest=r['source_manifest'],outer_session_reaped=reaped['outer_session'],validated_files=len(manifest),tests=len(expected_set),checks=9,repos=repos,scope='Certify affine parameter transport over arbitrary exact Real coefficients in Hypersolve; normalize exact rational candidate powers without restricting nonrational values. Transport finite field coefficients with reduced Horner substitution, selected-root replay, strict pole checks and scoped successful-proof caches. Hypercurve reuses field arithmetic for unresolved source bilinear signs; native and represented kernels and all derivative normalization formulas stay unchanged. All controlled callers use the strict affine API directly. The unchanged public V895 probe repairs both traversal blockers without scalar projection.',unresolved='The full exact-geometry goal remains active. Nonaffine selected-parameter embeddings, stationary/arbitrary higher-jet ordering and general Boolean/offset/fillet/chamfer composition closure remain unproved; no universal coefficient-size bound or measured speedup is claimed.')
(A/f'{prefix}-reviewed.json').write_text(json.dumps(review,indent=2)+'\n')
print('Reviewed',len(expected_set),'release cases,thirteen pinned binaries,nine checks,2048sources and30repositories')
