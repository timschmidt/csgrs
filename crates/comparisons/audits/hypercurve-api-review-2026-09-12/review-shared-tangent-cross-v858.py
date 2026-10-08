from pathlib import Path
import hashlib,json,re,subprocess
A=Path(__file__).resolve().parent;W=A.parent;prefix='shared-tangent-cross-v858'
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
expected=json.loads((A/'shared-tangent-cross-cases-v858.json').read_text());expected_set=set()
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
promotion=json.loads((A/'shared-tangent-cross-promotion-v858.json').read_text());changed=set(promotion['promoted']);assert len(changed)==2
old_manifest=json.loads((A/'endpoint-derivative-retention-v852-sources.json').read_text());assert {n for n in manifest if manifest[n]!=old_manifest[n]}==changed
for name in changed:
 assert digest(Path(promotion['candidates'][name]))==manifest[name]==promotion['promoted'][name]
 assert promotion['base'][name]==old_manifest[name]
old_root=A/'source-archives/endpoint-derivative-retention-v852'
name='hypercurve/src/bezier_algebraic_image.rs';before=(old_root/name).read_text();after=(W/name).read_text()
a=before.index('        let parameter = match (first_parameter, second_parameter) {',before.index('    pub(crate) fn same_retained_rational_point('));b=before.index('        let (\n            Some((first_x,',a)
old_identity=before[a:b].replace('&other.data.parameter','second_root').replace('&self.data.parameter','first_root')
a=after.index('    let parameter = match (first_parameter, second_parameter) {',after.index("fn shared_image_parameter<'a>"));b=after.index('    Ok(Some(parameter))',a)
assert ''.join(old_identity.split())==''.join(after[a:b].split())
for marker,end in [('        let (\n            Some((first_x,','    /// Compares one affine coordinate'),('    pub(crate) fn coordinate_sign(','impl QuadraticBezier2 {')]:
 start_before=before.index(marker);end_before=before.index(end,start_before);start_after=after.index(marker);end_after=after.index(end,start_after)
 assert before[start_before:end_before]==after[start_after:end_after],marker
# Image construction, retained source expressions and denominator admission do not change.
for marker,end in [('pub(crate) fn rational_derivative_images_from_power_basis(','fn multiply_polynomials('),('fn subtract_polynomials(','\n#[derive(Clone, Debug)]')]:
 start_before=before.index(marker);end_before=before.index(end,start_before);start_after=after.index(marker);end_after=after.index(end,start_after)
 assert before[start_before:end_before]==after[start_after:end_after],marker
assert 'if left.is_empty() || right.is_empty()'in after
new=after[after.index('    pub(crate) fn shared_parameter_cross_sign('):after.index('    pub(crate) fn coordinate_sign(')]
assert 'let strict = policy.strict_counterpart();'in new
assert 'for denominator in [first_denominator, second_denominator]'in new
assert 'Classification::Decided(RealSign::Zero)'in new
assert 'multiply_polynomials(first_x, second_y)'in new and 'multiply_polynomials(first_y, second_x)'in new
assert 'reduce_algebraic_image_polynomial('in new and 'reverse_sign = !reverse_sign'in new
name='hypercurve/src/bezier_tangent_order.rs';before=(old_root/name).read_text();after=(W/name).read_text()
a0=before.index('    let first = BezierAlgebraicTangentVector2::from_image(first);',before.index('pub(crate) fn algebraic_endpoint_tangent_cross_sign('));a1=after.index('    let first = BezierAlgebraicTangentVector2::from_image(first);',after.index('pub(crate) fn algebraic_endpoint_tangent_cross_sign('))
b0=before.index('\n#[cfg(test)]',a0);b1=after.index('\n#[cfg(test)]',a1)
assert before[a0:b0]==after[a1:b1]
assert before[:before.index('pub(crate) fn algebraic_endpoint_tangent_cross_sign(')]==after[:after.index('pub(crate) fn algebraic_endpoint_tangent_cross_sign(')]
assert len(expected)==12 and len(expected_set)==330
for name in ['retained_tangent_determinants_use_selected_source_signs','tangent_determinants_do_not_merge_distinct_selected_roots','scalar_sign_accepts_an_exact_real_arithmetic_result']:
 assert ('hypercurve','bezier_tangent_order::exact_real_status_tests::'+name)in expected_set
previous=json.loads((A/'endpoint-derivative-retention-v852-repositories-after.json').read_text());repositories={};repos={}
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
review=dict(qualification=f'{prefix}-terminal.json',source_manifest=r['source_manifest'],outer_session_reaped=reaped['outer_session'],validated_files=len(manifest),tests=len(expected_set),checks=7,repos=repos,scope='Share the existing point-image selected-parameter identity authority with strict tangent determinants. Preserve both denominator signs and evaluate the combined reduced determinant at the certified root; independent represented-coordinate arithmetic and source/domain construction remain unchanged. Validate analytic positive/negative/zero signs, equivalent isolators, mixed images, signed weights and distinct roots.',unresolved='The full exact-geometry implementation goal remains active; this migration does not establish universal composition closure or a measured speedup.')
(A/f'{prefix}-reviewed.json').write_text(json.dumps(review,indent=2)+'\n')
print('Reviewed',len(expected_set),'release cases,twelve pinned binaries,seven checks,2048sources and30repositories')
