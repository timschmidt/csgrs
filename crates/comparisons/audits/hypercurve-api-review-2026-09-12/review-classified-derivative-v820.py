from pathlib import Path
import hashlib,json,re,subprocess
A=Path(__file__).resolve().parent;W=A.parent;prefix='classified-derivative-v820'
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
expected=json.loads((A/'classified-derivative-cases-v820.json').read_text());expected_set=set()
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
promotion=json.loads((A/'classified-derivative-promotion-v820.json').read_text());changed=set(promotion['promoted']);assert len(changed)==15
old_manifest=json.loads((A/'polynomial-derivative-v817-sources.json').read_text());assert {n for n in manifest if manifest[n]!=old_manifest[n]}==changed
for name in changed:
 assert digest(Path(promotion['candidates'][name]))==manifest[name]==promotion['promoted'][name]
 assert promotion['base'][name]==old_manifest[name]
old_root=A/'source-archives/polynomial-derivative-v817'
image_name='hypercurve/src/bezier_algebraic_image.rs'
before=(old_root/image_name).read_text();after=(W/image_name).read_text()
def between(source,start,end):
 a=source.index(start);b=source.index(end,a);return source[a:b].strip()
# Exact scalar meaning, root replay and every coordinate coefficient formula are unchanged.
start='\npub(crate) fn parameter_representation('
assert before[before.index(start):]==after[after.index(start):]
old_native=between(before,'\nimpl QuadraticBezier2 {','\nimpl RationalQuadraticBezier2 {')
new_native=between(after,'\nimpl QuadraticBezier2 {','\nimpl RationalQuadraticBezier2 {')
assert old_native==new_native.replace('CurveResult<Classification<RationalBezierAlgebraicTangentImage2>>','CurveResult<RationalBezierAlgebraicTangentImage2>')
old_recurrence=between(before,'    let denominator_derivative = derivative_coefficients(&denominator);','        images.push(rational_tangent_image(')
new_recurrence=between(after,'    let denominator_derivative = derivative_coefficients(&denominator);','        match rational_tangent_image(')
assert old_recurrence==new_recurrence
old_queries=between(before,'    pub(crate) fn coordinate_sign(','    /// Returns a compact diagnostic message for failed construction.')
new_queries=between(after,'    pub(crate) fn coordinate_sign(','\n}\n\nimpl QuadraticBezier2 {')
assert old_queries.replace('self.data.retained_expression.as_ref()','self.retained_expression()')==new_queries
state=between(after,'struct RationalBezierAlgebraicTangentImageData {','impl PartialEq for RationalBezierAlgebraicTangentImage2 {')
assert 'definition: RationalTangentDefinition' in state
assert 'Option<' not in state and 'message:' not in state and 'status:' not in state
assert 'Failed(UncertaintyReason)' in after
assert 'RationalCoordinateImagePair::Failed(reason) => {' in after
endpoint=(W/'hypercurve/src/bezier_split_endpoint.rs').read_text()
assert 'tangent: OnceLock<RationalBezierAlgebraicTangentImage2>' in endpoint
assert 'pub fn tangent(&self) -> CurveResult<Classification<&RationalBezierAlgebraicTangentImage2>>' in endpoint
assert 'tangent.set(image)' in endpoint and 'tangent.get_or_init' not in endpoint
for name in manifest:
 if name.endswith(('.rs','.md')):
  source=(W/name).read_text()
  assert not re.search(r'\btry_tangent\b|BezierAlgebraicImageStatus::(?:InvalidParameterEvidence|XImageFailed|YImageFailed)',source),name
# The original source-specific derivative cache ownership and optional higher-order selection remain.
assert 'and_then(transformed_rational_derivative)' in endpoint
for name in ['hypercurve/src/bezier_algebraic_image.rs','hypercurve/src/rational_bezier_general.rs']:
 source=(W/name).read_text();assert 'if let Classification::Decided(images) = &images' in source
 assert '.all(|image| image.status() ==' in source
assert len(expected)==12 and len(expected_set)==323
assert ('hypercurve_bezier_algebraic_image','rational_derivative_images_require_finite_affine_domain') in expected_set
assert ('hypercurve','bezier_split_endpoint::tests::retained_derivatives_replay_nonrational_source_signs') in expected_set
assert ('hypercurve','bezier_split_endpoint::tests::lazy_endpoint_images_preserve_affine_domain_blockers') in expected_set
previous=json.loads((A/'polynomial-derivative-v817-repositories-after.json').read_text());repositories={};repos={}
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
review=dict(qualification=f'{prefix}-terminal.json',source_manifest=r['source_manifest'],outer_session_reaped=reaped['outer_session'],validated_files=len(manifest),tests=len(expected_set),checks=7,repos=repos,scope='Store only exact finite tangent definitions. Return construction blockers through Classification, retain their original reasons, migrate all derivative and endpoint consumers directly, and cache only successful lazy endpoint images.',unresolved='The full exact-geometry implementation goal remains active; this migration does not establish universal composition closure or a measured speedup.')
(A/f'{prefix}-reviewed.json').write_text(json.dumps(review,indent=2)+'\n')
print('Reviewed',len(expected_set),'release cases,twelve pinned binaries,seven checks,2048sources and30repositories')
