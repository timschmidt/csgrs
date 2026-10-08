from pathlib import Path
import hashlib,json,re,subprocess
A=Path(__file__).resolve().parent;W=A.parent;prefix='finite-point-image-v788'
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def git(repo,*args):return subprocess.check_output(['git',*args],cwd=W/repo)
reaped=json.loads((A/f'{prefix}-reaped.json').read_text());assert reaped['outer_exit_code']==0
r=json.loads((A/f'{prefix}-terminal.json').read_text());assert r['normal_production_build']and r['qualification_complete']and r['probe_complete']and r['all_processes_reaped']
manifest=json.loads((A/r['source_manifest']).read_text());assert len(manifest)==2048
for name,sha in manifest.items():
 for root in [W,Path(r['source_directory']),A/'build-workspace-20260925']:assert digest(root/name)==sha,(root,name)
assert len(r['builds'])==1 and len(r['test_listings'])==8 and len(r['checks'])==7 and all(c['returncode']==0 for c in r['checks'])
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
expected=json.loads((A/'finite-point-image-cases-v788.json').read_text());expected_set=set()
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
promotion=json.loads((A/'finite-point-image-promotion-v788.json').read_text());changed=set(promotion['promoted']);assert len(changed)==25
old_manifest=json.loads((A/'boundary-provenance-admission-v783-sources.json').read_text());assert {n for n in manifest if manifest[n]!=old_manifest[n]}==changed
for name in changed:
 assert digest(Path(promotion['candidates'][name]))==manifest[name]==promotion['promoted'][name]
 assert promotion['base'][name]==old_manifest[name]
before=(A/'source-archives/boundary-provenance-admission-v783/hypercurve/src/bezier_algebraic_image.rs').read_text();after=(W/'hypercurve/src/bezier_algebraic_image.rs').read_text()
def interval(source,start,end):
 a=source.index(start);b=source.index(end,a);return source[a:b]
point_factory=interval(after,'fn rational_point_image_with_parameter_representation(','pub(crate) fn rational_point_image_from_power_basis(')
assert 'RationalCoordinateImagePair::Failed { reason, .. } => Ok(Classification::Uncertain(reason))' in point_factory
assert point_factory.count('RationalBezierAlgebraicPointImage2::new(')==2
assert 'reason: UncertaintyReason::Boundary' in interval(after,'fn rational_coordinate_image_pair(','fn coordinate_image_from_replay(')
assert 'resolved: OnceLock<RationalBezierAlgebraicPointImage2>' in after
assert 'get_or_init' not in interval(after,'    pub(crate) fn resolved(','    /// Materializes both exact coordinate representations')
for path in ['hypercurve/src/bezier_algebraic_image.rs','hypercurve/src/rational_bezier_general.rs']:
 code=(W/path).read_text();assert '&& image.status() == crate::BezierAlgebraicImageStatus::Transformed' in code
# Non-pole exact Real expressions remain representable without requiring
# rational reconstruction or projected Cartesian coordinates.
for start,end in [('    pub(crate) fn from_retained_expression(', '    pub(crate) fn from_parametric_source('),('    pub(crate) fn from_parametric_source(', '    #[inline(never)]\n    pub(crate) fn resolved(')]:
 assert interval(before,start,end)==interval(after,start,end)
endpoint=(W/'hypercurve/src/bezier_split_endpoint.rs').read_text()
assert 'pub fn point(&self) -> CurveResult<Classification<&BezierEndpointPointImage2>>' in endpoint
assert 'try_point(' not in endpoint
assert 'point: OnceLock<BezierEndpointPointImage2>' in endpoint
assert 'certified private split endpoint point image must remain constructible' not in endpoint
for marker in ['fn lazy_endpoint_points_preserve_affine_domain_blockers()', 'Classification::Uncertain(UncertaintyReason::Boundary)', 'CurveContext::STRICT, CurveContext::APPROXIMATE_512']:
 assert marker in endpoint
pole=(W/'hypercurve/tests/hypercurve_bezier_algebraic_image.rs').read_text()
for marker in ['fn rational_point_images_require_finite_affine_coordinates()', 'conic.point_at_algebraic_parameter(&pole, &policy)', 'general.point_at_algebraic_parameter(&pole, &policy)', 'BezierAlgebraicEndpointImage2::rational_quadratic', 'BezierAlgebraicEndpointImage2::rational(&general', 'q(3, 4)']:
 assert marker in pole,marker
assert re.search(r"point\s*\.coincides_with\(",pole)
assert (W/'hypercurve/src/curve_point.rs').read_bytes()==(A/'source-archives/boundary-provenance-admission-v783/hypercurve/src/curve_point.rs').read_bytes()
# CurvePoint2 equality is unchanged: failure is stopped at construction.
for name in ['hypercurve/src/curve_fillet.rs','hypercurve/src/curve_region_corner.rs']:
 if name in manifest:assert manifest[name]==old_manifest[name]
previous=json.loads((A/'boundary-provenance-admission-v783-repositories-after.json').read_text());repositories={};repos={}
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
review=dict(qualification=f'{prefix}-terminal.json',source_manifest=r['source_manifest'],outer_session_reaped=reaped['outer_session'],validated_files=len(manifest),tests=len(expected_set),checks=7,repos=repos,scope='Publish only finite rational affine point images; propagate construction blockers through eager/lazy endpoints and splitting; preserve exact retained expressions and successful materialization reuse without ownership cycles',unresolved='The full exact-geometry implementation goal remains active; this cleanup does not claim full spatial BREP closure or a measured speedup.')
(A/f'{prefix}-reviewed.json').write_text(json.dumps(review,indent=2)+'\n')
print('Reviewed',len(expected_set),'release cases,eight pinned binaries,seven checks,2048sources and30repositories')
