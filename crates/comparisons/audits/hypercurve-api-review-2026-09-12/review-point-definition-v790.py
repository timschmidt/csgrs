from pathlib import Path
import hashlib,json,re,subprocess
A=Path(__file__).resolve().parent;W=A.parent;prefix='point-definition-v790'
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
expected=json.loads((A/'point-definition-cases-v790.json').read_text());expected_set=set()
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
promotion=json.loads((A/'point-definition-promotion-v790.json').read_text());changed=set(promotion['promoted']);assert len(changed)==1
old_manifest=json.loads((A/'finite-point-image-v788-sources.json').read_text());assert {n for n in manifest if manifest[n]!=old_manifest[n]}==changed
for name in changed:
 assert digest(Path(promotion['candidates'][name]))==manifest[name]==promotion['promoted'][name]
 assert promotion['base'][name]==old_manifest[name]
before=(A/'source-archives/finite-point-image-v788/hypercurve/src/bezier_algebraic_image.rs').read_text();after=(W/'hypercurve/src/bezier_algebraic_image.rs').read_text()
def interval(source,start,end):
 a=source.index(start);b=source.index(end,a);return source[a:b]
layout=interval(after,'struct RationalBezierAlgebraicPointImageData {','pub(crate) struct RationalBezierAlgebraicPointPredicate2')
assert 'definition: RationalPointDefinition' in layout
assert 'status:' not in layout and 'Option<' not in layout and 'String' not in layout
assert "message: &'static str" in layout
assert 'x: BezierAlgebraicRationalCoordinateImage' in layout and 'y: BezierAlgebraicRationalCoordinateImage' in layout
for marker in ['Coordinates {','Expression {','Parametric(RetainedRationalPointParametricSource)']:
 assert marker in layout
constructors=interval(after,'    fn from_coordinates(','    fn retained_expression(')
assert 'to_owned' not in constructors and 'None' not in constructors
assert 'fn new(' not in interval(after,'impl RationalBezierAlgebraicPointImage2 {','impl RationalBezierAlgebraicPointPredicate2')
status=interval(after,'    /// Reports whether coordinates are projected','    /// Returns the represented Bezier parameter used as the source root.')
assert 'self.data.status' not in status and 'RationalPointDefinition::Coordinates' in status
# The algebraic root/coordinate transforms, exact non-pole decision, polynomial
# and tangent images, and parameter reduction remain byte-identical.
for start,end in [
 ('pub(crate) fn rational_point_image_from_power_basis(', 'fn rational_coordinate_image_pair('),
 ('fn rational_coordinate_image_pair(', 'fn coordinate_image_from_replay('),
 ('impl RationalBezierAlgebraicPointPredicate2', 'fn rational_point_image_with_parameter_representation('),
]:
 assert interval(before,start,end)==interval(after,start,end),(start,end)
assert before[before.index('fn coordinate_image_from_replay('):]==after[after.index('fn coordinate_image_from_replay('):]
# Changing a point's storage must not change any operation, caller, or test
# outside the owning module. The one existing internal assertion now queries
# the explicit parametric form to check that a constant axis stays lazy.
assert len(changed)==1
assert after.count('fn retained_constant_coordinates_do_not_require_scalar_parameter_images()')==1
old_prefix=before[:before.index('/// Exact algebraic image of a rational quadratic Bezier affine point.')]
new_prefix=after[:after.index('/// Exact affine point of a rational Bezier at one selected algebraic parameter.')]
old_prefix=re.sub(r'image\s*\.data\s*\.parametric_source\s*\.as_ref\(\)', 'image.parametric_source()',old_prefix)
assert re.sub(r'\s+', '',old_prefix)==re.sub(r'\s+', '',new_prefix)
previous=json.loads((A/'finite-point-image-v788-repositories-after.json').read_text());repositories={};repos={}
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
review=dict(qualification=f'{prefix}-terminal.json',source_manifest=r['source_manifest'],outer_session_reaped=reaped['outer_session'],validated_files=len(manifest),tests=len(expected_set),checks=7,repos=repos,scope='Represent each rational point by exactly one coordinate,expression or parametric definition; derive status from that definition,remove optional payload combinations and static diagnostic String allocations; preserve algebraic transformations and replay kernels',unresolved='The full exact-geometry implementation goal remains active; this cleanup does not claim full spatial BREP closure or a measured speedup.')
(A/f'{prefix}-reviewed.json').write_text(json.dumps(review,indent=2)+'\n')
print('Reviewed',len(expected_set),'release cases,eight pinned binaries,seven checks,2048sources and30repositories')
