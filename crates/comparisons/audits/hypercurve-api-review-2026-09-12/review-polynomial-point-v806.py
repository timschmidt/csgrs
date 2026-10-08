from pathlib import Path
import hashlib,json,re,subprocess
A=Path(__file__).resolve().parent;W=A.parent;prefix='polynomial-point-v806'
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
expected=json.loads((A/'polynomial-point-cases-v806.json').read_text());expected_set=set()
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
promotion=json.loads((A/'polynomial-point-promotion-v806.json').read_text());changed=set(promotion['promoted']);assert len(changed)==14
old_manifest=json.loads((A/'homogeneous-projection-v800-sources.json').read_text());assert {n for n in manifest if manifest[n]!=old_manifest[n]}==changed
for name in changed:
 assert digest(Path(promotion['candidates'][name]))==manifest[name]==promotion['promoted'][name]
 assert promotion['base'][name]==old_manifest[name]
# The shared rational point and derivative kernels remain exactly as qualified.
old_root=A/'source-archives/homogeneous-projection-v800'
image_name='hypercurve/src/bezier_algebraic_image.rs'
before=(old_root/image_name).read_text();after=(W/image_name).read_text()
start='\nfn rational_point_image('
assert before[before.index(start):]==after[after.index(start):]
point=after[after.index('\nfn point_image('):after.index(start)]
assert 'denominator: vec![Real::one()]' in point and 'rational_point_image(' in point
assert 'point_at_algebraic_parameter' in after
for name in manifest:
 if name.endswith(('.rs','.md')):
  source=(W/name).read_text()
  assert not re.search(r'\b(?:BezierAlgebraicPointImage2|BezierEndpointPointImage2)\b',source),name
endpoint=(W/'hypercurve/src/bezier_split_endpoint.rs').read_text()
assert 'point: OnceLock<RationalBezierAlgebraicPointImage2>' in endpoint
assert 'point: RationalBezierAlgebraicPointImage2' in endpoint
assert '.is_ok_and(|point| matches!(point, Classification::Decided(_)))' in endpoint
measure_name='hypercurve/src/bezier_retained_measure.rs'
old_measure=(old_root/measure_name).read_text();measure=(W/measure_name).read_text()
start='fn polynomial_image_constant_remainder('
assert old_measure[old_measure.index(start):]==measure[measure.index(start):]
assert 'coordinate.denominator_coefficients() == [Real::one()]' in measure
assert len(expected)==12 and 'hypercurve_bezier_region' in expected and len(expected_set)>285
for name in ['polynomial_point_images_share_exact_replay_across_coefficient_domains','polynomial_point_images_retain_nonrational_source_roots']:
 assert ('hypercurve_bezier_algebraic_image',name) in expected_set
assert ('hypercurve','bezier_arrangement::endpoint_adjacency_tests::lazy_polynomial_endpoint_derivatives_match_eager_images') in expected_set
probe=json.loads((A/'polynomial-point-probe-v808-terminal.json').read_text())
probe_reap=json.loads((A/'polynomial-point-probe-v808-reaped.json').read_text())
assert probe_reap['outer_exit_code']==0 and probe['probe_complete']and probe['all_processes_reaped']
assert probe['baseline']==r['source_manifest']and all(p['returncode']==0 for p in probe['processes'])
assert digest(Path(probe['rlib']))==probe['rlib_sha256']
source=(A/'polynomial-point-probe-v804.rs').read_text()
for expression in ['quad.point_at_algebraic_parameter(&root,policy).unwrap()', 'BezierAlgebraicEndpointImage2::quadratic(&quad,&root,policy).unwrap()', 'cubic.point_at_algebraic_parameter(&root,policy).unwrap()', 'BezierAlgebraicEndpointImage2::cubic(&cubic,&root,policy).unwrap()']:
 assert source.count(expression)==1;source=source.replace(expression,'decided('+expression+')')
assert source==(A/'polynomial-point-probe-v807.rs').read_text()
assert probe['source_sha256']==digest(A/'polynomial-point-probe-v807.rs')
assert (A/'polynomial-point-probe-v808-run.log').read_text()==(A/'polynomial-point-probe-v804-run.log').read_text().replace('XImageFailed','RetainedRationalExpression')
previous=json.loads((A/'homogeneous-projection-v800-repositories-after.json').read_text());repositories={};repos={}
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
review=dict(qualification=f'{prefix}-terminal.json',source_manifest=r['source_manifest'],outer_session_reaped=reaped['outer_session'],validated_files=len(manifest),tests=len(expected_set),checks=7,repos=repos,scope='Remove polynomial-only point and endpoint wrapper types; share the existing exact rational-function point representation for polynomial and rational Beziers, preserving native coordinate formulas and replayable exact expressions. Migrate all consumers directly.',unresolved='The full exact-geometry implementation goal remains active; this cleanup does not claim full spatial BREP closure or a measured speedup.')
(A/f'{prefix}-reviewed.json').write_text(json.dumps(review,indent=2)+'\n')
print('Reviewed',len(expected_set),'release cases,twelve pinned binaries,seven checks,2048sources and30repositories')
