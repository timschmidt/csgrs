from pathlib import Path
import hashlib,json,re,subprocess
A=Path(__file__).resolve().parent;W=A.parent;prefix='polynomial-derivative-v817'
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
expected=json.loads((A/'polynomial-derivative-cases-v817.json').read_text());expected_set=set()
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
promotion=json.loads((A/'polynomial-derivative-promotion-v817.json').read_text());changed=set(promotion['promoted']);assert len(changed)==11
old_manifest=json.loads((A/'polynomial-point-v806-sources.json').read_text());assert {n for n in manifest if manifest[n]!=old_manifest[n]}==changed
for name in changed:
 assert digest(Path(promotion['candidates'][name]))==manifest[name]==promotion['promoted'][name]
 assert promotion['base'][name]==old_manifest[name]
old_root=A/'source-archives/polynomial-point-v806'
image_name='hypercurve/src/bezier_algebraic_image.rs'
before=(old_root/image_name).read_text();after=(W/image_name).read_text()
def between(source,start,end):
 a=source.index(start);b=source.index(end,a);return source[a:b].strip()
assert between(before,'\nfn point_image(', '\nfn tangent_image(')==between(after,'\nfn point_image(', '\nfn tangent_image(')
assert between(before,'\nfn rational_tangent_image(', '\nfn coordinate_image(')==between(after,'\nfn rational_tangent_image(', '\nenum RationalCoordinateImagePair {')
assert between(before,'\nenum RationalCoordinateImagePair {', '\nfn coordinate_image_from_replay(')==between(after,'\nenum RationalCoordinateImagePair {', '\npub(crate) fn parameter_representation(')
start='\npub(crate) fn parameter_representation('
assert before[before.index(start):]==after[after.index(start):]
native=between(after,'\nfn tangent_image(', '\nfn rational_tangent_image(')
assert 'rational_tangent_image_from_power_basis(' in native and 'vec![Real::one()]' in native
for name in manifest:
 if name.endswith(('.rs','.md')):
  source=(W/name).read_text()
  assert not re.search(r'\b(?:BezierAlgebraicCoordinateImage|BezierAlgebraicTangentImage2|BezierEndpointTangentImage2|from_endpoint_image)\b',source),name
endpoint=(W/'hypercurve/src/bezier_split_endpoint.rs').read_text()
assert 'second_derivative: Option<RationalBezierAlgebraicTangentImage2>' in endpoint
assert 'third_derivative: Option<RationalBezierAlgebraicTangentImage2>' in endpoint
assert 'Box<RationalBezierAlgebraicTangentImage2>' not in endpoint
assert 'curve: Box<BezierSubcurve2>' in endpoint
assert len(expected)==12 and len(expected_set)==321
assert ('hypercurve_bezier_algebraic_image','polynomial_endpoint_derivatives_replay_exact_values_in_the_shared_carrier') in expected_set
assert ('hypercurve','bezier_arrangement::endpoint_adjacency_tests::lazy_polynomial_endpoint_derivatives_match_eager_images') in expected_set
probe=json.loads((A/'polynomial-derivative-probe-v818-terminal.json').read_text())
probe_reap=json.loads((A/'polynomial-derivative-probe-v818-reaped.json').read_text())
assert probe_reap['outer_exit_code']==0 and probe['probe_complete']and probe['all_processes_reaped']
assert probe['baseline']==r['source_manifest']and all(p['returncode']==0 for p in probe['processes'])
assert digest(Path(probe['rlib']))==probe['rlib_sha256']
assert probe['source_sha256']==digest(A/'polynomial-derivative-probe-v809.rs')==json.loads((A/'polynomial-derivative-probe-v809-terminal.json').read_text())['source_sha256']
assert (A/'polynomial-derivative-probe-v818-run.log').read_text()==(A/'polynomial-derivative-probe-v809-run.log').read_text().replace('native=XImageFailed','native=Transformed').replace('endpoint_exact=false','endpoint_exact=true')
previous=json.loads((A/'polynomial-point-v806-repositories-after.json').read_text());repositories={};repos={}
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
review=dict(qualification=f'{prefix}-terminal.json',source_manifest=r['source_manifest'],outer_session_reaped=reaped['outer_session'],validated_files=len(manifest),tests=len(expected_set),checks=7,repos=repos,scope='Remove polynomial coordinate/tangent and endpoint tangent wrapper types; share the existing rational-function derivative carrier and selected-source reduction, preserve native derivative-order demand, and store optional shared derivatives without extra boxes. Migrate all consumers directly.',unresolved='The full exact-geometry implementation goal remains active; this migration does not establish universal composition closure or a measured speedup.')
(A/f'{prefix}-reviewed.json').write_text(json.dumps(review,indent=2)+'\n')
print('Reviewed',len(expected_set),'release cases,twelve pinned binaries,seven checks,2048sources and30repositories')
