from pathlib import Path
import hashlib,json,re,subprocess
A=Path(__file__).resolve().parent;W=A.parent;prefix='boundary-provenance-admission-v783'
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def git(repo,*args):return subprocess.check_output(['git',*args],cwd=W/repo)
reaped=json.loads((A/f'{prefix}-reaped.json').read_text());assert reaped['outer_exit_code']==0
r=json.loads((A/f'{prefix}-terminal.json').read_text());assert r['normal_production_build']and r['qualification_complete']and r['probe_complete']and r['all_processes_reaped']
manifest=json.loads((A/r['source_manifest']).read_text());assert len(manifest)==2048
for name,sha in manifest.items():
 for root in [W,Path(r['source_directory']),A/'build-workspace-20260925']:assert digest(root/name)==sha,(root,name)
assert len(r['builds'])==1 and len(r['test_listings'])==3 and len(r['checks'])==7 and all(c['returncode']==0 for c in r['checks'])
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
expected=json.loads((A/'boundary-provenance-admission-cases-v783.json').read_text());expected_set=set()
for target,listing in r['test_listings'].items():
 assert listing['returncode']==0
 available={s.removesuffix(': test')for s in(A/listing['log']).read_text().splitlines()if s.endswith(': test')}
 selected=set(expected[target])if expected[target]is not None else available
 assert selected<=available and selected==set(r['expected_cases'][target]);expected_set.update((target,name)for name in selected)
assert len(r['cases'])==len(expected_set)==48
assert expected_set=={(c['target'],c['name'])for c in r['cases']}and len({c['log']for c in r['cases']})==48
for c in r['cases']:
 assert c['passed']and c['returncode']==0 and 'test result: ok. 1 passed;'in(A/c['log']).read_text()
 assert c['command'][0]==binaries[c['target']]['path']and '--include-ignored'in c['command']
promotion=json.loads((A/'boundary-provenance-admission-promotion-v783.json').read_text());changed=set(promotion['promoted']);assert len(changed)==3
old_manifest=json.loads((A/'unordered-result-removal-v781-sources.json').read_text());assert {n for n in manifest if manifest[n]!=old_manifest[n]}==changed
for name in changed:
 assert digest(Path(promotion['candidates'][name]))==manifest[name]==promotion['promoted'][name]
 assert promotion['base'][name]==old_manifest[name]
prior=json.loads((A/'unordered-result-removal-v781-terminal.json').read_text());prior_archive=Path(prior['source_directory'])
before=(prior_archive/'hypercurve/src/bezier_region.rs').read_text();after=(W/'hypercurve/src/bezier_region.rs').read_text()
def interval(source,start,end):
 a=source.index(start);b=source.index(end,a);return source[a:b]
def erase(source,start,end):
 a=source.index(start);b=source.index(end,a);return source[:a]+source[b:]
old_production=before.split('\n#[cfg(test)]\nmod tests {')[0];new_production=after.split('\n#[cfg(test)]\nmod tests {')[0]
expected_production=erase(old_production,'    /// Constructs a retained boundary loop with one source record per fragment.','    fn try_new_from_certified_arrangement_chain(')
expected_production=expected_production.replace('/// Arrangement provenance for one retained boundary fragment.','/// Provenance recorded while constructing a retained boundary fragment.\n///\n/// Curve operations supply these records; completed boundaries expose them\n/// for inspection without accepting caller-authored arrangement indices.',1)
a=expected_production.index('impl CurveRegionFragmentSource2 {');b=expected_production.index('    /// Returns the retained arrangement-graph fragment index.',a)
block=expected_production[a:b];assert block.count('pub const fn new(')==1
expected_production=expected_production[:a]+block.replace('pub const fn new(','pub(crate) const fn new(')+expected_production[b:]
assert expected_production==new_production
# Every production validator, private constructor, certificate, transform,
# Boolean, offset and root-replay path is byte-identical outside the deleted
# unused public constructor and visibility/documentation change.
for start,end in [('    fn try_new_from_certified_arrangement_chain(','    /// Returns each exact boundary curve in traversal order.'),('fn validate_retained_boundary_loop_sources(','fn validate_retained_boundary_loop_connectivity('),('fn validate_unique_arrangement_source_indices(','fn retained_line_loop_to_contour(')]:
 assert interval(before,start,end)==interval(after,start,end)
assert 'pub(crate) const fn new('in interval(after,'impl CurveRegionFragmentSource2 {','/// An exact regularized planar filled set')
for name in manifest:
 if name.endswith(('.rs','.md')):assert 'try_new_with_arrangement_sources'not in(W/name).read_text(),name
assert 'CurveRegionFragmentSource2'in(W/'hypercurve/src/lib.rs').read_text()
# The two retained fixture migrations retain ordinary geometry validation
# before using the private operation constructor for their known provenance.
for marker in ['let boundary = CurveRegionBoundaryLoop2::new(fragments, &CurveContext::STRICT).unwrap();','let boundary = CurveRegionBoundaryLoop2::new(fragments, policy).unwrap();']:
 assert marker in after
old_tests=set(re.findall(r'#\[test\]\s*fn (\w+)',before));new_tests=set(re.findall(r'#\[test\]\s*fn (\w+)',after))
assert not old_tests-new_tests and new_tests-old_tests=={'certified_boundary_constructors_validate_arrangement_sources'}
test=interval(after,'    fn certified_boundary_constructors_validate_arrangement_sources()','    #[test]\n    fn retained_region_constructor_rejects_reused_arrangement_sources_across_loops()')
for marker in ['CurveContext::STRICT, CurveContext::APPROXIMATE_512','try_new_from_certified_arrangement_chain','try_new_from_certified_connected_chain','Err(CurveError::Topology(_))','Some(sources.as_slice())']:
 assert marker in test,marker
name='hypercurve/tests/hypercurve_bezier_region.rs';old=(prior_archive/name).read_text();new=(W/name).read_text()
old_names=set(re.findall(r'#\[test\]\s*fn (\w+)',old));new_names=set(re.findall(r'#\[test\]\s*fn (\w+)',new))
assert old_names-new_names=={'retained_boundary_loop_constructor_rejects_duplicate_arrangement_sources'}and not new_names-old_names
# The public malformed-source assertion is superseded by the private gate
# matrix. Ordinary empty/malformed/forged-endpoint and provenance reads remain.
for marker in ['empty_boundary_loops_do_not_certify_signed_area','retained_boundary_loop_constructor_rejects_forged_source_endpoint_image','regularized_nonlinear_boundary_retains_roles_area_and_provenance','arrangement_sources()']:
 assert marker in new
assert 'retained_algebraic_line_fragment'not in new
previous=json.loads((A/'unordered-result-removal-v781-repositories-after.json').read_text());repositories={};repos={}
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
review=dict(qualification=f'{prefix}-terminal.json',source_manifest=r['source_manifest'],outer_session_reaped=reaped['outer_session'],validated_files=len(manifest),tests=48,checks=7,repos=repos,scope='Remove unused public provenance authoring,retain read-only operation records and all private geometry/source validation,verify invalid/valid source gates directly',unresolved='The full exact-geometry implementation goal remains active; this cleanup does not claim full spatial BREP closure or a measured speedup.')
(A/f'{prefix}-reviewed.json').write_text(json.dumps(review,indent=2)+'\n')
print('Reviewed48 release cases,three pinned binaries,seven checks,unchanged production geometry/certificate validation,2048sources and30repositories')
