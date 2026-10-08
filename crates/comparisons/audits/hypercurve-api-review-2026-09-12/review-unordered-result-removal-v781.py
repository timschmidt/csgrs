from pathlib import Path
import hashlib,json,re,subprocess
A=Path(__file__).resolve().parent;W=A.parent;prefix='unordered-result-removal-v781'
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def git(repo,*args):return subprocess.check_output(['git',*args],cwd=W/repo)
reaped=json.loads((A/f'{prefix}-reaped.json').read_text());assert reaped['outer_exit_code']==0
r=json.loads((A/f'{prefix}-terminal.json').read_text());assert r['normal_production_build']and r['qualification_complete']and r['probe_complete']and r['all_processes_reaped']
manifest=json.loads((A/r['source_manifest']).read_text());assert len(manifest)==2048
for name,sha in manifest.items():
 for root in [W,Path(r['source_directory']),A/'build-workspace-20260925']:assert digest(root/name)==sha,(root,name)
assert len(r['builds'])==1 and len(r['test_listings'])==2 and len(r['checks'])==7 and all(c['returncode']==0 for c in r['checks'])
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
expected=json.loads((A/'unordered-result-removal-cases-v781.json').read_text());expected_set={(t,n)for t,ns in expected.items()for n in ns}
assert len(r['cases'])==len(expected_set)==40
assert expected_set=={(c['target'],c['name'])for c in r['cases']}and len({c['log']for c in r['cases']})==40
for c in r['cases']:
 assert c['passed']and c['returncode']==0 and 'test result: ok. 1 passed;'in(A/c['log']).read_text()
 assert c['command'][0]==binaries[c['target']]['path']and '--include-ignored'in c['command']
for target,listing in r['test_listings'].items():
 assert listing['returncode']==0
 available={s.removesuffix(': test')for s in(A/listing['log']).read_text().splitlines()if s.endswith(': test')};assert set(expected[target])<=available
promotion=json.loads((A/'unordered-result-removal-promotion-v781.json').read_text());changed=set(promotion['promoted']);assert len(changed)==8
old_manifest=json.loads((A/'native-boundary-admission-v780-sources.json').read_text());assert {n for n in manifest if manifest[n]!=old_manifest[n]}==changed
for name in changed:
 assert digest(Path(promotion['candidates'][name]))==manifest[name]==promotion['promoted'][name]
 assert promotion['base'][name]==old_manifest[name]
prior=json.loads((A/'native-boundary-admission-v780-terminal.json').read_text());prior_archive=Path(prior['source_directory'])
before=(prior_archive/'hypercurve/src/bezier_region.rs').read_text();after=(W/'hypercurve/src/bezier_region.rs').read_text()
def interval(source,start,end):
 a=source.index(start);b=source.index(end,a);return source[a:b]
def erase(source,start,end):
 a=source.index(start);b=source.index(end,a);return source[:a]+source[b:]
def compact(source):return re.sub(r'\s+','',source)
old_kernel=interval(before,'fn arrange_unordered_native_segments_raw(','fn curve_region_edit_error(')
new_kernel=interval(after,'fn arrange_unordered_native_segments_raw(','fn curve_region_edit_error(')
assert compact(interval(old_kernel,'    let paths = rings','    let mut raw ='))==compact(interval(new_kernel,'    let paths = rings','    let mut raw ='))
assert interval(old_kernel,'    if !raw.is_empty() {','    let region =')==interval(new_kernel,'    if !raw.is_empty() {','    raw.finish_construction(policy)')
assert 'assemble_unordered_segment_rings(source_segments, policy).map_err(|reason|'in new_kernel
assert 'ExactCurveError::blocked(CurveOperation2::Construction, family, reason)'in new_kernel
assert 'let mut raw = CurveRegion2::try_from_boundary_paths_raw(&paths, policy)?;'in new_kernel
assert 'raw.finish_construction(policy)'in new_kernel
assert 'native_line_arc_region'not in new_kernel
assert interval(before,'    fn finish_construction(','    pub(crate) fn try_from_boundary_paths_raw(')==interval(after,'    fn finish_construction(','    pub(crate) fn try_from_boundary_paths_raw(')
# Outside the removed report types/kernel and its public result signature,
# the entire geometric source is unchanged (allowing formatter whitespace).
old_rest=erase(before,'/// Furthest exact stage reached by unified unordered-boundary arrangement.','/// Certified source-segmentation evidence')
old_rest=erase(old_rest,'impl CurveRegionArrangement2 {','impl CurveRegionSegmentationLoopEvidence2 {')
for token in ['RetainedTopologyStatus,','SegmentKindCounts,']:old_rest=old_rest.replace(token,'')
for source_name in ['old_rest','after']:
 text=locals()[source_name]
 text=erase(text,'fn arrange_unordered_native_segments_raw(','fn curve_region_edit_error(')
 text=erase(text,'    /// Arranges unordered exact line/arc segments through unified region topology.','    /// Constructs a unified region directly from explicit native contour roles.')
 if source_name=='old_rest':expected_rest=text
 else:actual_rest=text
assert compact(expected_rest)==compact(actual_rest)
wrapper=interval(after,'    pub fn arrange_unordered_segments(','    /// Constructs a unified region directly from explicit native contour roles.')
assert 'ExactCurveResult<CurveOutcome<Self>>'in wrapper and 'resolve_certified_operation(policy, |attempt|'in wrapper
assert 'arrange_unordered_native_segments_raw(source_segments, fill_rule, attempt)'in wrapper
for name in manifest:
 if not name.endswith(('.rs','.md')):continue
 text=(W/name).read_text()
 for token in ['CurveRegionArrangement2','CurveRegionArrangementStage2','blocked_unordered_curve_region_arrangement','region_segment_kind_counts']:
  assert token not in text,(name,token)
assert 'pub use retained_status::RetainedTopologyStatus;'in(W/'hypercurve/src/lib.rs').read_text()
for name in changed:
 old_tests=set(re.findall(r'#\[test\]\s*fn (\w+)',(prior_archive/name).read_text()));new_tests=set(re.findall(r'#\[test\]\s*fn (\w+)',(W/name).read_text()))
 if name.endswith('/hypercurve_curve_region_promotion.rs'):
  assert old_tests-new_tests=={'unified_native_arrangement_exposes_immediate_evidence'}
  assert new_tests-old_tests=={'unified_native_arrangement_returns_a_certified_region'}
 else:
  assert not old_tests-new_tests,name
  assert new_tests-old_tests==({'unordered_native_regions_reenter_operations_without_summary_queries'}if name.endswith('/native_region_tests.rs')else set()),name
native=(W/'hypercurve/src/native_region_tests.rs').read_text()
for marker in ['assert_boundary_blocked','CurveOperation2::Construction','CurveCertainty::Approximate512Consumed','Err(ExactCurveError::Blocked(_))','the next operation is deliberately the first consumer']:
 assert marker.lower()in native.lower(),marker
# Existing success/error fixtures and property-test input generators remain
# qualified through direct geometry, blocker reasons and common certainty.
for name in ['unordered_native_arrangement_obeys_the_approximate_512_terminal','generated_unordered_line_rectangles_build_unified_regions','generated_unordered_line_arc_semicircles_build_unified_regions','empty_unordered_arrangement_reenters_exact_set_operations','unordered_native_regions_reenter_operations_without_summary_queries']:
 assert 'native_region_tests::'+name in expected['hypercurve'],name
perf=(W/'hypercurve/PERFORMANCE.md').read_text();old_perf=(prior_archive/'hypercurve/PERFORMANCE.md').read_text()
old_tables=[line for line in old_perf.splitlines()if line.startswith('|')];new_tables=[line for line in perf.splitlines()if line.startswith('|')];assert old_tables==new_tables
previous=json.loads((A/'native-boundary-admission-v780-repositories-after.json').read_text());repositories={};repos={}
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
review=dict(qualification=f'{prefix}-terminal.json',source_manifest=r['source_manifest'],outer_session_reaped=reaped['outer_session'],validated_files=len(manifest),tests=40,checks=7,repos=repos,scope='Return unordered arrangement regions and common exact blockers directly;remove duplicate report/status/count machinery and eager native summaries without changing assembly or fill semantics',unresolved='The full exact-geometry implementation goal remains active; this cleanup does not claim full spatial BREP closure or a measured speedup.')
(A/f'{prefix}-reviewed.json').write_text(json.dumps(review,indent=2)+'\n')
print('Reviewed40 release cases,two pinned binaries,seven checks,geometric-kernel preservation,2048sources and30repositories')
