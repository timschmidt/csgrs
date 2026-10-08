from pathlib import Path
import hashlib,json,re,subprocess
A=Path(__file__).resolve().parent;W=A.parent;prefix='region-admission-v763'
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
expected=json.loads((A/'region-admission-cases-v763.json').read_text());expected_set={(t,n)for t,ns in expected.items()for n in ns}
assert len(r['cases'])==len(expected_set)==75
assert expected_set=={(c['target'],c['name'])for c in r['cases']}and len({c['log']for c in r['cases']})==75
for c in r['cases']:
 assert c['passed']and c['returncode']==0 and 'test result: ok. 1 passed;'in(A/c['log']).read_text()
 assert c['command'][0]==binaries[c['target']]['path']and '--include-ignored'in c['command']
for target,listing in r['test_listings'].items():
 assert listing['returncode']==0
 available={s.removesuffix(': test')for s in(A/listing['log']).read_text().splitlines()if s.endswith(': test')};assert set(expected[target])<=available
promotion=json.loads((A/'region-admission-promotion-v763.json').read_text());changed=set(promotion['promoted']);assert len(changed)==5
old_manifest=json.loads((A/'nesting-result-removal-v757-sources.json').read_text());assert {n for n in manifest if manifest[n]!=old_manifest[n]}==changed
for name in changed:
 assert digest(Path(promotion['candidates'][name]))==manifest[name]==promotion['promoted'][name]
 assert promotion['base'][name]==old_manifest[name]
prior=json.loads((A/'nesting-result-removal-v757-terminal.json').read_text());prior_archive=Path(prior['source_directory'])
source=(W/'hypercurve/src/bezier_region.rs').read_text();expected=(prior_archive/'hypercurve/src/bezier_region.rs').read_text()
replacements=[
 ('    if source_segments.is_empty() {\n        return Err(curve_region_promotion_error(CurveError::EmptyCurveString));\n    }\n',''),
 ('    raw.data_mut_for_construction().certified_loop_fill_rules =\n        Some(Arc::from(vec![fill_rule; paths.len()]));','    if !raw.is_empty() {\n        raw.data_mut_for_construction().certified_loop_fill_rules =\n            Some(Arc::from(vec![fill_rule; paths.len()]));\n    }'),
 ('    /// operations.\n    pub fn arrange_unordered_segments(','    /// operations. An empty collection produces the canonical empty region.\n    pub fn arrange_unordered_segments('),
 ('    /// Unsupported retained source fragments return explicit uncertainty rather\n    /// than sampled geometry.\n    pub fn offset(','    /// Unsupported retained source fragments return explicit uncertainty rather\n    /// than sampled geometry. After corner-option validation, an empty region\n    /// remains empty for every signed distance without requiring its sign.\n    pub fn offset('),
]
for old,new in replacements:assert expected.count(old)==1;expected=expected.replace(old,new)
start=expected.index('    fn offset_exact_general_raw(');position=expected.index('        if is_zero(&distance, policy) == Some(true) {',start)
expected=expected[:position]+expected[position:].replace('        if is_zero(&distance, policy) == Some(true) {','        if self.is_empty() || is_zero(&distance, policy) == Some(true) {',1)
assert expected==source
before_recovery=(prior_archive/'hypercurve/src/reconstruct.rs').read_text();recovery=(W/'hypercurve/src/reconstruct.rs').read_text()
assert before_recovery[before_recovery.index('const DEFAULT_DISTANCE_TOLERANCE'):before_recovery.index('impl CurveRegion2 {')]==recovery[recovery.index('const DEFAULT_DISTANCE_TOLERANCE'):recovery.index('impl CurveRegion2 {')]
assert before_recovery[before_recovery.index('fn recover_finite_ring('):]==recovery[recovery.index('fn recover_finite_ring('):]
assert 'Self::try_from_native_contours(material_contours, hole_contours, policy)'in recovery
assert ') -> ExactCurveResult<CurveOutcome<Self>> {'in recovery
added={'hypercurve/src/native_region_tests.rs':{'empty_unordered_arrangement_reenters_exact_set_operations','empty_region_offsets_preserve_set_and_policy_identity'},'hypercurve/tests/hypercurve_reconstruct.rs':{'finite_profile_recovery_regularizes_overlaps_before_publication','finite_profile_recovery_preserves_nested_islands_and_cancels_filled_holes','finite_profile_recovery_accepts_empty_input_with_certified_topology'}}
for name in changed:
 before=set(re.findall(r'#\[test\]\s*fn (\w+)',(prior_archive/name).read_text()));after=set(re.findall(r'#\[test\]\s*fn (\w+)',(W/name).read_text()))
 assert not before-after,name
 assert after-before==added.get(name,set()),name
for prefix2,key,marker in [('empty-arrangement-probe-v759','reproduced_empty_input_rejection','verified_rejections=4'),('region-admission-probe-v761','reproduced_empty_offset_and_raw_recovery_defects','empty_offset_failures=6')]:
 probe=json.loads((A/f'{prefix2}-terminal.json').read_text());probe_reaped=json.loads((A/f'{prefix2}-reaped.json').read_text())
 assert probe_reaped['outer_exit_code']==0 and probe['all_processes_reaped']and probe[key]
 assert probe['baseline']=='nesting-result-removal-v757-sources.json'
 assert digest(A/f'{prefix2}-binary')==probe['binary_sha256']and digest(A/f'{prefix2}.rs')==probe['source_sha256']
 assert all(p['returncode']==0 for p in probe['processes'])and marker in(A/probe['processes'][-1]['log']).read_text()
previous=json.loads((A/'nesting-result-removal-v757-repositories-after.json').read_text());repositories={};repos={}
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
review=dict(qualification=f'{prefix}-terminal.json',source_manifest=r['source_manifest'],outer_session_reaped=reaped['outer_session'],validated_files=len(manifest),tests=75,checks=7,repos=repos,scope='Publish normalized reconstructed regions, admit empty unordered arrangements and preserve empty-set offset closure without distance-sign decisions',unresolved='The full exact-geometry implementation goal remains active; this cleanup does not claim full spatial BREP closure or a measured speedup.')
(A/f'{prefix}-reviewed.json').write_text(json.dumps(review,indent=2)+'\n')
print('Reviewed75 release cases,three pinned binaries,seven checks,two baseline defect probes,2048sources and30repositories')
