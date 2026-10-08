from pathlib import Path
import hashlib,json,re,subprocess
A=Path(__file__).resolve().parent;W=A.parent;prefix='native-boundary-admission-v780'
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def git(repo,*args):return subprocess.check_output(['git',*args],cwd=W/repo)
reaped=json.loads((A/f'{prefix}-reaped.json').read_text());assert reaped['outer_exit_code']==0
r=json.loads((A/f'{prefix}-terminal.json').read_text());assert r['normal_production_build']and r['qualification_complete']and r['probe_complete']and r['all_processes_reaped']
manifest=json.loads((A/r['source_manifest']).read_text());assert len(manifest)==2048
for name,sha in manifest.items():
 for root in [W,Path(r['source_directory']),A/'build-workspace-20260925']:assert digest(root/name)==sha,(root,name)
assert len(r['builds'])==3 and len(r['test_listings'])==3 and len(r['checks'])==8 and all(c['returncode']==0 for c in r['checks'])
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
expected=json.loads((A/'native-boundary-admission-cases-v777.json').read_text());expected_set={(t,n)for t,ns in expected.items()for n in ns}
assert len(r['cases'])==len(expected_set)==46
assert expected_set=={(c['target'],c['name'])for c in r['cases']}and len({c['log']for c in r['cases']})==46
for c in r['cases']:
 assert c['passed']and c['returncode']==0 and 'test result: ok. 1 passed;'in(A/c['log']).read_text()
 assert c['command'][0]==binaries[c['target']]['path']and '--include-ignored'in c['command']
for target,listing in r['test_listings'].items():
 assert listing['returncode']==0
 available={s.removesuffix(': test')for s in(A/listing['log']).read_text().splitlines()if s.endswith(': test')};assert set(expected[target])<=available
promotion=json.loads((A/'native-boundary-admission-promotion-v780.json').read_text());changed=set(promotion['promoted']);assert len(changed)==7
old_manifest=json.loads((A/'topology-hint-removal-v773-sources.json').read_text());assert {n for n in manifest if manifest[n]!=old_manifest[n]}==changed
for name in changed:
 assert digest(Path(promotion['candidates'][name]))==manifest[name]==promotion['promoted'][name]
 assert promotion['base'][name]==old_manifest[name]
prior=json.loads((A/'topology-hint-removal-v773-terminal.json').read_text());prior_archive=Path(prior['source_directory'])
before=(prior_archive/'hypercurve/src/bezier_region.rs').read_text();after=(W/'hypercurve/src/bezier_region.rs').read_text()
old_start=before.index('    /// Nests unordered native boundary contours and promotes their decided roles.')
new_start=after.index('    /// Constructs the exact regularized fill of native boundary contours.')
end='    /// Classifies native contours through the shared raw-loop nesting authority.'
assert before[:old_start]==after[:new_start]
assert before[before.index(end,old_start):]==after[after.index(end,new_start):]
block=after[new_start:after.index(end,new_start)]
assert 'contours: &[Contour2],'in block and 'fill_rule: FillRule,'in block
assert 'ExactCurveResult<CurveOutcome<Self>>'in block
body=block[block.index('        let paths'):]
assert body=="        let paths = contours\n            .iter()\n            .map(curve_path_from_native_contour)\n            .collect::<ExactCurveResult<Vec<_>>>()?;\n        Self::try_from_boundary_paths(&paths, fill_rule, policy)\n    }\n\n"
assert 'native_boundary_contour_nesting_raw'not in block
# Only the native adapter changes in the production geometric authority;
# private nesting, orientation, arrangement, replay and offset logic is identical.
removed={'boundary_contour_nesting_assigns_disjoint_nested_roles','boundary_contour_nesting_rejects_crossing_or_touching_loops'}
added={'boundary_contour_fill_assigns_disjoint_nested_roles','boundary_contour_fill_regularizes_crossings_and_touches','native_boundary_global_fill_retains_winding_and_recursive_islands','native_boundary_circle_fills_reenter_exact_boolean_operations'}
for name in changed:
 old_tests=set(re.findall(r'#\[test\]\s*fn (\w+)',(prior_archive/name).read_text()));new_tests=set(re.findall(r'#\[test\]\s*fn (\w+)',(W/name).read_text()))
 if name.endswith('/native_region_tests.rs'):assert old_tests-new_tests==removed and new_tests-old_tests==added
 else:
  assert not old_tests-new_tests,name
  assert new_tests-old_tests==({'gerber_boundary_contours_preserve_nested_parity_and_roundtrip'}if name=='csgrs/tests/geometry_context.rs'else set()),name
native=(W/'hypercurve/src/native_region_tests.rs').read_text()
for marker in ['CurveCertainty::Certified','BooleanOp::Xor','native_contours_fast_path','OffsetCornerStyle2::Round','(3, 2)','outer.segments().iter().chain(outer.segments())','for contour_fill in [FillRule::EvenOdd, FillRule::NonZero]']:
 assert re.sub(r'\s+','',marker) in re.sub(r'\s+','',native),marker
probe_prefix='native-boundary-probe-v776'
probe=json.loads((A/f'{probe_prefix}-terminal.json').read_text());probe_reaped=json.loads((A/f'{probe_prefix}-reaped.json').read_text())
assert probe_reaped['outer_exit_code']==0 and probe['all_processes_reaped']and probe['probe_complete']
assert probe['baseline']=='topology-hint-removal-v773-sources.json'
assert digest(A/f'{probe_prefix}-binary')==probe['binary_sha256']and digest(A/f'{probe_prefix}.rs')==probe['source_sha256']
assert all(p['returncode']==0 for p in probe['processes'])and 'controls=4 blocked_roundtrips=6'in(A/probe['processes'][-1]['log']).read_text()
previous_run=json.loads((A/'native-boundary-admission-v777-terminal.json').read_text());receipt=json.loads((A/'native-boundary-admission-v777-reaped.json').read_text())
assert receipt['outer_exit_code']==1 and previous_run['all_processes_reaped']
previous_sources=json.loads((A/previous_run['source_manifest']).read_text());changed_test='hypercurve/tests/hypercurve_curve_region_promotion.rs'
assert {n for n in manifest if manifest[n]!=previous_sources[n]}=={changed_test}
for n,sha in previous_sources.items():assert digest(Path(previous_run['source_directory'])/n)==sha,n
proof=promotion['test_only_correction'];old_source=(Path(previous_run['source_directory'])/changed_test).read_text();assert old_source.count(proof['before'])==1
formatted=subprocess.check_output(['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustfmt','--edition','2024','--config','skip_children=true','--emit','stdout'],input=old_source.replace(proof['before'],proof['after']).encode(),cwd=W/'hypercurve')
assert formatted==(W/changed_test).read_bytes()
assert r['cases'][:35]==[c for c in previous_run['cases']if c['passed']]and r['source_change_proof']['reused_passing_cases']==35
assert r['reused_builds_from']=='native-boundary-admission-v777-terminal.json'
previous=json.loads((A/'topology-hint-removal-v773-repositories-after.json').read_text());repositories={};repos={}
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
review=dict(qualification=f'{prefix}-terminal.json',source_manifest=r['source_manifest'],outer_session_reaped=reaped['outer_session'],validated_files=len(manifest),tests=46,checks=8,repos=repos,scope='Admit native compound boundaries through the shared arrangement with explicit global fill, remove the separate nesting prepass and classification wrapper, and migrate callers directly',unresolved='The full exact-geometry implementation goal remains active; this cleanup does not claim full spatial BREP closure or a measured speedup.')
(A/f'{prefix}-reviewed.json').write_text(json.dumps(review,indent=2)+'\n')
print('Reviewed46 release cases,three pinned binaries,eight checks,baseline admission probe,2048sources and30repositories')
