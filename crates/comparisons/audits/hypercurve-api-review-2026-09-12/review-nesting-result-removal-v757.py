from pathlib import Path
import hashlib,json,re,subprocess
A=Path(__file__).resolve().parent;W=A.parent;prefix='nesting-result-removal-v757'
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def git(repo,*args):return subprocess.check_output(['git',*args],cwd=W/repo)
reaped=json.loads((A/f'{prefix}-reaped.json').read_text());assert reaped['outer_exit_code']==0
r=json.loads((A/f'{prefix}-terminal.json').read_text());assert r['normal_production_build']and r['qualification_complete']and r['probe_complete']and r['all_processes_reaped']
manifest=json.loads((A/r['source_manifest']).read_text());assert len(manifest)==2048
for name,sha in manifest.items():
 for root in [W,Path(r['source_directory']),A/'build-workspace-20260925']:assert digest(root/name)==sha,(root,name)
assert len(r['builds'])==1 and len(r['test_listings'])==2 and len(r['checks'])==8 and all(c['returncode']==0 for c in r['checks'])
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
expected=json.loads((A/'nesting-result-removal-cases-v755.json').read_text());expected_set={(t,n)for t,ns in expected.items()for n in ns}
assert len(r['cases'])==len(expected_set)==73
assert expected_set=={(c['target'],c['name'])for c in r['cases']}and len({c['log']for c in r['cases']})==73
for c in r['cases']:
 assert c['passed']and c['returncode']==0 and 'test result: ok. 1 passed;'in(A/c['log']).read_text()
 assert c['command'][0]==binaries[c['target']]['path']and '--include-ignored'in c['command']
for target,listing in r['test_listings'].items():
 assert listing['returncode']==0
 available={s.removesuffix(': test')for s in(A/listing['log']).read_text().splitlines()if s.endswith(': test')};assert set(expected[target])<=available
prior_qualification=json.loads((A/'nesting-result-removal-v755-terminal.json').read_text());old_sources=json.loads((A/prior_qualification['source_manifest']).read_text());fix=json.loads((A/'nesting-helper-comment-fix-v756.json').read_text());name=fix['path']
assert json.loads((A/'nesting-result-removal-v755-reaped.json').read_text())['outer_exit_code']==0
assert prior_qualification['qualification_complete']and prior_qualification['all_processes_reaped']
assert r['reused_builds_from']=='nesting-result-removal-v755'and r['source_change_proof']['only_comments_changed']
assert {n for n in manifest if manifest[n]!=old_sources[n]}=={name}
for n,sha in old_sources.items():assert digest(Path(prior_qualification['source_directory'])/n)==sha,n
for block in [fix['old'],fix['new']]:assert all(line.strip().startswith('///')for line in block.splitlines())
old_text=(Path(prior_qualification['source_directory'])/name).read_text();assert old_text.count(fix['old'])==1
assert old_text.replace(fix['old'],fix['new'])==(W/name).read_text()
for key in ['builds','cases','test_listings','expected_cases']:assert r[key]==prior_qualification[key],key
assert r['checks'][:-1]==prior_qualification['checks']and r['checks'][-1]['label']=='comment-format'
promotion=json.loads((A/'nesting-result-removal-promotion-v757.json').read_text());changed=set(promotion['promoted']);assert len(changed)==5
old_manifest=json.loads((A/'unordered-arrangement-api-v752-sources.json').read_text());assert {n for n in manifest if manifest[n]!=old_manifest[n]}==changed
for name in changed:
 assert digest(Path(promotion['candidates'][name]))==manifest[name]==promotion['promoted'][name]
 assert promotion['base'][name]==old_manifest[name]
source=(W/'hypercurve/src/bezier_region.rs').read_text()
assert 'CurveRegionNestingRoleEvidence2'not in source and 'curved_nesting_role_evidence'not in source
assert 'struct NativeLoopNesting2 {'in source
prior=json.loads((A/'unordered-arrangement-api-v752-terminal.json').read_text());prior_archive=Path(prior['source_directory']);before=(prior_archive/'hypercurve/src/bezier_region.rs').read_text()
start=before.index('        let Some(native_loops)',before.index('    pub(crate) fn curved_nesting_role_evidence_raw('));end=before.index('        let evidence = CurveRegionNestingRoleEvidence2::new(',start)
expected=before[start:end].replace('        let mut nesting_depths = Vec::with_capacity(native_loops.len());\n','').replace('            nesting_depths.push(depth);\n','')
start=source.index('        let Some(native_loops)',source.index('    fn native_loop_nesting_raw('));end=source.index('        // Check source identity',start);assert source[start:end]==expected
assert 'validate_retained_region_arrangement_sources(&self.data.boundary_loops)?;'in source[end:end+500]
for start,end in [('fn validate_retained_boundary_loop_sources(', 'fn validate_retained_boundary_loop_connectivity('),('fn validate_unique_arrangement_source_indices(', 'fn validate_evidence_length(')]:
 bstart=before.index(start);bend=before.index(end,bstart)
 aend=source.index('fn retained_line_loop_to_contour('if end=='fn validate_evidence_length('else end,source.index(start))
 assert before[bstart:bend]==source[source.index(start):aend]
rename={'retained_curved_nesting_role_evidence_assigns_same_orientation_nonlinear_hole':'regularized_nonlinear_boundary_retains_roles_area_and_provenance'}
for name in changed:
 before_tests=set(re.findall(r'#\[test\]\s*fn (\w+)',(prior_archive/name).read_text()));after_tests=set(re.findall(r'#\[test\]\s*fn (\w+)',(W/name).read_text()))
 before_tests={rename.get(n,n)for n in before_tests}
 assert before_tests-after_tests==({'retained_nesting_evidence_constructor_rejects_mismatched_evidence'}if name=='hypercurve/tests/hypercurve_bezier_region.rs'else set()),name
 assert after_tests-before_tests==({'empty_native_boundary_input_constructs_an_exact_empty_region'}if name=='hypercurve/src/native_region_tests.rs'else set()),name
probe=json.loads((A/'empty-native-boundary-probe-v754-terminal.json').read_text());probe_reaped=json.loads((A/'empty-native-boundary-probe-v754-reaped.json').read_text())
assert probe_reaped['outer_exit_code']==0 and probe['all_processes_reaped']and probe['reproduced_empty_input_rejection']
assert probe['baseline']=='unordered-arrangement-api-v752-sources.json'
assert digest(A/'empty-native-boundary-probe-v754.rs')==probe['source_sha256']and digest(A/'empty-native-boundary-probe-v754-binary')==probe['binary_sha256']
assert all(p['returncode']==0 for p in probe['processes'])
assert 'verified_rejections=2'in(A/probe['processes'][-1]['log']).read_text()
previous=json.loads((A/'unordered-arrangement-api-v752-repositories-after.json').read_text());repositories={};repos={}
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
review=dict(qualification=f'{prefix}-terminal.json',source_manifest=r['source_manifest'],outer_session_reaped=reaped['outer_session'],validated_files=len(manifest),tests=73,checks=8,repos=repos,scope='Remove duplicated public nesting report data and validation, retain the raw geometric authority and owning boundary provenance, and admit the exact empty native boundary set',unresolved='The full exact-geometry implementation goal remains active; this cleanup does not claim full spatial BREP closure or a measured speedup.')
(A/f'{prefix}-reviewed.json').write_text(json.dumps(review,indent=2)+'\n')
print('Reviewed73 release cases,two pinned binaries,eight checks,baseline empty-input rejection,2048sources and30repositories')
