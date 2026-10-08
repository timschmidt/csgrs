from pathlib import Path
import hashlib,json,re,subprocess,sys
A=Path(__file__).resolve().parent;W=A.parent;prefix='parameter-construction-20260928-v605'
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def git(repo,*args):return subprocess.check_output(['git',*args],cwd=W/repo)
r=json.loads((A/f'{prefix}-terminal.json').read_text());assert r['qualification_complete'] and r['probe_complete'] and r['all_processes_reaped']
manifest=json.loads((A/r['source_manifest']).read_text())
for name,sha in manifest.items():
 for root in [W,Path(r['source_directory']),A/'build-workspace-20260925']:assert digest(root/name)==sha,(root,name)
assert len(r['builds'])==1 and len(r['checks'])==7 and len(manifest)==2047
for b in r['builds']:
 assert b['returncode']==0
 for artifact in b['binaries'].values():assert digest(Path(artifact['path']))==artifact['sha256']
focus=json.loads((A/'rational-image-selection-20260928-v608-terminal.json').read_text());assert focus['all_processes_reaped'] and focus['probe_complete']
assert json.loads((A/focus['source_manifest']).read_text())==manifest
assert focus['builds'][0]['binaries']['hypercurve']['sha256']==r['builds'][0]['binaries']['hypercurve']['sha256']
reused=[c for c in r['cases']if 'reused_from'in c];assert len(reused)==len(focus['cases'])==29
for c in reused:
 assert c['reused_from']=='rational-image-selection-20260928-v608-terminal.json'
 assert any((old['target'],old['name'],old['log'],old['elapsed_seconds'])==(c['target'],c['name'],c['log'],c['elapsed_seconds'])and old['passed']for old in focus['cases'])
expected={(target,name) for target,names in r['expected_cases'].items() for name in names}
assert len(r['cases'])==len(expected)==622
assert {(c['target'],c['name']) for c in r['cases']}==expected
for c in r['cases']:assert c['passed'] and c['returncode']==0 and re.search(r'test result: ok\. 1 passed;',(A/c['log']).read_text())
assert all(c['returncode']==0 for c in r['checks'])
prior=json.loads((A/'common-point-incidence-broad-20260928-v593-terminal.json').read_text());assert prior['all_processes_reaped'] and prior['qualification_complete']
old_manifest=json.loads((A/prior['source_manifest']).read_text());paths={'hypercurve': ['benches/bezier_algebraic_parameter.rs', 'benches/bezier_arrangement.rs', 'benches/bezier_region.rs', 'benches/bezier_split_materialization.rs', 'benches/curve_region_boolean_batch.rs', 'benches/offset.rs', 'fuzz/fuzz_targets/bezier_arrangement.rs', 'fuzz/fuzz_targets/bezier_region.rs', 'fuzz/fuzz_targets/bezier_split_materialization.rs', 'src/bezier_offset.rs', 'src/bezier_parameter.rs', 'src/bezier_region.rs', 'src/curve_region_trim.rs', 'src/curve_subdivision.rs', 'src/curve_support_intersection.rs', 'src/rational_bezier_general.rs', 'tests/hypercurve_analytic_parallel_region.rs', 'tests/hypercurve_bezier_algebraic_parameter.rs', 'tests/hypercurve_bezier_arrangement.rs', 'tests/hypercurve_bezier_region.rs', 'tests/hypercurve_bezier_split_materialization.rs'], 'hypersolve': ['src/algebraic.rs']}
assert {n for n,h in manifest.items()if h!=old_manifest.get(n)}=={repo+'/'+p for repo,ps in paths.items()for p in ps}
for n,h in old_manifest.items():assert digest(Path(prior['source_directory'])/n)==h,n
assert len(r['builds'][0]['binaries'])==12
for target,listing in r['test_listings'].items():
 assert listing['returncode']==0
 available={line.removesuffix(': test')for line in(A/listing['log']).read_text().splitlines()if line.endswith(': test')}
 assert set(r['expected_cases'][target])<=available
assert len(r['test_listings'])==12

old_times={}
for baseline in ['common-point-incidence-broad-20260928-v593']:
 for case in json.loads((A/f'{baseline}-terminal.json').read_text())['cases']:old_times[case['name']]=case['elapsed_seconds']
regressions=[];workload_changes=[]
for case in r['cases']:
 old=old_times.get(case['name'])
 if old is not None and old>.1 and case['elapsed_seconds']>old*1.25+.2:regressions.append(dict(name=case['name'],before=old,after=case['elapsed_seconds']))
previous=json.loads((A/'common-point-incidence-broad-20260928-v593-repositories-after.json').read_text());repositories={};repos={}
for name,old in previous.items():
 head=git(name,'rev-parse','HEAD').decode().strip();status=git(name,'status','--porcelain=v1').decode();assert head==old['head'],name
 if name in paths:
  assert status==''.join(' M '+p+'\n'for p in paths[name]),(name,status)
  assert not git(name,'diff','--cached','--name-only').strip()
  subprocess.run(['git','diff','--check'],cwd=W/name,check=True)
  repos[name]=dict(parent=head,paths={p:manifest[name+'/'+p]for p in paths[name]})
 else:assert not status,name
 repositories[name]=dict(head=head,status=status)
(A/f'{prefix}-repositories-before.json').write_text(json.dumps(repositories,indent=2)+'\n')
review=dict(qualification=f'{prefix}-terminal.json',source_manifest=r['source_manifest'],outer_session_reaped=int(sys.argv[1]),validated_files=len(manifest),selected_geometry_tests=len(r['cases']),checks=len(r['checks']),repos=repos,scope='Parameter values retain arbitrary exact scalar or selected-root meaning; operation importers own their unit-domain admission; all callers use enum variants directly; full goal remains active',performance_changes=regressions,workload_changes=workload_changes)
(A/f'{prefix}-reviewed.json').write_text(json.dumps(review,indent=2)+'\n')
message=f"""Unify parameter construction and retain exact image selection

Construct BezierParameter2 through its existing Exact and Algebraic variants. Remove the wrappers that respectively hid a unit-interval restriction and merely forwarded to the enum. Keep domain admission explicit in the root importer, rational image maps and finite line contacts. Update all source, test, benchmark and fuzz callers directly and remove their obsolete forwarding helpers.

Rational image selection now excludes only certified disjoint roots and selects a unique possible owner within the complete unit-domain inventory. Learned scalar views and retained isolators obey the same rule, competing deflated isolators refine alongside the source, and exact endpoint images replay the numerator relation at the selected source. This prevents both endless refinement after learning a scalar view and premature selection when a competing root remains possible. The close-image regression timed out on the committed baseline and now completes in milliseconds with the opposite conjugate warmed first.

Validation: {len(r['cases'])} release regressions across twelve binaries and seven static/downstream checks, including all affected public integration tests, both all-target feature configurations, four fuzz targets and downstream compilation. Twenty-nine focused receipts are reused only after matching every source hash and the exact library binary hash. Regressions cover learned irrational/transcendental scalar views, repeated close-image selection, exact unit endpoints, exterior images and a deflated competing isolator. An independently checked importer regression covers rational, irrational and transcendental exact point, owned endpoint and deflated linear representations in native and unrestricted finite domains.
"""
(A/f'{prefix}-hypercurve-commit.txt').write_text(message)
(A/f'{prefix}-hypersolve-commit.txt').write_text("Clarify the finite domain of retained algebraic roots\n\nAlgebraicRootRepresentation stores a certified finite interval or exact point; it does not impose a unit interval. Keep that domain restriction with geometric importers. Validated with Hypercurve's represented-root import regression and strict documentation checks.\n")
print('Reviewed',len(manifest),'sources;',len(r['cases']),'domain/family tests;7 checks;',len(repositories),'repositories; performance changes',regressions)
