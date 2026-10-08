from pathlib import Path
import hashlib,json,re,subprocess,sys
A=Path(__file__).resolve().parent;W=A.parent;prefix='finite-parameter-interval-20260928-v577'
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def git(repo,*args):return subprocess.check_output(['git',*args],cwd=W/repo)
r=json.loads((A/f'{prefix}-terminal.json').read_text());assert r['qualification_complete'] and r['probe_complete'] and r['all_processes_reaped']
manifest=json.loads((A/r['source_manifest']).read_text())
for name,sha in manifest.items():
 for root in [W,Path(r['source_directory']),A/'build-workspace-20260925']:assert digest(root/name)==sha,(root,name)
assert len(r['builds'])==1 and len(r['checks'])==6 and len(manifest)==2047
for b in r['builds']:
 assert b['returncode']==0
 for artifact in b['binaries'].values():assert digest(Path(artifact['path']))==artifact['sha256']
expected={(target,name) for target,names in r['expected_cases'].items() for name in names}
assert len(r['cases'])==len(expected)==453+len(r['expected_cases']['hypercurve_bezier_algebraic_parameter'])
assert {(c['target'],c['name']) for c in r['cases']}==expected
for c in r['cases']:assert c['passed'] and c['returncode']==0 and re.search(r'test result: ok\. 1 passed;',(A/c['log']).read_text())
assert all(c['returncode']==0 for c in r['checks'])
prior=json.loads((A/'primitive-tangent-reuse-broad-20260928-v571-terminal.json').read_text());assert prior['all_processes_reaped'] and prior['qualification_complete']
old_manifest=json.loads((A/prior['source_manifest']).read_text());paths={'hypercurve': ['src/bezier_offset.rs', 'src/bezier_parameter.rs', 'src/bezier_region.rs', 'src/curve_region_boolean.rs', 'src/curve_support_intersection.rs', 'tests/hypercurve_bezier_algebraic_parameter.rs']}
assert {n for n,h in manifest.items()if h!=old_manifest.get(n)}=={'hypercurve/'+p for p in paths['hypercurve']}
for n,h in old_manifest.items():assert digest(Path(prior['source_directory'])/n)==h,n
assert len(r['builds'][0]['binaries'])==9
listing=r['test_listing'];assert listing['returncode']==0
listed=[line.removesuffix(': test')for line in(A/listing['log']).read_text().splitlines()if line.endswith(': test')]
assert listed==r['expected_cases']['hypercurve_bezier_algebraic_parameter']
assert len(listed)>=20
for name in ['finite_parameter_interval_contract::exterior_root_images_preserve_equations_and_native_curve_domains','finite_parameter_interval_contract::positive_unit_weights_do_not_admit_exterior_algebraic_poles','reversed_parameter_intervals_are_rejected']:assert name in listed
assert 'invalid_parameter_intervals_are_rejected' not in listed
assert 'bezier_parameter::finite_interval_import_regression::unit_import_keeps_its_domain_after_generic_interval_construction' in r['expected_cases']['hypercurve']

old_times={}
for baseline in ['primitive-tangent-reuse-broad-20260928-v571']:
 for case in json.loads((A/f'{baseline}-terminal.json').read_text())['cases']:old_times[case['name']]=case['elapsed_seconds']
regressions=[];workload_changes=[]
for case in r['cases']:
 old=old_times.get(case['name'])
 if old is not None and old>.1 and case['elapsed_seconds']>old*1.25+.2:regressions.append(dict(name=case['name'],before=old,after=case['elapsed_seconds']))
previous=json.loads((A/'primitive-tangent-reuse-broad-20260928-v571-repositories-after.json').read_text());repositories={};repos={}
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
review=dict(qualification=f'{prefix}-terminal.json',source_manifest=r['source_manifest'],outer_session_reaped=int(sys.argv[1]),validated_files=len(manifest),selected_geometry_tests=len(r['cases']),checks=len(r['checks']),repos=repos,scope='One ordered finite interval constructor replaces the unit-only/public versus exterior/private split; owning operations keep explicit domain/pole checks; full goal remains active',performance_changes=regressions,workload_changes=workload_changes)
(A/f'{prefix}-reviewed.json').write_text(json.dumps(review,indent=2)+'\n')
message=f"""Give exact parameter intervals one ordered finite domain

Remove the separate private exterior-interval constructor and update every caller. An interval now certifies ordering of exact bounds, while the native root importer explicitly enforces its unit domain. Correct the range documentation to include retained exterior charts.

Validation: {len(r['cases'])} release regressions across nine binaries and six static/downstream checks. The complete public algebraic-parameter suite includes negative and exterior roots, exact polynomial images, native curve-domain rejection, and true exterior rational poles despite positive unit-span weights. A private regression preserves the unit import restriction. No compatibility alias remains.
"""
(A/f'{prefix}-hypercurve-commit.txt').write_text(message)
print('Reviewed',len(manifest),'sources;',len(r['cases']),'domain/family tests;6 checks;',len(repositories),'repositories; performance changes',regressions)
