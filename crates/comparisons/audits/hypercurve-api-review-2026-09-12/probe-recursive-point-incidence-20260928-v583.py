from pathlib import Path
import hashlib,json,os,shutil,signal,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent;prefix='recursive-point-incidence-20260928-v583';archive=A/'source-archives'/prefix;build=A/'build-workspace-20260925'
prior=json.loads((A/'ordered-field-gcd-20260928-v581-terminal.json').read_text())
assert (A/'recursive-point-and-cell-work-20260928-v575-reaped.json').exists()
guard=json.loads((A/prior['source_manifest']).read_text());manifest={};production={};assert not archive.exists()
for name,sha in guard.items():
 src=W/name;data=src.read_bytes();current=hashlib.sha256(data).hexdigest()
 assert current==sha or name in ['hypercurve/src/bezier_offset.rs','hypercurve/src/curve_fillet.rs'],name
 production[name]=current;manifest[name]=current
 dst=archive/name;dst.parent.mkdir(parents=True,exist_ok=True);dst.write_bytes(data)
 target=build/name
 if target.read_bytes()!=data:target.write_bytes(data);os.utime(target,None)
(A/f'{prefix}-sources.json').write_text(json.dumps(manifest,indent=2)+'\n')
(A/f'{prefix}-production-sources.json').write_text(json.dumps(production,indent=2)+'\n')
names={'hypercurve': ['curve::curve_fillet::stationary_recursive_point_constraint_regression::stationary_fillet_family_accepts_independent_recursive_point_constraints', 'bezier_offset::regular_parallel_contact_tests::recursive_incidence_preserves_selected_point_fields', 'bezier_offset::regular_parallel_contact_tests::recursive_incidence_keeps_stationary_and_collapsed_semantics', 'bezier_offset::regular_parallel_contact_tests::recursive_incidence_preserves_exterior_domains_and_pole_barriers', 'bezier_parameter::finite_interval_import_regression::unit_import_keeps_its_domain_after_generic_interval_construction', 'bezier_offset::regular_parallel_contact_tests::primitive_source_frames_share_factorization_across_distances_and_branches', 'curve::curve_fillet::stationary_retained_point_constraint_regression::stationary_fillet_family_accepts_independent_retained_point_constraints', 'bezier_parameter::finite_field_bernstein_regression::finite_field_isolation_preserves_represented_contacts_near_domain_boundaries', 'bezier_parameter::finite_field_bernstein_regression::finite_field_isolation_reuses_original_polynomial_and_simple_root_proof', 'bezier_parameter::finite_field_bernstein_regression::finite_field_isolation_keeps_repeated_roots_and_closed_boundaries', 'curve::curve_fillet::stationary_family_composition_regression::normalized_region_selects_and_reuses_a_stationary_fillet_family', 'bezier_offset::conversion_tests::point_incidence_checks_regular_frames_at_each_isolated_contact', 'bezier_offset::conversion_tests::algebraic_cusp_semicircle_point_incidence_selects_exactly_one_half', 'bezier_offset::conversion_tests::correlated_point_incidence_retains_exterior_contact_parameters', 'bezier_offset::conversion_tests::nonrepresented_center_transverse_chamfer_inverts_by_correlated_point', 'bezier_offset::conversion_tests::point_incidence_visits_a_collapsed_circle_as_a_complete_domain', 'bezier_offset::conversion_tests::represented_circle_retained_point_incidence_observes_requested_policy', 'bezier_offset::conversion_tests::retained_point_incidence_visits_all_preimages_and_can_stop_at_one', 'curve::curve_fillet::tests::joined_path_selects_and_replays_a_continuous_fillet_family', 'curve::curve_fillet::tests::normalized_region_selects_and_reuses_a_continuous_fillet_family', 'curve_region_boolean::certified_successor_tests::boundary_probe_reuses_endpoint_incidence_and_keeps_residual_contacts', 'bezier_offset::conversion_tests::correlated_point_membership_replays_exterior_rational_domains', 'bezier_offset::conversion_tests::retained_point_membership_preserves_domains_and_regular_barriers', 'curve::curve_fillet::stationary_continuous_family_regression::continuous_fillets_preserve_stationary_reparameterization']}
report=dict(normal_production_build=True,source_manifest=f'{prefix}-sources.json',source_directory=str(archive),builds=[],cases=[],checks=[],expected_cases=names,all_processes_reaped=False,probe_complete=False,qualification_complete=False)
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()));cargo='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
def verify():
 for name,sha in manifest.items():
  for root in [archive,build]:assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,(root,name)
 for name,sha in production.items():assert hashlib.sha256((W/name).read_bytes()).hexdigest()==sha,(W,name)
def save():(A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
def run(command,cwd,log,timeout):
 start=time.monotonic()
 with log.open('w')as out:
  process=subprocess.Popen(command,cwd=cwd,stdout=out,stderr=subprocess.STDOUT,env=env,start_new_session=True)
  try:code=process.wait(timeout=timeout)
  except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait();code=124
  except BaseException:os.killpg(process.pid,signal.SIGKILL);process.wait();raise
 return dict(command=command,returncode=code,log=log.name,elapsed_seconds=time.monotonic()-start)
verify();save();code=0
try:
 log=A/f'{prefix}-build.log';row=run([cargo,'test','--lib','--release','--all-features','--locked','--offline','--no-run','--message-format=json'],build/'hypercurve',log,900);report['builds'].append(row);save()
 if row['returncode']:
  for line in log.read_text().splitlines():
   try:v=json.loads(line)
   except ValueError:continue
   if v.get('reason')=='compiler-message'and v['message']['level']=='error':print(v['message'].get('rendered','')[:3000],flush=True)
  raise RuntimeError('fixture build failed')
 artifacts={}
 for line in log.read_text().splitlines():
  try:v=json.loads(line)
  except ValueError:continue
  target=v.get('target',{}).get('name')
  if v.get('reason')=='compiler-artifact' and target in names and v.get('executable'):artifacts[target]=v
 assert set(artifacts)==set(names)
 row['binaries']={}
 for target,artifact in artifacts.items():
  binary=A/f'{prefix}-{target}-tests';shutil.copy2(artifact['executable'],binary);row['binaries'][target]=dict(path=str(binary),sha256=hashlib.sha256(binary.read_bytes()).hexdigest());save()
 for target,cases in names.items():
  binary=Path(report['builds'][0]['binaries'][target]['path'])
  for name in cases:
   log=A/f'{prefix}-{name.split("::")[-1]}.log';case=run([str(binary),'--exact',name,'--nocapture','--test-threads=1'],archive/'hypercurve',log,240);case.update(target=target,name=name,passed=case['returncode']==0 and 'test result: ok. 1 passed;'in log.read_text());report['cases'].append(case);save();print(name,case['returncode'],round(case['elapsed_seconds'],3),flush=True)
   if not case['passed']:print(log.read_text()[-1800:],flush=True)
   for line in log.read_text().splitlines():
    if line.startswith('SOURCE_CELL_WORK '):print(line,flush=True)
 report['probe_complete']=True
 if not all(row['passed']for row in report['cases']):raise RuntimeError('focused regression failure')
 report['qualification_complete']=False
except Exception as error:code=1;report['failure']=str(error)
finally:verify();report['all_processes_reaped']=True;save()
raise SystemExit(code)
