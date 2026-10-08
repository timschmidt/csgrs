from pathlib import Path
import hashlib,json,os,shutil,signal,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent;prefix='rational-image-scalar-view-20260928-v606';archive=A/'source-archives'/prefix;build=A/'build-workspace-20260925'
prior=json.loads((A/'common-point-incidence-broad-20260928-v593-terminal.json').read_text());assert prior['qualification_complete'] and prior['all_processes_reaped']
assert (A/'common-point-incidence-broad-20260928-v593-committed.json').exists()
assert json.loads((A/'represented-parameter-domain-baseline-20260928-v596-reaped.json').read_text())['outer_exit_code']==0
changed=json.loads((A/'parameter-construction-candidate-v595.json').read_text())
guard=json.loads((A/prior['source_manifest']).read_text());manifest={};assert not archive.exists()
for name,sha in guard.items():
 src=W/name;current=hashlib.sha256(src.read_bytes()).hexdigest();assert name in changed or current==sha,name;manifest[name]=current
 dst=archive/name;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,dst)
 target=build/name
 if target.read_bytes()!=src.read_bytes():shutil.copy2(src,target);os.utime(target,None)
(A/f'{prefix}-sources.json').write_text(json.dumps(manifest,indent=2)+'\n')
names={'hypercurve': ['rational_bezier_general::tests::rational_image_selection_reuses_learned_exact_scalar_views', 'rational_bezier_general::tests::conic_rational_image_separation_refines_past_the_old_limit', 'rational_bezier_general::tests::affine_subcurve_rejects_a_crossed_projective_pole', 'rational_bezier_general::tests::conic_parameter_primary_map_defers_fallback_image_polynomial', 'rational_bezier_general::tests::conic_parameter_refines_primary_map_before_constructing_fallback', 'rational_bezier_general::tests::endpoint_projective_cubic_correspondence_maps_both_orientations', 'rational_bezier_general::tests::exact_degree_elevated_line_recovers_linear_parameter_transport', 'rational_bezier_general::tests::exact_high_degree_rational_elevation_recovers_its_minimal_parameter_frame', 'rational_bezier_general::tests::implicit_conic_certificate_is_parameterization_independent_and_shared', 'rational_bezier_general::tests::implicit_conic_contacts_retain_source_parameter_point_image_first', 'rational_bezier_general::tests::injectivity_scope_preserves_every_parameter_of_a_collapsed_chart', 'rational_bezier_general::tests::mixed_weight_projective_overlap_preserves_the_finite_parameter_domain', 'rational_bezier_general::tests::range_projective_correspondence_maps_and_inverts_oriented_ranges', 'rational_bezier_general::tests::rational_parameter_image_map_reuses_quotient_authority_across_isolators', 'rational_bezier_general::tests::unit_weight_degree_one_curve_exposes_its_exact_line_parameterization', 'rational_bezier_general::tests::conic_dual_coordinate_sum_maps_endpoints_without_a_pole', 'rational_bezier_general::tests::exact_line_image_route_replays_algebraic_conic_contact', 'rational_bezier_general::tests::implicit_conic_route_replays_quadratic_line_contact', 'rational_bezier_general::tests::retained_circle_line_contacts_fall_back_at_an_omitted_chart_point', 'rational_bezier_general::tests::quotient_ring_rational_image_matches_exact_resultant_samples', 'rational_bezier_general::tests::quotient_ring_rational_image_retains_nonrational_source_coefficients', 'rational_bezier_general::tests::quotient_ring_rational_image_reuses_nonrational_source_scale', 'bezier_parameter::finite_interval_import_regression::represented_root_imports_admit_exact_values_in_the_requested_domain', 'bezier_parameter::conversion_tests::nonlinear_and_approximate_zeroes_do_not_create_scalar_witnesses', 'bezier_parameter::conversion_tests::progressive_refinement_matches_one_pass_proof_budget', 'bezier_parameter::conversion_tests::strict_linear_queries_retain_witnesses_after_rational_reconstruction_declines', 'bezier_parameter::conversion_tests::strict_scalar_equalities_retain_arbitrary_real_parameter_witnesses']}
new_cases=['rational_bezier_general::tests::rational_image_selection_reuses_learned_exact_scalar_views', 'rational_bezier_general::tests::conic_rational_image_separation_refines_past_the_old_limit']
full_targets=[]

report=dict(normal_production_build=True,source_manifest=f'{prefix}-sources.json',source_directory=str(archive),builds=[],cases=[],checks=[],expected_cases=names,all_processes_reaped=False,probe_complete=False,qualification_complete=False)
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()));cargo='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
def verify():
 for name,sha in manifest.items():
  for root in [W,archive,build]:assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,(root,name)
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
 log=A/f'{prefix}-build.log';row=run([cargo,'test','--lib',*[arg for target in names if target!='hypercurve' for arg in ['--test',target]],'--release','--all-features','--locked','--offline','--no-run','--message-format=json'],build/'hypercurve',log,900);report['builds'].append(row);save()
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
 report['test_listings']={}
 for target,cases in names.items():
  list_log=A/f'{prefix}-{target}-test-list.log'
  listed=run([row['binaries'][target]['path'],'--list'],archive/'hypercurve',list_log,30)
  report['test_listings'][target]=listed;save();assert listed['returncode']==0
  available={line.removesuffix(': test')for line in list_log.read_text().splitlines()if line.endswith(': test')}
  assert set(cases)<=available,(target,set(cases)-available)
  if target in full_targets:assert set(cases)==available,(target,available-set(cases))
 order=[('hypercurve',name)for name in new_cases]
 order.extend((target,name)for target,cases in names.items()for name in cases if 'trim_or_extend_analytic_parallel_support_corners_retain_algebraic_fillet_extensions'in name)
 order.extend((target,name)for target,cases in names.items()for name in cases if (target,name)not in order)
 assert len(order)==len(set(order))==sum(map(len,names.values()))
 save()

 for target,name in order:
  binary=Path(report['builds'][0]['binaries'][target]['path'])
  log=A/f'{prefix}-{name.split("::")[-1]}.log';case=run([str(binary),'--exact',name,'--nocapture','--test-threads=1'],archive/'hypercurve',log,45 if "conic_rational_image_separation_refines_past_the_old_limit" in name else 240);case.update(target=target,name=name,passed=case['returncode']==0 and 'test result: ok. 1 passed;'in log.read_text());report['cases'].append(case);save();print(name,case['returncode'],round(case['elapsed_seconds'],3),flush=True)
  if not case['passed']:
   print(log.read_text()[-1800:],flush=True)
   raise RuntimeError('parameter construction regression failed')
 report['probe_complete']=True;code=0 if all(row['passed']for row in report['cases'])else 1
 if code:raise RuntimeError('focused regression failure')
 report['qualification_complete']=False
except Exception as error:code=1;report['failure']=str(error)
finally:verify();report['all_processes_reaped']=True;save()
raise SystemExit(code)
