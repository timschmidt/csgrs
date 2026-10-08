from pathlib import Path
import hashlib,json,os,shutil,signal,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent;prefix='unit-domain-projection-20260928-v644';archive=A/'source-archives'/prefix;build=A/'build-workspace-20260925'
assert json.loads((A/'parameter-root-closure-20260928-v638-reaped.json').read_text())['outer_exit_code']==0
assert (A/'parameter-root-closure-20260928-v638-committed.json').exists()
assert json.loads((A/'retained-closure-probes-20260928-v639-reaped.json').read_text())['outer_exit_code']==1
assert json.loads((A/'unit-domain-projection-20260928-v642-reaped.json').read_text())['outer_exit_code']==1
prior=json.loads((A/'parameter-root-closure-20260928-v638-terminal.json').read_text());guard=json.loads((A/prior['source_manifest']).read_text())
fixture=A/'unit-domain-projection-candidate-v643.rs';fixture_sha=hashlib.sha256(fixture.read_bytes()).hexdigest();manifest={};production={};assert not archive.exists()
for name,sha in guard.items():
 src=W/name;data=src.read_bytes();assert hashlib.sha256(data).hexdigest()==sha,name;production[name]=sha
 if name=='hypercurve/src/bezier_offset.rs':data=fixture.read_bytes()
 manifest[name]=hashlib.sha256(data).hexdigest();dst=archive/name;dst.parent.mkdir(parents=True,exist_ok=True);dst.write_bytes(data)
 target=build/name
 if target.read_bytes()!=data:target.write_bytes(data);os.utime(target,None)
(A/f'{prefix}-sources.json').write_text(json.dumps(manifest,indent=2)+'\n')
(A/f'{prefix}-production-sources.json').write_text(json.dumps(production,indent=2)+'\n')
names=['bezier_offset::structural_overlap_trace_regression::'+suffix for suffix in ['monotone_reparameterization_preserves_structural_pair_crossings_strict','monotone_reparameterization_preserves_structural_pair_crossings_approximate','reversed_monotone_reparameterization_preserves_structural_pair_crossings_strict','reversed_monotone_reparameterization_preserves_structural_pair_crossings_approximate']]
names=['bezier_offset::conversion_tests::selected_structural_parallel_overlap_replays_off_diagonal_contacts', 'bezier_offset::retained_structural_pair_domain_regression::retained_structural_correspondence_preserves_off_diagonal_contact_domains', 'bezier_offset::conversion_tests::non_source_parallel_overlap_replays_the_radical_component', 'bezier_offset::conversion_tests::radical_component_saturation_retains_norm_intersections', 'bezier_offset::conversion_tests::analytic_parallel_self_intersection_removes_the_parameter_diagonal', 'bezier_offset::conversion_tests::exact_rational_parallel_self_intersections_reuse_rational_authority', 'bezier_offset::conversion_tests::ordered_self_contact_domains_retain_axis_roles_after_restriction', 'bezier_offset::conversion_tests::incident_pair_projection_accepts_a_non_source_radical_component', 'bezier_offset::conversion_tests::incident_non_source_parallel_overlap_reaches_the_common_projection', 'curve_region_boolean::certified_successor_tests::finite_parallel_self_contacts_retain_active_domain', 'bezier_offset::conversion_tests::odd_source_cusp_branches_do_not_saturate_a_false_source_diagonal', 'bezier_offset::conversion_tests::regular_ph_branch_pairs_retain_exact_cusp_incidence', 'bezier_offset::conversion_tests::nonzero_parallel_domains_exclude_undefined_normals_and_poles', 'bezier_offset::conversion_tests::nonzero_parallel_domains_partition_poles_on_components', 'bezier_offset::regular_parallel_contact_tests::stationary_component_queries_keep_one_sided_endpoints_and_exact_constraints', 'bezier_offset::regular_parallel_contact_tests::regular_source_cells_preserve_reversed_and_algebraic_range_boundaries', 'bezier_offset::regular_parallel_contact_tests::regular_source_frame_keeps_contact_scale_across_a_center_cusp', 'bezier_offset::regular_parallel_contact_tests::exterior_regular_pair_keeps_contacts_outside_ancestral_bounds']+names
report=dict(normal_production_build=False,source_manifest=f'{prefix}-sources.json',source_directory=str(archive),builds=[],cases=[],checks=[],expected_cases={'hypercurve':names},all_processes_reaped=False,probe_complete=False,qualification_complete=False)
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()));cargo='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
def verify():
 assert hashlib.sha256(fixture.read_bytes()).hexdigest()==fixture_sha
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
  raise RuntimeError('unit-domain candidate build failed')
 print('Candidate compiled',round(row['elapsed_seconds'],3),flush=True)
 artifact=None
 for line in log.read_text().splitlines():
  try:v=json.loads(line)
  except ValueError:continue
  if v.get('reason')=='compiler-artifact'and v.get('target',{}).get('name')=='hypercurve'and v.get('executable'):artifact=v
 assert artifact
 binary=A/f'{prefix}-hypercurve-tests';shutil.copy2(artifact['executable'],binary);row['binaries']={'hypercurve':dict(path=str(binary),sha256=hashlib.sha256(binary.read_bytes()).hexdigest())};save()
 listing=subprocess.check_output([str(binary),'--list'],text=True,cwd=archive/'hypercurve');assert all(name+': test' in listing for name in names)
 for name in names:
  log=A/f'{prefix}-{name.split("::")[-1]}.log';case=run([str(binary),'--exact',name,'--nocapture','--test-threads=1'],archive/'hypercurve',log,180);case.update(target='hypercurve',name=name,passed=case['returncode']==0 and 'test result: ok. 1 passed;'in log.read_text());report['cases'].append(case);save();print(name,case['returncode'],round(case['elapsed_seconds'],3),flush=True)
  if not case['passed']:
   print(log.read_text()[-2500:],flush=True);raise RuntimeError('focused regression failed: '+name)
 report['probe_complete']=True;code=0 if all(case['passed']for case in report['cases'])else 1
except Exception as error:code=1;report['failure']=str(error);print(str(error),flush=True)
finally:verify();report['all_processes_reaped']=True;save()
raise SystemExit(code)
