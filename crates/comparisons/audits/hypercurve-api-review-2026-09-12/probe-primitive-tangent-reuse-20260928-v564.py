from pathlib import Path
import hashlib,json,os,shutil,signal,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent;prefix='primitive-tangent-reuse-20260928-v564';archive=A/'source-archives'/prefix;build=A/'build-workspace-20260925'
prior=json.loads((A/'finite-field-root-identity-20260928-v563-terminal.json').read_text());assert prior['qualification_complete'] and prior['all_processes_reaped']
assert (A/'finite-field-root-identity-20260928-v563-committed.json').exists()
composition=prior
assert composition['all_processes_reaped'] and composition['probe_complete'] and all(c['passed'] for c in composition['cases'])
base=json.loads((A/'primitive-tangent-reuse-v542-base.json').read_text())
assert hashlib.sha256((W/base['path']).read_bytes()).hexdigest()==base['sha256']
candidate=A/'primitive-tangent-reuse-v542-candidate.rs'
candidate_sha=hashlib.sha256(candidate.read_bytes()).hexdigest()
guard=json.loads((A/prior['source_manifest']).read_text());manifest={};production={};assert not archive.exists()
for name,sha in guard.items():
 src=W/name;current=hashlib.sha256(src.read_bytes()).hexdigest();assert current==sha,name;production[name]=current
 data=candidate.read_bytes() if name==base['path'] else src.read_bytes()
 manifest[name]=hashlib.sha256(data).hexdigest()
 dst=archive/name;dst.parent.mkdir(parents=True,exist_ok=True);dst.write_bytes(data)
 target=build/name
 if target.read_bytes()!=data:target.write_bytes(data);os.utime(target,None)
(A/f'{prefix}-sources.json').write_text(json.dumps(manifest,indent=2)+'\n')
(A/f'{prefix}-production-sources.json').write_text(json.dumps(production,indent=2)+'\n')
names={'hypercurve':[
 'bezier_offset::regular_parallel_contact_tests::primitive_source_frames_share_factorization_across_distances_and_branches',
 'bezier_offset::conversion_tests::opaque_retracing_line_uses_constant_direction_rank_component',
 'bezier_offset::regular_parallel_contact_tests::regular_source_frames_retain_unprojectable_selected_parameters',
 'bezier_offset::regular_parallel_contact_tests::regular_source_frames_accept_every_retained_parameter_authority',
 'bezier_offset::regular_parallel_contact_tests::regular_source_cells_preserve_reversed_and_algebraic_range_boundaries',
 'bezier_offset::regular_parallel_contact_tests::rational_pair_endpoint_tangents_preserve_both_operand_roles',
 'bezier_offset::regular_parallel_contact_tests::general_regular_pair_replays_unequal_contact_scale_changes',
 'bezier_offset::regular_parallel_contact_tests::stationary_component_queries_keep_one_sided_endpoints_and_exact_constraints',
 'curve::curve_fillet::tests::joined_path_selects_and_replays_a_continuous_fillet_family',
 'curve::curve_fillet::tests::nonlinear_linear_fillet_components_retain_unique_contacts_and_tangents',
 'bezier_offset::conversion_tests::regular_ph_branch_pairs_retain_exact_cusp_incidence',
 'bezier_offset::conversion_tests::independent_oblique_chords_support_constrained_fillet_families',
 'curve::curve_fillet::stationary_continuous_family_regression::continuous_fillets_preserve_stationary_reparameterization',
 'curve::curve_fillet::tests::normalized_region_selects_and_reuses_a_continuous_fillet_family',
 'curve::curve_fillet::stationary_family_composition_regression::normalized_region_selects_and_reuses_a_stationary_fillet_family',
]}
report=dict(normal_production_build=False,source_manifest=f'{prefix}-sources.json',source_directory=str(archive),builds=[],cases=[],checks=[],expected_cases=names,all_processes_reaped=False,probe_complete=False,qualification_complete=False)
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()));cargo='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
def verify():
 assert hashlib.sha256(candidate.read_bytes()).hexdigest()==candidate_sha
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
 old_times={c['name']:c['elapsed_seconds'] for baseline in (prior,composition) for c in baseline['cases']}
 report['performance']=[dict(name=c['name'],before=old_times[c['name']],after=c['elapsed_seconds']) for c in report['cases'] if c['name'] in old_times and old_times[c['name']]>.1]
 report['probe_complete']=True;code=0 if all(row['passed']for row in report['cases'])else 1
 if code:raise RuntimeError('focused regression failure')
 report['qualification_complete']=False
except Exception as error:code=1;report['failure']=str(error)
finally:verify();report['all_processes_reaped']=True;save()
raise SystemExit(code)
