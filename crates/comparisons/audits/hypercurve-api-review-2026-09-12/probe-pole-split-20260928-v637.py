from pathlib import Path
import hashlib,json,os,shutil,signal,subprocess,time

A=Path(__file__).resolve().parent;W=A.parent
prefix='pole-split-20260928-v637';archive=A/'source-archives'/prefix;build=A/'build-workspace-20260925'
assert json.loads((A/'parameter-root-closure-20260928-v633-reaped.json').read_text())['outer_exit_code']==1
assert json.loads((A/'pole-split-20260928-v636-reaped.json').read_text())['outer_exit_code']==1
prior=json.loads((A/'parameter-root-closure-20260928-v633-terminal.json').read_text())
assert not prior['qualification_complete'] and prior['all_processes_reaped']
guard=json.loads((A/prior['source_manifest']).read_text())
changed={'hypercurve/src/bezier_split.rs','hypercurve/src/rational_bezier.rs','hypercurve/tests/hypercurve_bezier_split_materialization.rs'}
manifest={};assert not archive.exists()
for name,sha in guard.items():
 src=W/name;data=src.read_bytes();current=hashlib.sha256(data).hexdigest();assert name in changed or current==sha,name;manifest[name]=current
 dst=archive/name;dst.parent.mkdir(parents=True,exist_ok=True);dst.write_bytes(data)
 target=build/name
 if target.read_bytes()!=data:target.write_bytes(data);os.utime(target,None)
assert {name for name,sha in manifest.items()if sha!=guard[name]}==changed
(A/f'{prefix}-sources.json').write_text(json.dumps(manifest,indent=2)+'\n')
previous=prior
names={'hypercurve_bezier_split_materialization':previous['expected_cases']['hypercurve_bezier_split_materialization'], 'hypercurve':[
 'bezier_offset::conversion_tests::denominator_sign_excludes_a_pole_outside_selected_bounds',
 'bezier_offset::conversion_tests::exterior_rational_analytic_points_use_the_selected_denominator_sign',
 'rational_bezier_general::tests::denominator_sign_tracks_the_requested_range_and_keeps_unit_cache_scope',
 'rational_bezier_general::tests::affine_subcurve_materializes_a_finite_rational_parabola_extension',
 'rational_bezier_general::tests::affine_subcurve_rejects_a_crossed_projective_pole',
 'rational_bezier_general::tests::affine_subcurve_retains_zero_intermediate_weight_without_a_pole',
 'bezier_offset::regular_parallel_contact_tests::regular_source_frames_reject_poles_in_every_parameter_authority',
 'bezier_offset::conversion_tests::retained_incident_domains_admit_exterior_affine_endpoints',
 'bezier_parameter::conversion_tests::projective_images_preserve_exterior_singletons_across_unused_poles',
 'bezier_region::tests::retained_rational_chamfer_extends_exact_and_algebraic_pre_pole_roots',
]}
full_targets={'hypercurve_bezier_split_materialization'}
report=dict(normal_production_build=True,source_manifest=f'{prefix}-sources.json',source_directory=str(archive),builds=[],cases=[],checks=[],test_listings={},expected_cases=names,all_processes_reaped=False,probe_complete=False,qualification_complete=False)
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
cargo='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def verify():
 for name,sha in manifest.items():
  for root in [W,archive,build]:assert digest(root/name)==sha,(root,name)
def save():(A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
def run(command,cwd,log,timeout):
 start=time.monotonic()
 with log.open('w')as out:
  process=subprocess.Popen(command,cwd=cwd,stdout=out,stderr=subprocess.STDOUT,env=env,start_new_session=True)
  try:code=process.wait(timeout=timeout)
  except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait();code=124
  except BaseException:os.killpg(process.pid,signal.SIGKILL);process.wait();raise
 return dict(command=command,returncode=code,log=log.name,elapsed_seconds=time.monotonic()-start)
def compile_tests(crate,targets):
 log=A/f'{prefix}-{crate}-build.log'
 args=[cargo,'test','--lib',*[arg for target in targets if target!=crate for arg in ['--test',target]],'--release','--all-features','--locked','--offline','--no-run','--message-format=json']
 row=run(args,build/crate,log,900);report['builds'].append(row);save()
 if row['returncode']:
  for line in log.read_text().splitlines():
   try:v=json.loads(line)
   except ValueError:continue
   if v.get('reason')=='compiler-message'and v['message']['level']=='error':print(v['message'].get('rendered','')[:3000],flush=True)
  raise RuntimeError(crate+' release build failed')
 artifacts={}
 for line in log.read_text().splitlines():
  try:v=json.loads(line)
  except ValueError:continue
  target=v.get('target',{}).get('name')
  if v.get('reason')=='compiler-artifact'and target in targets and v.get('executable'):artifacts[target]=v
 assert set(artifacts)==set(targets)
 row['binaries']={}
 for target,artifact in artifacts.items():
  binary=A/f'{prefix}-{target}-tests';shutil.copy2(artifact['executable'],binary);row['binaries'][target]=dict(path=str(binary),sha256=digest(binary));save()
  log=A/f'{prefix}-{target}-test-list.log';listing=run([str(binary),'--list'],archive/crate,log,30);report['test_listings'][target]=listing;save();assert listing['returncode']==0
  available={line.removesuffix(': test')for line in log.read_text().splitlines()if line.endswith(': test')}
  if target=='hypersolve':
   modules=('root_isolation::','root_sign::','ordered_field_roots::','algebraic_binary::','algebraic_tensor_image::','algebraic_polynomial_image::','algebraic_rational_image::','tensor_resultant::')
   names[target]=sorted(name for name in available if name.startswith(modules))
   assert any('primitive_root_evidence_regression::'in name for name in names[target])
  else:
   assert set(names[target])<=available,(target,set(names[target])-available)
   if target in full_targets:assert set(names[target])==available,(target,available-set(names[target]))
 print(crate,'compiled',round(row['elapsed_seconds'],3),flush=True)
 return row['binaries']
verify();save();code=0
try:
 binaries=compile_tests('hypercurve',list(names))
 pole=('hypercurve_bezier_split_materialization','rational_algebraic_boundary_with_zero_denominator_returns_explicit_uncertainty')
 order=[pole]+[(target,name)for target,tests in names.items()for name in tests if (target,name)!=pole]
 save();print('Focused inventory',len(order),'tests',len(binaries),'binaries',flush=True)
 for target,name in order:
  crate='hypersolve'if target=='hypersolve'else'hypercurve'
  suffix=hashlib.sha256((target+'::'+name).encode()).hexdigest()[:10]
  log=A/f'{prefix}-{name.split("::")[-1]}-{suffix}.log'
  case=run([binaries[target]['path'],'--exact',name,'--nocapture','--test-threads=1'],archive/crate,log,240)
  case.update(target=target,name=name,passed=case['returncode']==0 and 'test result: ok. 1 passed;'in log.read_text())
  report['cases'].append(case);save();print(name,case['returncode'],round(case['elapsed_seconds'],3),flush=True)
  if not case['passed']:print(log.read_text()[-5000:],flush=True);raise RuntimeError('release regression failed: '+name)
 report['probe_complete']=True;save()
except Exception as error:code=1;report['failure']=str(error);print(str(error),flush=True)
finally:verify();report['all_processes_reaped']=True;save()
raise SystemExit(code)
