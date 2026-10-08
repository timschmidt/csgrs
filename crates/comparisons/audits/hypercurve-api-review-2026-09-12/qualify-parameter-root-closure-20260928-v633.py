from pathlib import Path
import hashlib,json,os,shutil,signal,subprocess,time

A=Path(__file__).resolve().parent;W=A.parent
prefix='parameter-root-closure-20260928-v633';archive=A/'source-archives'/prefix;build=A/'build-workspace-20260925'
assert json.loads((A/'primitive-root-evidence-minimal-20260928-v631-reaped.json').read_text())['outer_exit_code']==0
prior=json.loads((A/'common-point-incidence-broad-20260928-v593-terminal.json').read_text())
assert prior['qualification_complete'] and prior['all_processes_reaped']
guard=json.loads((A/prior['source_manifest']).read_text())
changed=set(json.loads((A/'parameter-construction-candidate-v595.json').read_text()))|{'hypersolve/src/root_isolation.rs'}
manifest={};assert not archive.exists()
for name,sha in guard.items():
 src=W/name;data=src.read_bytes();current=hashlib.sha256(data).hexdigest();assert name in changed or current==sha,name;manifest[name]=current
 dst=archive/name;dst.parent.mkdir(parents=True,exist_ok=True);dst.write_bytes(data)
 target=build/name
 if target.read_bytes()!=data:target.write_bytes(data);os.utime(target,None)
assert {name for name,sha in manifest.items()if sha!=guard[name]}==changed
(A/f'{prefix}-sources.json').write_text(json.dumps(manifest,indent=2)+'\n')
previous=json.loads((A/'parameter-construction-20260928-v605-terminal.json').read_text())
names=previous['expected_cases']
names['hypercurve'].append('bezier_offset::retained_structural_pair_domain_regression::retained_structural_correspondence_preserves_off_diagonal_contact_domains')
assert sum(map(len,names.values()))==623
full_targets={'hypercurve_bezier_algebraic_parameter','hypercurve_bezier_arrangement','hypercurve_bezier_region','hypercurve_bezier_split_materialization','hypercurve_analytic_parallel_region'}
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
 hc_targets=list(names)
 binaries=compile_tests('hypersolve',['hypersolve'])
 binaries.update(compile_tests('hypercurve',hc_targets))
 new_hc=[
  'bezier_parameter::finite_interval_import_regression::represented_root_imports_admit_exact_values_in_the_requested_domain',
  'rational_bezier_general::tests::rational_image_selection_certifies_unit_endpoints_before_admission',
  'rational_bezier_general::tests::rational_image_selection_refines_competing_deflated_isolators',
  'rational_bezier_general::tests::rational_image_selection_reuses_learned_exact_scalar_views',
  'rational_bezier_general::tests::conic_rational_image_separation_refines_past_the_old_limit',
  'bezier_offset::retained_structural_pair_domain_regression::retained_structural_correspondence_preserves_off_diagonal_contact_domains',
 ]
 order=[('hypersolve',name)for name in names['hypersolve']if 'primitive_root_evidence_regression::'in name]
 order.extend(('hypercurve',name)for name in new_hc)
 order.extend(('hypercurve_analytic_parallel_region',name)for name in ['radical_parallel_cusp_offsets_exactly_under_both_policies','radical_parallel_cusp_spans_connect_under_both_policies'])
 order.extend(('hypersolve',name)for name in names['hypersolve']if ('hypersolve',name)not in order)
 order.extend((target,name)for target,tests in names.items()for name in tests if (target,name)not in order)
 assert len(order)==len(set(order))==sum(map(len,names.values()))
 save();print('Qualification inventory',len(order),'tests',len(binaries),'binaries; 9 checks',flush=True)
 for target,name in order:
  crate='hypersolve'if target=='hypersolve'else'hypercurve'
  suffix=hashlib.sha256((target+'::'+name).encode()).hexdigest()[:10]
  log=A/f'{prefix}-{name.split("::")[-1]}-{suffix}.log'
  case=run([binaries[target]['path'],'--exact',name,'--nocapture','--test-threads=1'],archive/crate,log,240)
  case.update(target=target,name=name,passed=case['returncode']==0 and 'test result: ok. 1 passed;'in log.read_text())
  report['cases'].append(case);save();print(name,case['returncode'],round(case['elapsed_seconds'],3),flush=True)
  if not case['passed']:print(log.read_text()[-5000:],flush=True);raise RuntimeError('release regression failed: '+name)
 report['probe_complete']=True;save()
 checks=[
  ('hypercurve-clippy-all-features',[cargo,'clippy','--all-targets','--all-features','--locked','--offline','--','-D','warnings'],'hypercurve'),
  ('hypercurve-clippy-no-default',[cargo,'clippy','--all-targets','--no-default-features','--locked','--offline','--','-D','warnings'],'hypercurve'),
  ('format',['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustfmt','--edition','2024','--config','skip_children=true','--check',*[str(build/n)for n in sorted(changed)]],'hypercurve'),
  ('fuzz-check',[cargo,'check','--manifest-path','fuzz/Cargo.toml','--bin','curve_string_editing','--bin','bezier_arrangement','--bin','bezier_region','--bin','bezier_split_materialization','--locked','--offline'],'hypercurve'),
  ('hypercurve-documentation',[cargo,'doc','--no-deps','--all-features','--locked','--offline'],'hypercurve'),
  ('hypersolve-clippy-all-features',[cargo,'clippy','--all-targets','--all-features','--locked','--offline','--','-D','warnings'],'hypersolve'),
  ('hypersolve-clippy-no-default',[cargo,'clippy','--all-targets','--no-default-features','--locked','--offline','--','-D','warnings'],'hypersolve'),
  ('hypersolve-documentation',[cargo,'doc','--no-deps','--all-features','--locked','--offline'],'hypersolve'),
  ('hyperbrep-check',[cargo,'check','--all-targets','--all-features','--locked','--offline'],'hyperbrep'),
 ]
 env['RUSTDOCFLAGS']='-D warnings'
 for label,command,crate in checks:
  log=A/f'{prefix}-{label}.log';row=run(command,build/crate,log,900);row['label']=label;report['checks'].append(row);save();print(label,row['returncode'],round(row['elapsed_seconds'],3),flush=True)
  if row['returncode']:print(log.read_text()[-5000:],flush=True);raise RuntimeError(label+' failed')
 report['qualification_complete']=True
except Exception as error:code=1;report['failure']=str(error);print(str(error),flush=True)
finally:verify();report['all_processes_reaped']=True;save()
raise SystemExit(code)
