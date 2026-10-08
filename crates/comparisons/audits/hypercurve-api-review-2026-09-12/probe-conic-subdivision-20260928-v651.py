from pathlib import Path
import hashlib,json,os,shutil,signal,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent;prefix='conic-subdivision-20260928-v651';archive=A/'source-archives'/prefix;build=A/'build-workspace-20260925'
assert json.loads((A/'unit-domain-core-20260928-v649-reaped.json').read_text())['outer_exit_code']==0
assert (A/'unit-domain-core-20260928-v649-committed.json').exists()
prior=json.loads((A/'unit-domain-core-20260928-v649-terminal.json').read_text());guard=json.loads((A/prior['source_manifest']).read_text())
candidate=A/'conic-subdivision-candidate-v650';fixtures={str(p.relative_to(candidate)):hashlib.sha256(p.read_bytes()).hexdigest()for p in candidate.rglob('*.rs')};assert len(fixtures)==7
base=json.loads((A/'conic-subdivision-candidate-v650-base.json').read_text())
for name,sha in base.items():assert guard['hypercurve/'+name]==sha,name
manifest={};production={};assert not archive.exists()
for name,sha in guard.items():
 src=W/name;data=src.read_bytes();assert hashlib.sha256(data).hexdigest()==sha,name;production[name]=sha
 if name in fixtures:data=(candidate/name).read_bytes()
 manifest[name]=hashlib.sha256(data).hexdigest();dst=archive/name;dst.parent.mkdir(parents=True,exist_ok=True);dst.write_bytes(data)
 target=build/name
 if target.read_bytes()!=data:target.write_bytes(data);os.utime(target,None)
(A/f'{prefix}-sources.json').write_text(json.dumps(manifest,indent=2)+'\n')
(A/f'{prefix}-production-sources.json').write_text(json.dumps(production,indent=2)+'\n')
(A/f'{prefix}-candidate-sources.json').write_text(json.dumps(fixtures,indent=2)+'\n')
names={target:prior['expected_cases'][target] for target in ['hypercurve_bezier_split_materialization','hypercurve_bezier_region','hypercurve_bezier_arrangement']}
names={'hypercurve': [
 'bezier_split::finite_conic_split_regression::finite_conic_split_retains_zero_intermediate_homogeneous_weight',
 'bezier_split::finite_conic_split_regression::conic_homogeneous_cuts_retain_circle_and_tangent_evidence',
 'bezier_split::finite_conic_split_regression::conic_exterior_cuts_do_not_export_unit_weight_signs_across_poles',
 'bezier_moment::tests::rational_quadratic_first_moments_are_exactly_additive_under_subdivision',
 'bezier_moment::tests::rational_quadratic_quarter_circle_has_exact_green_first_moments',
 'bezier_moment::tests::rational_quadratic_linear_weight_denominator_keeps_geometric_line_moments',
 'bezier_moment::tests::rational_quadratic_area_cache_reuses_equal_weight_integrals',
 'rational_bezier_general::tests::homogeneous_representation_preserves_exact_split',
 'rational_bezier_general::tests::affine_subcurve_retains_zero_intermediate_weight_without_a_pole',
 'rational_bezier_general::tests::affine_subcurve_rejects_a_crossed_projective_pole',
 'arc_bezier::tests::one_arc_decomposition_shares_exact_conic_provenance',
 'arc_bezier::tests::mixed_weight_major_circle_is_recognized_without_a_projective_pole',
 'arc_bezier::tests::mixed_weight_conic_with_a_projective_pole_is_not_a_regular_arc',
],**names}
report=dict(normal_production_build=False,source_manifest=f'{prefix}-sources.json',source_directory=str(archive),builds=[],cases=[],checks=[],expected_cases=names,test_listings={},all_processes_reaped=False,probe_complete=False,qualification_complete=False)
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()));cargo='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
def verify():
 for name,sha in fixtures.items():assert hashlib.sha256((candidate/name).read_bytes()).hexdigest()==sha,name
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
 log=A/f'{prefix}-build.log';row=run([cargo,'test','--lib',*[arg for target in names if target!='hypercurve' for arg in ['--test',target]],'--release','--all-features','--locked','--offline','--no-run','--message-format=json'],build/'hypercurve',log,900);report['builds'].append(row);save()
 if row['returncode']:
  for line in log.read_text().splitlines():
   try:v=json.loads(line)
   except ValueError:continue
   if v.get('reason')=='compiler-message'and v['message']['level']=='error':print(v['message'].get('rendered','')[:3000],flush=True)
  raise RuntimeError('conic candidate build failed')
 print('Candidate compiled',round(row['elapsed_seconds'],3),flush=True)
 artifacts={}
 for line in log.read_text().splitlines():
  try:v=json.loads(line)
  except ValueError:continue
  target=v.get('target',{}).get('name')
  if v.get('reason')=='compiler-artifact'and target in names and v.get('executable'):artifacts[target]=v
 assert set(artifacts)==set(names)
 binaries={}
 for target,artifact in artifacts.items():
  binary=A/f'{prefix}-{target}-tests';shutil.copy2(artifact['executable'],binary);binaries[target]=dict(path=str(binary),sha256=hashlib.sha256(binary.read_bytes()).hexdigest())
 row['binaries']=dict(binaries);save()
 for target,selected in names.items():
  binary=binaries[target]['path'];listing_log=A/f'{prefix}-{target}-list.log';listing=run([binary,'--list'],archive/'hypercurve',listing_log,30);report['test_listings'][target]=listing;save();assert listing['returncode']==0
  available={line.removesuffix(': test')for line in listing_log.read_text().splitlines()if line.endswith(': test')};assert set(selected)<=available,(target,set(selected)-available)
 for target,selected in names.items():
  for name in selected:
   log=A/f'{prefix}-{name.split("::")[-1]}-{hashlib.sha256((target+name).encode()).hexdigest()[:10]}.log';case=run([binaries[target]['path'],'--exact',name,'--nocapture','--test-threads=1'],archive/'hypercurve',log,180);case.update(target=target,name=name,passed=case['returncode']==0 and 'test result: ok. 1 passed;'in log.read_text());report['cases'].append(case);save();print(name,case['returncode'],round(case['elapsed_seconds'],3),flush=True)
   if not case['passed']:
    print(log.read_text()[-2500:],flush=True);raise RuntimeError('focused regression failed: '+name)
 report['probe_complete']=True;code=0 if all(case['passed']for case in report['cases'])else 1
except Exception as error:code=1;report['failure']=str(error);print(str(error),flush=True)
finally:verify();report['all_processes_reaped']=True;save()
raise SystemExit(code)
