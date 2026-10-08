from pathlib import Path
import hashlib,json,os,re,shutil,subprocess,time,signal
A=Path(__file__).resolve().parent;W=A.parent;prefix='independent-oblique-fillet-20260927-v426';build=A/'build-workspace-20260925';archive=A/'source-archives'/prefix
prior=json.loads((A/'point-similarity-composition-20260927-v410-sources.json').read_text());manifest={};assert not archive.exists()
for name,old_sha in prior.items():
 src=W/name;data=src.read_bytes();sha=hashlib.sha256(data).hexdigest();assert sha==old_sha or name in ['hypercurve/src/bezier_offset.rs','hypercurve/src/curve.rs','hypercurve/src/curve_fillet.rs','hypercurve/src/curve_region_boolean.rs','hypercurve/src/curve_support_intersection.rs'],name
 dst=archive/name;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,dst);target=build/name
 if target.read_bytes()!=data:shutil.copy2(src,target);os.utime(target,None)
 assert dst.stat().st_ino!=target.stat().st_ino;manifest[name]=sha

(A/f'{prefix}-sources.json').write_text(json.dumps(manifest,indent=2)+'\n')
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()));cargo='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
report=dict(source_manifest=f'{prefix}-sources.json',source_directory=str(archive),cases=[],builds=[],all_processes_reaped=False)
def verify():
 for name,sha in manifest.items():
  for root in [W,archive,build]:assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,name

def save():
 (A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
def run_owned(cmd, *, cwd, env, stdout, stderr, timeout):
 process=subprocess.Popen(cmd,cwd=cwd,env=env,stdout=stdout,stderr=stderr,start_new_session=True)
 try:return process.wait(timeout=timeout)
 except BaseException:
  os.killpg(process.pid,signal.SIGKILL);process.wait();raise

def compile_binary(crate,target,selector):
 cmd=[cargo,'test',*selector,'--release','--all-features','--no-run','--message-format=json','--locked','--offline'];log=A/f'{prefix}-{target}-build.log';start=time.monotonic()
 with log.open('w')as out:code=run_owned(cmd,cwd=build/crate,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=900)
 record=dict(crate=crate,command=cmd,returncode=code,log=log.name,elapsed_seconds=time.monotonic()-start);report['builds'].append(record);save()
 if code:
  for line in log.read_text().splitlines():
   try:r=json.loads(line)
   except ValueError:continue
   if r.get('reason')=='compiler-message'and r['message']['level']=='error':print(r['message'].get('rendered','')[:3000],flush=True)
  raise RuntimeError('build failure')
 rows=[]
 for line in log.read_text().splitlines():
  try:rows.append(json.loads(line))
  except ValueError:pass
 row=next(r for r in rows if r.get('reason')=='compiler-artifact'and r['target']['name']==target and r.get('executable'))
 binary=A/f'{prefix}-{target}';shutil.copy2(row['executable'],binary);record['binary']=dict(path=str(binary),sha256=hashlib.sha256(binary.read_bytes()).hexdigest());save();return binary

def case(binary,name):
 log=A/f'{prefix}-case-{len(report["cases"]):03d}.log';start=time.monotonic()
 with log.open('w')as out:
  try:code=run_owned([str(binary),'--exact',name,'--nocapture','--test-threads=1'],cwd=archive,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=240)
  except subprocess.TimeoutExpired:code=124
 report['cases'].append(dict(binary=binary.name,name=name,returncode=code,log=log.name,elapsed_seconds=time.monotonic()-start));save();print(name,code,round(time.monotonic()-start,2),flush=True)
 if code or not re.search(r'test result: ok\. 1 passed;',log.read_text()):print(log.read_text()[-3000:],flush=True);raise RuntimeError('case failure or missing exact test')
def run_record(label,cmd,cwd,timeout=900):
 log=A/f'{prefix}-{label}.log';start=time.monotonic()
 with log.open('w')as out:
  try:code=run_owned(cmd,cwd=cwd,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=timeout)
  except subprocess.TimeoutExpired:code=124
 record=dict(label=label,command=cmd,returncode=code,log=log.name,elapsed_seconds=time.monotonic()-start);report['checks'].append(record);save();print(label,code,round(record['elapsed_seconds'],2),flush=True)
 if code:
  with log.open()as f:f.seek(max(0,log.stat().st_size-5000));print(f.read()[-3000:],flush=True)
  raise RuntimeError(label+' failed')
 return log,record
verify();code=0;report['checks']=[];report['qualification_scope']='Independent selected endpoint fields must preserve continuous fillet constraint behavior'
try:
 expected={'hypercurve':['bezier_offset::conversion_tests::independent_oblique_offset_coordinates_replay_exact_equalities_and_separation','bezier_offset::conversion_tests::independent_chord_linear_forms_replay_exact_coefficient_relations','bezier_offset::conversion_tests::independent_oblique_chords_support_constrained_fillet_families']}
 expected['hypercurve'] += ['curve::curve_fillet::tests::nonlinear_linear_fillet_components_retain_unique_contacts_and_tangents','curve::curve_fillet::tests::coincident_retained_line_fillet_keeps_independent_endpoint_fields','bezier_offset::conversion_tests::recursive_projective_chord_replays_identically_zero_norm_sheet','bezier_offset::conversion_tests::regular_rational_parallel_components_select_exterior_speed_sheets']
 expected['hypercurve'] += ['bezier_offset::conversion_tests::domain_component_normal_constraints_follow_swapped_operands', 'bezier_offset::conversion_tests::original_parallel_normal_constraints_clip_finite_and_incident_components', 'bezier_offset::conversion_tests::finite_parallel_pair_components_obey_exact_domain_boundaries', 'bezier_offset::conversion_tests::mixed_zero_parallel_pairs_replay_both_exterior_domain_roles', 'bezier_offset::conversion_tests::ordered_ph_parallel_domains_retain_same_sheet_exterior_contacts', 'bezier_offset::conversion_tests::ordered_ph_parallel_domains_reject_a_cached_opposite_normal_contact', 'bezier_offset::conversion_tests::ordered_ph_parallel_domains_do_not_reuse_the_wrong_speed_sheet', 'bezier_offset::conversion_tests::nonzero_parallel_domains_exclude_undefined_normals_and_poles', 'bezier_offset::conversion_tests::nonzero_parallel_domains_partition_poles_on_components', 'bezier_offset::conversion_tests::finite_component_charts_accept_nonrational_exact_scalar_bounds', 'bezier_offset::conversion_tests::non_source_parallel_overlap_replays_the_radical_component']
 expected['hypercurve'] += ['bezier_offset::conversion_tests::split_chord_linear_tangents_reuse_the_oriented_support','bezier_offset::conversion_tests::nested_procedural_chord_reversals_match_exact_side_and_winding']
 report['expected_cases']=expected;report['known_unresolved']=json.loads((A/'point-similarity-composition-20260927-v410-terminal.json').read_text())['known_unresolved'];save()
 binary=compile_binary('hypercurve','hypercurve',['--lib'])
 for name in expected['hypercurve']:case(binary,name)
except Exception as error:
 code=1;report['failure']=str(error)
finally:
 verify();report['all_processes_reaped']=True;report['qualification_complete']=code==0;save();print('focused_qualification_complete',code==0,flush=True)
raise SystemExit(code)
