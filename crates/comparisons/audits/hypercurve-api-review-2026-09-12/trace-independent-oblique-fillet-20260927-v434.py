from pathlib import Path
import hashlib,json,os,re,shutil,subprocess,time,signal
A=Path(__file__).resolve().parent;W=A.parent;prefix='trace-independent-oblique-fillet-20260927-v434';build=A/'build-workspace-20260925';archive=A/'source-archives'/prefix
prior=json.loads((A/'independent-oblique-fillet-20260927-v432-sources.json').read_text());manifest={};assert not archive.exists()
for name,old_sha in prior.items():
 src=W/name;data=src.read_bytes();sha=hashlib.sha256(data).hexdigest();assert sha==old_sha,name
 dst=archive/name;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,dst);target=build/name
 if target.read_bytes()!=data:shutil.copy2(src,target);os.utime(target,None)
 assert dst.stat().st_ino!=target.stat().st_ino;manifest[name]=sha

trial={}
for root in [archive,build]:
 p=root/'hypercurve/src/bezier_offset.rs';value=p.read_text()
 a=value.index('    fn tangent_linear_form_sign(');b=value.index('\n    fn tangent_relation_to_vector_sign(',a);part=value[a:b]
 target='        let exact_endpoint_sign = policy.strict_predicate_pass(|| -> CurveResult<_> {'
 assert target in part
 part=part.replace(target, r'''        let query_time = std::time::Instant::now();
        let kind = |point: &CurvePoint2| match point {
            CurvePoint2(CurvePointData2::Exact(_)) => "exact",
            CurvePoint2(CurvePointData2::Algebraic(_)) => "algebraic",
            CurvePoint2(CurvePointData2::AlgebraicChordPair(_)) => "pair",
            CurvePoint2(CurvePointData2::AlgebraicCuspChord(_)) => "cusp",
            CurvePoint2(CurvePointData2::AlgebraicCuspChordDerived(_)) => "derived",
            CurvePoint2(CurvePointData2::AlgebraicChordParallel(_)) => "parallel",
            CurvePoint2(CurvePointData2::AnalyticParallel(_)) => "analytic",
            CurvePoint2(CurvePointData2::Similarity(_) | CurvePointData2::Endpoint(_)) => "similarity",
        };
        eprintln!("linear begin kinds=({},{}) bounded={} approximate={}", kind(start_point), kind(end_point), policy.has_bounded_exact_predicate_budget(), policy.permits_approximate_512());
        let exact_endpoint_sign = policy.strict_predicate_pass(|| -> CurveResult<_> {''',1)
 target='        if let Some(Classification::Decided(sign)) = exact_endpoint_sign {'
 assert target in part
 part=part.replace(target, '        eprintln!("linear native ms={} status={:?}", query_time.elapsed().as_millis(), exact_endpoint_sign);\n'+target,1)
 target='            let (Classification::Decided(start), Classification::Decided(end)) = ('
 assert target in part
 part=part.replace(target, '            eprintln!("linear bounds begin steps={} ms={}", refinement_steps, query_time.elapsed().as_millis());\n'+target,1)
 target='            terminal_refined |= refinement_steps == 512;'
 part=part.replace(target, '            eprintln!("linear bounds ready steps={} ms={}", refinement_steps, query_time.elapsed().as_millis());\n'+target,1)
 target='        // Preserve every native interval decision before adjoining fields.'
 part=part.replace(target, '        eprintln!("linear replay begin ms={}", query_time.elapsed().as_millis());\n'+target,1)
 target='            let [start, end] = frame.direction_endpoints;'
 part=part.replace(target, '            eprintln!("linear field ready ms={}", query_time.elapsed().as_millis());\n'+target,1)
 target='                return Ok(Classification::Decided(if reversed {'
 part=part.replace(target, '                eprintln!("linear replay decided ms={} sign={:?}", query_time.elapsed().as_millis(), sign);\n'+target,1)
 target='        if terminal_refined && policy.permits_approximate_512() {'
 part=part.replace(target, '        eprintln!("linear replay unresolved ms={}", query_time.elapsed().as_millis());\n'+target,1)
 value=value[:a]+part+value[b:];p.write_text(value)
 p=root/'hypercurve/src/bezier_region.rs';value=p.read_text()
 a=value.index('    fn one_fragment_nonzero_parallel_loop_extends_chamfer_cuts_on_one_finite_envelope()');b=value.index('\n    #[test]',a);part=value[a:b]
 part=part.replace('                let seam = p(0, 0);', '                let query_time = std::time::Instant::now();\n                eprintln!("chamfer begin reversed={} approximate={}", reversed, policy.permits_approximate_512());\n                let seam = p(0, 0);',1)
 part=part.replace('                let trimmed = region', '                eprintln!("chamfer trim begin ms={}", query_time.elapsed().as_millis());\n                let trimmed = region',1)
 part=part.replace('                let extended_work = || {', '                eprintln!("chamfer trim complete ms={}", query_time.elapsed().as_millis());\n                let extended_work = || {',1)
 part=part.replace('                let extended = extended.unwrap();', '                eprintln!("chamfer extend complete ms={}", query_time.elapsed().as_millis());\n                let extended = extended.unwrap();',1)
 value=value[:a]+part+value[b:];p.write_text(value)
for name in ['hypercurve/src/bezier_offset.rs','hypercurve/src/bezier_region.rs']:
 trial[name]=hashlib.sha256((archive/name).read_bytes()).hexdigest()

(A/f'{prefix}-sources.json').write_text(json.dumps(manifest,indent=2)+'\n')
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()));cargo='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
report=dict(source_manifest=f'{prefix}-sources.json',source_directory=str(archive),cases=[],builds=[],trial=trial,all_processes_reaped=False)
def verify():
 for name,sha in manifest.items():
  for root in [W,archive,build]:assert hashlib.sha256((root/name).read_bytes()).hexdigest()==(sha if root==W else trial.get(name,sha)),name

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
 expected={'hypercurve':['bezier_region::tests::nonlinear_selected_corner_chamfers_through_retained_fixed_distance_image','bezier_region::tests::one_fragment_nonzero_parallel_loop_extends_chamfer_cuts_on_one_finite_envelope']}
 report['expected_cases']=expected;report['known_unresolved']=json.loads((A/'point-similarity-composition-20260927-v410-terminal.json').read_text())['known_unresolved'];save()
 binary=compile_binary('hypercurve','hypercurve',['--lib'])
 for name in expected['hypercurve']:case(binary,name)
except Exception as error:
 code=1;report['failure']=str(error)
finally:
 verify();report['all_processes_reaped']=True;report['qualification_complete']=code==0;save();print('focused_qualification_complete',code==0,flush=True)
raise SystemExit(code)
