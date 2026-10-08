from pathlib import Path
import hashlib,json,os,re,shutil,subprocess,time,signal
A=Path(__file__).resolve().parent;W=A.parent;prefix='selected-fiber-input-20260927-v466';build=A/'build-workspace-20260925';archive=A/'source-archives'/prefix
qualified=json.loads((A/'rational-root-dispatch-20260927-v468-terminal.json').read_text());assert qualified['qualification_complete'] and qualified['all_processes_reaped']
prior=json.loads((A/qualified['source_manifest']).read_text());manifest={};assert not archive.exists()
for name,old_sha in prior.items():
 src=W/name;data=src.read_bytes();sha=hashlib.sha256(data).hexdigest();assert sha==old_sha,name
 dst=archive/name;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,dst);target=build/name
 if target.read_bytes()!=data:shutil.copy2(src,target);os.utime(target,None)
 assert dst.stat().st_ino!=target.stat().st_ino;manifest[name]=sha


trial={}
for root in [archive,build]:
 p=root/'hyperreal/src/real/arithmetic/quadratic_tower_sign.rs';value=p.read_text()
 insertion=r'''impl Real {
    /// Diagnostic-only export of the retained exact tower payload.
    pub fn capture_root_query_tower_parts(&self) -> Option<[[Rational; 3]; 3]> {
        if let Some(rational) = self.exact_rational() {
            let mut parts = std::array::from_fn(|_| std::array::from_fn(|_| Rational::zero()));
            parts[0][0] = rational;
            return Some(parts);
        }
        let mut parts = self.computable_ref().quadratic_tower_parts()?;
        for row in &mut parts[..2] {
            row[0] = &row[0] * &self.rational;
            row[1] = &row[1] * &self.rational;
        }
        Some(parts)
    }
'''
 assert value.count('impl Real {')==1;value=value.replace('impl Real {',insertion,1);p.write_text(value)

 p=root/'hypersolve/src/algebraic_fiber.rs';value=p.read_text()
 marker='fn count_bivariate_fiber_system_roots('
 assert value.count(marker)==1
 helpers=r'''
static FIBER_CAPTURE_QUERY: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
struct FiberCaptureTimer { index: usize, started: std::time::Instant }
impl Drop for FiberCaptureTimer {
    fn drop(&mut self) { eprintln!("fiber-query end index={} elapsed_ms={}", self.index, self.started.elapsed().as_millis()); }
}
'''
 value=value.replace(marker,helpers+marker,1)
 a=value.index(marker);b=value.index('\nfn fiber_root_count_outcome_report(',a);part=value[a:b]
 marker='    let mut field = match LocalAlgebraicField::new(retained_root, policy) {'
 assert part.count(marker)==1
 capture=r'''    let capture_index = FIBER_CAPTURE_QUERY.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    let _timer=FiberCaptureTimer {index:capture_index,started:std::time::Instant::now()};
    eprintln!("fiber-query begin index={capture_index} polynomials={} root_coefficients={}",polynomials.len(),retained_root.polynomial_coefficients.len());
    if polynomials.len()==2 && let Some(path)=std::env::var_os("HYPERSOLVE_CAPTURE_FIBER_QUERY") {
        let encode = |value: &Real| match value.capture_root_query_tower_parts() {
            Some(parts) => {
                let rows = parts.map(|[a,b,d]| format!("[\"{a}\",\"{b}\",\"{d}\"]"));
                format!("[{}]", rows.join(","))
            },
            None => "null".to_owned(),
        };
        let polynomial = |values: &[Real]| format!("[{}]",values.iter().map(&encode).collect::<Vec<_>>().join(","));
        let bivariates = polynomials.iter().map(|value|format!("[{}]",value.coefficients.iter().map(|row|polynomial(row)).collect::<Vec<_>>().join(","))).collect::<Vec<_>>().join(",");
        let parameter = match retained_parameter {CurveResultantParameter::First=>"first",CurveResultantParameter::Second=>"second"};
        let endpoints = if endpoints==FiberIntervalEndpoints::RejectRoots {"reject"} else {"include"};
        let data=format!("{{\"root_polynomial\":{},\"root_interval\":[{},{}],\"root_count\":{},\"polynomials\":[{}],\"fiber_interval\":[{},{}],\"retained_parameter\":\"{parameter}\",\"endpoints\":\"{endpoints}\"}}",polynomial(&retained_root.polynomial_coefficients),encode(&retained_root.interval.lower),encode(&retained_root.interval.upper),retained_root.interval.distinct_root_count,bivariates,encode(fiber_lower),encode(fiber_upper));
        std::fs::write(std::path::Path::new(&path).join(format!("query-{capture_index:03}.json")),data).expect("write exact fiber query");
        eprintln!("captured fiber query index={capture_index}");
    }
'''
 part=part.replace(marker,capture+marker,1)
 ending='    fiber_root_count_outcome_report(outcome, &field)'
 assert part.count(ending)==1
 part=part.replace(ending,"    let report=fiber_root_count_outcome_report(outcome, &field);\n    eprintln!(\"fiber-query result index={capture_index} status={:?} count={:?} certainty={:?}\",report.status,report.distinct_root_count,report.certainty);\n    report")
 p.write_text(value[:a]+part+value[b:])
for name in ['hyperreal/src/real/arithmetic/quadratic_tower_sign.rs','hypersolve/src/algebraic_fiber.rs']:
 trial[name]=hashlib.sha256((archive/name).read_bytes()).hexdigest()

(A/f'{prefix}-sources.json').write_text(json.dumps(manifest,indent=2)+'\n')
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()));cargo='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
capture=A/f'{prefix}-queries';capture.mkdir();env['HYPERSOLVE_CAPTURE_FIBER_QUERY']=str(capture)
report=dict(diagnostic_only=True,source_manifest=f'{prefix}-sources.json',source_directory=str(archive),cases=[],builds=[],trial=trial,all_processes_reaped=False)
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
 if code not in [0,124] or not list(capture.glob('query-*.json')):print(log.read_text()[-3000:],flush=True);raise RuntimeError('case failure or missing exact test')
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
verify();code=0;report['checks']=[];report['qualification_scope']='Exact common-fiber input capture at the later local-field GCD; a timed diagnostic stop is not a passing test'
try:
 expected={'hypercurve_curve_region_promotion':['strict_trim_or_extend_analytic_parallel_support_corners_retain_algebraic_fillet_extensions']}
 report['expected_cases']=expected;save()
 binary=compile_binary('hypercurve','hypercurve_curve_region_promotion',['--test','hypercurve_curve_region_promotion'])
 for name in expected['hypercurve_curve_region_promotion']:case(binary,name)

except Exception as error:
 code=1;report['failure']=str(error)
finally:
 verify();report['all_processes_reaped']=True;report['capture_complete']=code==0;report['qualification_complete']=False;report['captures']=[dict(file=p.name,sha256=hashlib.sha256(p.read_bytes()).hexdigest())for p in sorted(capture.glob('query-*.json'))];save();print('focused_qualification_complete',code==0,flush=True)
raise SystemExit(code)
