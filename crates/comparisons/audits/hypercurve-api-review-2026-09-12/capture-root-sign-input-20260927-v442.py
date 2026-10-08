from pathlib import Path
import hashlib,json,os,re,shutil,subprocess,time,signal
A=Path(__file__).resolve().parent;W=A.parent;prefix='root-sign-input-20260927-v442';build=A/'build-workspace-20260925';archive=A/'source-archives'/prefix
prior=json.loads((A/'independent-oblique-fillet-20260927-v436-sources.json').read_text());manifest={};assert not archive.exists()
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
 p=root/'hypersolve/src/root_sign.rs';value=p.read_text()
 a=value.index('pub fn sign_at_selected_root(');b=value.index('\nfn chain_sign(',a);part=value[a:b]
 target='    if defining.len() < 2';assert target in part
 insertion=r'''    if predicate.len() == 6 && let Some(path) = std::env::var_os("HYPERSOLVE_CAPTURE_ROOT_QUERY") {
        static QUERY: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
        let index = QUERY.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        let encode = |value: &Real| match value.capture_root_query_tower_parts() {
            Some(parts) => {
                let rows = parts.map(|[a,b,d]| format!("[\"{a}\",\"{b}\",\"{d}\"]"));
                format!("[{}]", rows.join(","))
            },
            None => "null".to_owned(),
        };
        let polynomial = |values: &[Real]| values.iter().map(&encode).collect::<Vec<_>>().join(",");
        let data = format!("{{\"defining\":[{}],\"predicate\":[{}],\"interval\":[{},{}],\"distinct_root_count\":{}}}", polynomial(defining), polynomial(predicate), encode(&interval.lower), encode(&interval.upper), interval.distinct_root_count);
        let file = std::path::Path::new(&path).join(format!("query-{index:03}.json"));
        std::fs::write(file, data).expect("write exact selected-root query");
        eprintln!("captured selected root query index={index} defining={} predicate={}", defining.len(), predicate.len());
        if index == 2 { panic!("CAPTURED_SELECTED_ROOT_QUERY"); }
    }
'''
 part=part.replace(target,insertion+target,1)
 p.write_text(value[:a]+part+value[b:])
for name in ['hypersolve/src/root_sign.rs','hyperreal/src/real/arithmetic/quadratic_tower_sign.rs']:
 trial[name]=hashlib.sha256((archive/name).read_bytes()).hexdigest()

(A/f'{prefix}-sources.json').write_text(json.dumps(manifest,indent=2)+'\n')
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()));cargo='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
capture=A/f'{prefix}-queries';capture.mkdir();env['HYPERSOLVE_CAPTURE_ROOT_QUERY']=str(capture)
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
 if code!=101 or 'CAPTURED_SELECTED_ROOT_QUERY' not in log.read_text() or not re.search(r'test result: FAILED\. 0 passed; 1 failed;',log.read_text()):print(log.read_text()[-3000:],flush=True);raise RuntimeError('case failure or missing exact test')
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
verify();code=0;report['checks']=[];report['qualification_scope']='Diagnostic capture of exact selected-root inputs; intentional test stop, not a test pass'
try:
 expected={'hypercurve_curve_region_promotion':['strict_trim_or_extend_analytic_parallel_support_corners_retain_algebraic_fillet_extensions']}
 report['expected_cases']=expected;report['known_unresolved']=json.loads((A/'point-similarity-composition-20260927-v410-terminal.json').read_text())['known_unresolved'];save()
 binary=compile_binary('hypercurve','hypercurve_curve_region_promotion',['--test','hypercurve_curve_region_promotion'])
 for name in expected['hypercurve_curve_region_promotion']:case(binary,name)
except Exception as error:
 code=1;report['failure']=str(error)
finally:
 verify();report['all_processes_reaped']=True;report['capture_complete']=code==0;report['qualification_complete']=False;save();print('capture_complete',code==0,flush=True)
raise SystemExit(code)
