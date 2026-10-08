from pathlib import Path
import hashlib,json,os,re,shutil,subprocess,time,signal
A=Path(__file__).resolve().parent;W=A.parent;prefix='restricted-fiber-final-oracle-20260927-v475';build=A/'build-workspace-20260925';archive=A/'source-archives'/prefix
qualified=json.loads((A/'restricted-fiber-sign-20260927-v474-terminal.json').read_text());assert qualified['qualification_complete'] and qualified['all_processes_reaped']
prior=json.loads((A/qualified['source_manifest']).read_text());manifest={};assert not archive.exists()
for name,old_sha in prior.items():
 src=W/name;data=src.read_bytes();sha=hashlib.sha256(data).hexdigest();assert sha==old_sha,name
 dst=archive/name;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,dst);target=build/name
 if target.read_bytes()!=data:shutil.copy2(src,target);os.utime(target,None)
 assert dst.stat().st_ino!=target.stat().st_ino;manifest[name]=sha


from fractions import Fraction
capture=A/'selected-fiber-input-20260927-v466-queries/query-000.json'
data=json.loads(capture.read_text())
oracle=json.loads((A/'selected-fiber-box-20260927-v471-terminal.json').read_text())
assert oracle['complete'] and oracle['common_root_excluded'] and hashlib.sha256(capture.read_bytes()).hexdigest()==oracle['input_sha256']
def canonical(value):
 if ' ' in value:
  whole,fraction=value.lstrip('-+').split();v=(Fraction(whole)+Fraction(fraction))*(-1 if value.startswith('-')else 1)
 else:v=Fraction(value)
 return str(v)
def parts(v):return [[canonical(x)for x in row]for row in v]
module=r"""
#[cfg(test)]
mod captured_fiber_interval_query {
    use super::*;
    fn decode(parts: [[&str; 3]; 3]) -> Real {
        let quadratic = |[a,b,d]: [&str;3]| {
            let constant=Real::new(a.parse::<hyperreal::Rational>().unwrap());
            if b=="0" { constant } else {
                constant+Real::new(b.parse::<hyperreal::Rational>().unwrap())
                    *Real::new(d.parse::<hyperreal::Rational>().unwrap()).sqrt().unwrap()
            }
        };
        let even=quadratic(parts[0]);
        if parts[1][0]=="0" && parts[1][1]=="0" {even} else {
            even+quadratic(parts[1])*quadratic(parts[2]).sqrt().unwrap()
        }
    }
    #[test]
    fn exact_restricted_fiber_box_excludes_captured_common_root() {
"""
rows=','.join('vec!'+json.dumps([parts(c)for c in row])+'.into_iter().map(decode).collect::<Vec<_>>()'for row in data['polynomials'][1])
module+='let predicate=BivariatePolynomial::new(vec!['+rows+']);\n'
module+='let [lower,upper]='+json.dumps([parts(v)for v in data['root_interval']])+'.map(decode);\nlet first=RealInterval{lower,upper};\n'
module+='let [lower,upper]='+json.dumps([parts(v)for v in data['fiber_interval']])+'.map(decode);\nlet second=RealInterval{lower,upper};\n'
module+=r"""
        let started=std::time::Instant::now();
        let restricted=predicate.substitute_affine(&(&first.upper-&first.lower),&first.lower,&(&second.upper-&second.lower),&second.lower);
        let unit=RealInterval{lower:Real::zero(),upper:Real::one()};
        let sign=RealInterval::evaluate_bivariate_power_basis(&restricted,&unit,&unit)
            .and_then(|value|value.strict_nonzero_sign());
        eprintln!("restricted fiber box sign={sign:?} elapsed_ms={}",started.elapsed().as_millis());
        assert_eq!(sign,Some(RealSign::Positive));
    }
}
"""
trial={}
for root in [archive,build]:
 p=root/'hypercurve/src/bezier_offset.rs';p.write_text(p.read_text()+module)
trial['hypercurve/src/bezier_offset.rs']=hashlib.sha256((archive/'hypercurve/src/bezier_offset.rs').read_bytes()).hexdigest()

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
  try:code=run_owned([str(binary),'--exact',name,'--nocapture','--test-threads=1'],cwd=archive,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=90)
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
verify();code=0;report['checks']=[];report['qualification_scope']='Independent captured fiber-box sign appended to the final qualified production source; production implementation unchanged'
try:
 expected={'hypercurve':['bezier_offset::captured_fiber_interval_query::exact_restricted_fiber_box_excludes_captured_common_root']}
 report['expected_cases']=expected;save()
 binary=compile_binary('hypercurve','hypercurve',['--lib'])
 for name in expected['hypercurve']:case(binary,name)
except Exception as error:
 code=1;report['failure']=str(error)
finally:
 verify();report['all_processes_reaped']=True;report['qualification_complete']=code==0;save();print('focused_qualification_complete',code==0,flush=True)
raise SystemExit(code)
