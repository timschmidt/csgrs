from pathlib import Path
import hashlib,json,os,re,shutil,subprocess,time,signal
A=Path(__file__).resolve().parent;W=A.parent;prefix='root-sign-reduced-query-20260927-v450';build=A/'build-workspace-20260925';archive=A/'source-archives'/prefix
prior=json.loads((A/'independent-oblique-fillet-20260927-v436-sources.json').read_text());manifest={};assert not archive.exists()
for name,old_sha in prior.items():
 src=W/name;data=src.read_bytes();sha=hashlib.sha256(data).hexdigest();assert sha==old_sha,name
 dst=archive/name;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,dst);target=build/name
 if target.read_bytes()!=data:shutil.copy2(src,target);os.utime(target,None)
 assert dst.stat().st_ino!=target.stat().st_ino;manifest[name]=sha

from fractions import Fraction
capture=A/'root-sign-input-20260927-v442-queries'
oracle=json.loads((A/'root-sign-field-20260927-v444-terminal.json').read_text());assert oracle['complete']
def canonical(value):
 if ' ' in value:
  whole,fraction=value.lstrip('-+').split();rational=(Fraction(whole)+Fraction(fraction))*(-1 if value.startswith('-') else 1)
 else:rational=Fraction(value)
 return str(rational)
def canonical_parts(parts):return [[canonical(value) for value in row] for row in parts]
module=r'''
#[cfg(test)]
mod captured_quadratic_field_queries {
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
'''
for index,row in enumerate(oracle['cases']):
 p=capture/row['input'];assert hashlib.sha256(p.read_bytes()).hexdigest()==row['sha256'];data=json.loads(p.read_text())
 parts={key:json.dumps([canonical_parts(value) for value in data[key]])for key in ['defining','predicate','interval']}
 expected={-1:'Less',0:'Equal',1:'Greater'}[row['sign']]
 module+=f'''
    #[test]
    fn selected_root_query_{index:03}() {{
        let defining={parts['defining']}.map(decode);
        let predicate={parts['predicate']}.map(decode);
        let [lower,upper]={parts['interval']}.map(decode);
        let interval=IsolatedRootInterval{{lower,upper,exact_root:None,distinct_root_count:1}};
        let started=std::time::Instant::now();
        let result=sign_at_selected_root(&defining,&predicate,&interval);
        eprintln!("captured root query {index:03} elapsed_ms={{}} result={{result:?}}",started.elapsed().as_millis());
        assert_eq!(result,Some(Ordering::{expected}));
    }}
'''
module+='}\n'
trial={}
for root in [archive,build]:
 p=root/'hypersolve/src/root_sign.rs';p.write_text(p.read_text()+module)
trial['hypersolve/src/root_sign.rs']=hashlib.sha256((archive/'hypersolve/src/root_sign.rs').read_bytes()).hexdigest()
for root in [archive,build]:
 p=root/'hyperreal/src/computable/node/quadratic_tower.rs';value=p.read_text()
 old='''        let cross = &self.rational * &other.scale + &self.scale * &other.rational;
        let mut rational = &self.rational * &other.rational;
        if self.scale.sign() != Sign::NoSign && other.scale.sign() != Sign::NoSign {
            let disc = disc.as_ref()?;
            rational = rational + &self.scale * &other.scale * disc;
        }
'''
 new='''        let cross = Rational::signed_product_sum2(
            [true, true],
            [[&self.rational, &other.scale], [&self.scale, &other.rational]],
        );
        let rational = if self.scale.sign() != Sign::NoSign && other.scale.sign() != Sign::NoSign {
            let one = Rational::one();
            Rational::signed_product_sum(
                [true, true],
                [[&self.rational, &other.rational, &one], [&self.scale, &other.scale, disc.as_ref()?]],
            )
        } else {
            &self.rational * &other.rational
        };
'''
 assert value.count(old)==1;value=value.replace(old,new,1)
 old='        let norm = &self.rational * &self.rational - &self.scale * &self.scale * disc;'
 new='''        let one = Rational::one();
        let norm = Rational::signed_product_sum(
            [true, false],
            [[&self.rational, &self.rational, &one], [&self.scale, &self.scale, disc]],
        );'''
 assert value.count(old)==1;value=value.replace(old,new,1)
 a=value.index('    fn sign(&self) -> Option<RealSign> {');b=value.index('\n}\n\nimpl Tower {',a)
 replacement='''    fn sign(&self) -> Option<RealSign> {
        if self.scale.sign() == Sign::NoSign {
            return Some(rational_sign(&self.rational));
        }
        let disc = self.disc.as_ref()?;
        if disc.sign() != Sign::Plus {
            return None;
        }
        if self.rational.sign() == Sign::NoSign || self.rational.sign() == self.scale.sign() {
            return Some(rational_sign(&self.scale));
        }
        let one = Rational::one();
        Some(match Rational::signed_product_sum_ordering(
            [true, false],
            [[&self.rational, &self.rational, &one], [&self.scale, &self.scale, disc]],
        ) {
            core::cmp::Ordering::Greater => rational_sign(&self.rational),
            core::cmp::Ordering::Less => rational_sign(&self.scale),
            core::cmp::Ordering::Equal => RealSign::Zero,
        })
    }'''
 value=value[:a]+replacement+value[b:];p.write_text(value)
trial['hyperreal/src/computable/node/quadratic_tower.rs']=hashlib.sha256((archive/'hyperreal/src/computable/node/quadratic_tower.rs').read_bytes()).hexdigest()



for root in [archive,build]:
 p=root/'hypersolve/src/root_sign.rs';value=p.read_text()
 a=value.index('fn field_signed_sequence(');b=value.index('\nfn variations(',a);part=value[a:b]
 part=part.replace('    let mut chain = vec![first.clone()];','    first = compact_exact_coefficients(first);\n    let mut chain = vec![first.clone()];',1)
 part=part.replace('    loop {','    loop {\n        second = compact_exact_coefficients(second);',1)
 part=part.replace('        sign(second.last()?)?;','''        let leading_sign = sign(second.last()?)?;
        second = crate::root_isolation::gcd_monic_normalize(second, PredicatePolicy::STRICT)?;
        if leading_sign == Ordering::Less {
            for coefficient in &mut second { *coefficient = -coefficient.clone(); }
        }
        second = compact_exact_coefficients(second);''',1)
 p.write_text(value[:a]+part+value[b:])
 p=root/'hypersolve/src/root_isolation.rs';value=p.read_text();old='fn gcd_monic_normalize(polynomial:';assert value.count(old)==1;p.write_text(value.replace(old,'pub(crate) fn gcd_monic_normalize(polynomial:',1))
for name in ['hypersolve/src/root_sign.rs','hypersolve/src/root_isolation.rs']:
 trial[name]=hashlib.sha256((archive/name).read_bytes()).hexdigest()


for root in [archive,build]:
 p=root/'hypersolve/src/root_sign.rs';value=p.read_text()
 anchor='    first = compact_exact_coefficients(first);\n    let mut chain = vec![first.clone()];'
 replacement='''    first = compact_exact_coefficients(first);
    if second.len() >= first.len() {
        second = polynomial_div_rem(second, &first, PredicatePolicy::STRICT)?.1;
    }
    let mut chain = vec![first.clone()];'''
 assert value.count(anchor)==1;p.write_text(value.replace(anchor,replacement,1))
trial['hypersolve/src/root_sign.rs']=hashlib.sha256((archive/'hypersolve/src/root_sign.rs').read_bytes()).hexdigest()

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
verify();code=0;report['checks']=[];report['qualification_scope']='Standalone captured quartic-field root-sign queries, using reduced query numerators, positive leading units, and fused scalar arithmetic with independently proved signs'
try:
 expected={'hypersolve':['root_sign::captured_quadratic_field_queries::selected_root_query_002','root_sign::captured_quadratic_field_queries::selected_root_query_000','root_sign::captured_quadratic_field_queries::selected_root_query_001']}
 report['expected_cases']=expected;report['known_unresolved']=json.loads((A/'point-similarity-composition-20260927-v410-terminal.json').read_text())['known_unresolved'];save()
 binary=compile_binary('hypersolve','hypersolve',['--lib'])
 for name in expected['hypersolve']:case(binary,name)
except Exception as error:
 code=1;report['failure']=str(error)
finally:
 verify();report['all_processes_reaped']=True;report['qualification_complete']=code==0;save();print('focused_qualification_complete',code==0,flush=True)
raise SystemExit(code)
