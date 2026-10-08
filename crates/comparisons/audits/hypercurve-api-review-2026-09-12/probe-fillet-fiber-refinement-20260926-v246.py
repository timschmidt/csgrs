from pathlib import Path
import hashlib,json,os,shutil,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent;prefix='fillet-fiber-refinement-20260926-v246';build=A/'build-workspace-20260925';archive=A/'source-archives'/prefix
prior=json.loads((A/'fillet-reoffset-compact-20260926-v243-terminal.json').read_text());manifest=json.loads((A/prior['source_manifest']).read_text());guard=json.loads((A/prior['workspace_guard']).read_text());assert not archive.exists()
for name,sha in manifest.items():
 src=Path(prior['source_directory'])/name;assert hashlib.sha256(src.read_bytes()).hexdigest()==sha
 dst=archive/name;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,dst)
 target=build/name
 if target.read_bytes()!=src.read_bytes():shutil.copy2(src,target)
 assert dst.stat().st_ino!=target.stat().st_ino
file='hypersolve/src/algebraic_fiber.rs';p=archive/file;t=p.read_text()
needle='        AlgebraicRootPolynomialEvaluationStatus::Undecided\n        | AlgebraicRootPolynomialEvaluationStatus::EvaluatedExactRationalWitness'
assert needle in t
t=t.replace(needle,'        AlgebraicRootPolynomialEvaluationStatus::Undecided => { eprintln!("local evaluation unavailable message={:?}", evaluation.message); Ok(None) },\n        AlgebraicRootPolynomialEvaluationStatus::EvaluatedExactRationalWitness',1)
needle='        let mut remainder = local_polynomial_remainder(previous, last, field)?;'
assert needle in t
t=t.replace(needle,'        eprintln!("sturm remainder previous={} divisor={}", previous.len(), last.len());\n'+needle,1)

start=t.index('    fn sign_polynomial_if_separated(');end=t.index('    fn is_zero_polynomial(',start);part=t[start:end]
needle2='        if let Some(sign) = sign {\n            self.signed_polynomials.push((polynomial.to_vec(), sign));'
assert needle2 in part
part=part.replace(needle2,r"""        if sign.is_none() && self.root.exact_point_witness().is_none() {
            match self.refine_root() {
                Ok(()) => {
                    let evaluation = evaluate_polynomial_at_algebraic_root(&self.root, polynomial, self.policy);
                    sign = local_evaluation_sign(&evaluation)?;
                }
                Err(LocalFieldError::Undecided) => {}
                Err(error) => return Err(error),
            }
        }
"""+needle2,1)
t=t[:start]+part+t[end:]

p.write_text(t);shutil.copy2(p,build/file);manifest[file]=hashlib.sha256(p.read_bytes()).hexdigest()
project=build/'fiber-probe';project.mkdir(exist_ok=True);(project/'src').mkdir(exist_ok=True)
(project/'Cargo.toml').write_text('''[package]
name = "fiber-probe"
version = "0.0.0"
edition = "2024"
[workspace]
[dependencies]
hyperreal = { path = "../hyperreal", features = ["serde", "dispatch-trace"] }
hyperlimit = { path = "../hyperlimit", default-features = false, features = ["std", "dispatch-trace"] }
hypersolve = { path = "../hypersolve", default-features = false }
''')
shutil.copy2(A/'fillet-fiber-replay-20260926-v244.rs',project/'src/main.rs')
fixture=A/'fillet-reoffset-compact-20260926-v243-fiber.jsonl'
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()));cargo='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
with(A/f'{prefix}-lock.log').open('w')as log:subprocess.run([cargo,'generate-lockfile','--offline'],cwd=project,env=env,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=120)
for rel in ['Cargo.toml','Cargo.lock','src/main.rs']:
 name='fiber-probe/'+rel;dst=archive/name;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(project/rel,dst);manifest[name]=hashlib.sha256(dst.read_bytes()).hexdigest()
(A/f'{prefix}-sources.json').write_text(json.dumps(manifest,indent=2)+'\n')
fixture_sha=hashlib.sha256(fixture.read_bytes()).hexdigest()
def verify():
 for name,sha in guard.items():assert hashlib.sha256((W/name).read_bytes()).hexdigest()==sha,name
 for name,sha in manifest.items():
  for root in [archive,build]:assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,name
 assert hashlib.sha256(fixture.read_bytes()).hexdigest()==fixture_sha
report=dict(source_manifest=f'{prefix}-sources.json',source_directory=str(archive),workspace_guard=prior['workspace_guard'],fixture=str(fixture),fixture_sha256=fixture_sha,all_processes_reaped=False)
verify();cmd=[cargo,'build','--release','--locked','--offline','--message-format=json'];report['command']=cmd;start=time.monotonic()
with(A/f'{prefix}-build.log').open('w')as log:code=subprocess.run(cmd,cwd=project,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=900).returncode
report['build_returncode']=code;verify()
if code==0:
 rows=[]
 for line in(A/f'{prefix}-build.log').read_text().splitlines():
  try:rows.append(json.loads(line))
  except ValueError:pass
 row=next(r for r in rows if r.get('reason')=='compiler-artifact'and r['target']['name']=='fiber-probe'and r.get('executable'))
 binary=A/prefix;shutil.copy2(row['executable'],binary);report['binary_sha256']=hashlib.sha256(binary.read_bytes()).hexdigest()
 with(A/f'{prefix}.log').open('w')as log:
  try:code=subprocess.run([str(binary),str(fixture)],stdout=log,stderr=subprocess.STDOUT,timeout=240).returncode
  except subprocess.TimeoutExpired:code=124
 report['run_returncode']=code;print((A/f'{prefix}.log').read_text()[-5000:],flush=True)
else:
 for line in(A/f'{prefix}-build.log').read_text().splitlines():
  try:r=json.loads(line)
  except ValueError:continue
  if r.get('reason')=='compiler-message'and r['message']['level']=='error':print(r['message'].get('rendered','')[:3000],flush=True)
verify();report['elapsed_seconds']=time.monotonic()-start;report['all_processes_reaped']=True;(A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n');raise SystemExit(code)
