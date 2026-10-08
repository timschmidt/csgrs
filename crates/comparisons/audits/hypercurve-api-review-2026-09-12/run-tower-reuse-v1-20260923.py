from pathlib import Path
import hashlib,json,os,shutil,subprocess,time

A=Path(__file__).resolve().parent
root=Path('/tmp/hypercurve-tower-reuse-v1-2026-09-23'); repo=root/'hyperreal'
prefix='tower-reuse-20260923-v1'
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
bindings=json.loads((A/f'{prefix}-sources.json').read_text())
def verify():
    for name,sha in bindings.items():assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,name
verify();os.utime(repo/'src/lib.rs',None)
cargo='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
report=dict(checks=[],tests=[],all_processes_reaped=False)
def save():
    verify();report['all_sources_unchanged']=True
    (A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
for feature in ['--all-features','--no-default-features']:
    log=A/f'{prefix}-check-{len(report["checks"])}.log'
    with log.open('w') as out:code=subprocess.run([cargo,'check','--all-targets',feature,'--locked','--offline'],cwd=repo,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=1200).returncode
    report['checks'].append(dict(feature=feature,returncode=code,log=log.name));verify()
    if code!=0 or 'warning:' in log.read_text():
        report['all_processes_reaped']=True;save();print(log.read_text()[-4000:],flush=True);raise SystemExit(1)
    print('Feature check passed:',feature,flush=True)
with (A/f'{prefix}-build.jsonl').open('w') as out,(A/f'{prefix}-build.log').open('w') as err:
    code=subprocess.run([cargo,'test','--lib','--release','--all-features','--no-run','--message-format=json','--locked','--offline'],cwd=repo,env=env,stdout=out,stderr=err,timeout=1200).returncode
verify();assert code==0
artifacts=[json.loads(line) for line in (A/f'{prefix}-build.jsonl').read_text().splitlines()]
tests=[r for r in artifacts if r.get('reason')=='compiler-artifact' and r.get('executable') and r['target']['name']=='hyperreal']
assert len(tests)==1 and not tests[0]['fresh']
binary=A/f'{prefix}-libtest';shutil.copy2(tests[0]['executable'],binary);report['binary_sha256']=hashlib.sha256(binary.read_bytes()).hexdigest()
for label,args,limit in [('focused',['quadratic_tower','--nocapture','--test-threads=1'],60),('all',['--test-threads=2'],180)]:
    log=A/f'{prefix}-{label}.log';start=time.monotonic()
    with log.open('w') as out:
        try: code=subprocess.run([str(binary),*args,'--color','never'],cwd=repo,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=limit).returncode
        except subprocess.TimeoutExpired:code='timeout'
    verify();report['tests'].append(dict(label=label,returncode=code,elapsed_seconds=time.monotonic()-start,log=log.name));print(label,code,log.read_text()[-2500:],flush=True)
    if code!=0:break
report['all_processes_reaped']=True;save()
