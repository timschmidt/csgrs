from pathlib import Path
import hashlib,json,os,shutil,subprocess,time

A=Path(__file__).resolve().parent
root=Path('/tmp/hypercurve-tower-reuse-v3-2026-09-24')
prefix='tower-reuse-20260924-v3-dependents'
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
bindings=json.loads((A/'tower-reuse-20260924-v3-sources.json').read_text())
def verify():
    for name,sha in bindings.items():assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,name
cargo='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
report=dict(crates=[],all_processes_reaped=False)
for crate in ['hyperlimit','hypersolve']:
    verify();repo=root/crate;os.utime(repo/'src/lib.rs',None)
    record=dict(crate=crate,checks=[])
    for feature in ['--all-features','--no-default-features']:
        log=A/f'{prefix}-{crate}-check-{len(record["checks"])}.log'
        with log.open('w') as out:code=subprocess.run([cargo,'check','--all-targets',feature,'--locked','--offline'],cwd=repo,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=1200).returncode
        verify();record['checks'].append(dict(feature=feature,returncode=code,log=log.name))
        assert code==0 and 'warning:' not in log.read_text(),log
    build=A/f'{prefix}-{crate}-build.jsonl'
    with build.open('w') as out,(A/f'{prefix}-{crate}-build.log').open('w') as err:
        code=subprocess.run([cargo,'test','--lib','--release','--all-features','--no-run','--message-format=json','--locked','--offline'],cwd=repo,env=env,stdout=out,stderr=err,timeout=1200).returncode
    verify();assert code==0
    artifacts=[json.loads(line) for line in build.read_text().splitlines()]
    hyperreal=[r for r in artifacts if r.get('reason')=='compiler-artifact' and r['target']['name']=='hyperreal']
    assert hyperreal and all(str(root/'hyperreal') in r['package_id'] for r in hyperreal)
    record['hyperreal_artifacts']=hyperreal
    tests=[r for r in artifacts if r.get('reason')=='compiler-artifact' and r.get('executable') and r['target']['name']==crate]
    assert len(tests)==1 and not tests[0]['fresh']
    binary=A/f'{prefix}-{crate}-libtest';shutil.copy2(tests[0]['executable'],binary)
    log=A/f'{prefix}-{crate}-tests.log';start=time.monotonic()
    with log.open('w') as out:
        try:code=subprocess.run([str(binary),'--test-threads=2','--color','never'],cwd=repo,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=180).returncode
        except subprocess.TimeoutExpired:code='timeout'
    verify();record.update(returncode=code,elapsed_seconds=time.monotonic()-start,log=log.name,binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest());report['crates'].append(record)
    print(crate,code,log.read_text()[-1800:],flush=True)
    if code!=0:break
report['all_processes_reaped']=True;report['all_sources_unchanged']=True
(A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
