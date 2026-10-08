from pathlib import Path
import hashlib,json,os,shutil,subprocess,time

audit=Path(__file__).resolve().parent
root=Path('/tmp/hypercurve-chord-normal-products-minimal-2026-09-23')
hr=Path('/tmp/hyperreal-chord-normal-products-2026-09-23/candidate/hyperreal')
prefix='chord-normal-products-20260923-layers1'
cargo='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
env=dict(os.environ,**json.loads((audit/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
env.update(CARGO_PROFILE_DEV_DEBUG='0',CARGO_PROFILE_TEST_DEBUG='0')
b=json.loads((audit/'chord-normal-products-20260923-minimal-sources.json').read_text())
def verify():
    for n,h in b.items(): assert hashlib.sha256((root/n).read_bytes()).hexdigest()==h,n
verify();checks=[]
for feature in ['--all-features','--no-default-features']:
    log=audit/f'{prefix}-check-{len(checks)}.log'
    command=[cargo,'check','--all-targets',feature,'--locked','--offline']
    with log.open('w') as out: code=subprocess.run(command,cwd=hr,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=900).returncode
    verify();assert code==0 and 'warning:' not in log.read_text()
    checks.append(dict(command=command,returncode=code,log=log.name));print('Hyperreal check passed:',feature,flush=True)
command=[cargo,'test','--lib','--all-features','--no-run','--message-format=json','--locked','--offline']
with (audit/f'{prefix}-hypersolve-build.jsonl').open('w') as out,(audit/f'{prefix}-hypersolve-build.log').open('w') as err:
    code=subprocess.run(command,cwd=root/'hypersolve',env=env,stdout=out,stderr=err,timeout=900).returncode
verify();assert code==0
artifacts=[r for r in map(json.loads,(audit/f'{prefix}-hypersolve-build.jsonl').read_text().splitlines()) if r.get('reason')=='compiler-artifact' and r.get('executable') and r['target']['name']=='hypersolve'];assert len(artifacts)==1 and not artifacts[0]['fresh']
binary=audit/f'{prefix}-hypersolve-libtest';shutil.copy2(artifacts[0]['executable'],binary)
log=audit/f'{prefix}-hypersolve-tests.log';start=time.monotonic()
with log.open('w') as out: code=subprocess.run([str(binary),'--test-threads=2','--color','never'],cwd=root/'hypersolve',stdout=out,stderr=subprocess.STDOUT,timeout=300).returncode
verify();print(log.read_text()[-1200:],flush=True);assert code==0 and '506 passed; 0 failed;' in log.read_text()
report=dict(hyperreal_checks=checks,hypersolve=dict(returncode=code,passed=506,elapsed_seconds=time.monotonic()-start,binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest()),all_sources_unchanged=True,all_processes_reaped=True)
(audit/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
subprocess.run(['python3',str(audit/'run-chord-normal-products-minimal-20260923.py')],cwd=audit,check=True)
print('Terminal: layer qualification and minimal carrier diagnostic reaped.',flush=True)
