from pathlib import Path
import hashlib,json,os,shutil,subprocess,time

audit=Path(__file__).resolve().parent
prefix='rational-witness-axis-20260923'
cargo='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
env=dict(os.environ,**json.loads((audit/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
env.update(CARGO_PROFILE_DEV_DEBUG='0',CARGO_PROFILE_TEST_DEBUG='0')
reports={}
for kind,root in [('candidate',Path('/tmp/hypercurve-rational-witness-axis-2026-09-23')),('parent',Path('/tmp/hypersolve-rational-witness-axis-parent-2026-09-23'))]:
    repo=root/'hypersolve';b=json.loads((audit/f'{prefix}-{kind}-sources.json').read_text())
    def verify():
        for n,h in b.items():assert hashlib.sha256((root/n).read_bytes()).hexdigest()==h,n
    verify();os.utime(repo/'src/lib.rs',None);stem=f'{prefix}-{kind}'
    command=[cargo,'test','--lib','--all-features','--no-run','--message-format=json','--locked','--offline']
    with (audit/f'{stem}-build.jsonl').open('w') as out,(audit/f'{stem}-build.log').open('w') as err:code=subprocess.run(command,cwd=repo,env=env,stdout=out,stderr=err,timeout=900).returncode
    verify();assert code==0
    artifacts=[r for r in map(json.loads,(audit/f'{stem}-build.jsonl').read_text().splitlines()) if r.get('reason')=='compiler-artifact'];hr_rows=[r for r in artifacts if r['target']['name']=='hyperreal'];assert hr_rows and all(str(root/'hyperreal') in r['package_id'] for r in hr_rows)
    artifacts=[r for r in artifacts if r.get('executable') and r['target']['name']=='hypersolve'];assert len(artifacts)==1 and not artifacts[0]['fresh'];binary=audit/f'{stem}-libtest';shutil.copy2(artifacts[0]['executable'],binary)
    command=[str(binary),'--test-threads=2','--color','never'] if kind=='candidate' else [str(binary),'--exact','algebraic_tensor_image::tests::selected_rational_quadratic_witness_eliminates_its_tensor_axis','--nocapture','--test-threads=1','--color','never']
    log=audit/f'{stem}-tests.log';start=time.monotonic()
    with log.open('w') as out:code=subprocess.run(command,cwd=repo,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=300).returncode
    verify();output=log.read_text();print(kind,output[-1600:],flush=True)
    if kind=='candidate':assert code==0 and '507 passed; 0 failed;' in output
    else:assert code==101 and '0 passed; 1 failed;' in output
    reports[kind]=dict(returncode=code,elapsed_seconds=time.monotonic()-start,binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest())
    if kind=='candidate':
        checks=[]
        for feature in ['--all-features','--no-default-features']:
            command=[cargo,'check','--all-targets',feature,'--locked','--offline'];log=audit/f'{stem}-check-{len(checks)}.log'
            with log.open('w') as out:code=subprocess.run(command,cwd=repo,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=900).returncode
            verify();assert code==0 and 'warning:' not in log.read_text();checks.append(dict(command=command,returncode=code,log=log.name));print('Hypersolve check passed:',feature,flush=True)
        reports[kind]['checks']=checks
reports.update(all_sources_unchanged=True,all_processes_reaped=True)
(audit/f'{prefix}-terminal.json').write_text(json.dumps(reports,indent=2)+'\n')
print('Terminal: rational witness qualification reaped.',flush=True)
