from pathlib import Path
import hashlib,json,os,shutil,subprocess,time

audit=Path(__file__).resolve().parent
root=Path('/tmp/hyper-chord-normal-products-layers-2026-09-23')
hr=Path('/tmp/hyperreal-chord-normal-products-2026-09-23/candidate/hyperreal')
prefix='chord-normal-products-20260923-layers2'
cargo='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
env=dict(os.environ,**json.loads((audit/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
env.update(CARGO_PROFILE_DEV_DEBUG='0',CARGO_PROFILE_TEST_DEBUG='0')
b=json.loads((audit/'chord-normal-products-20260923-layers2-sources.json').read_text())
def verify():
    for n,h in b.items(): assert hashlib.sha256((root/n).read_bytes()).hexdigest()==h,n
verify();reports={}
for crate in ['hyperlimit','hypersolve']:
    stem=f'{prefix}-{crate}'
    command=[cargo,'test','--lib','--all-features','--no-run','--message-format=json','--locked','--offline']
    with (audit/f'{stem}-build.jsonl').open('w') as out,(audit/f'{stem}-build.log').open('w') as err:
        code=subprocess.run(command,cwd=root/crate,env=env,stdout=out,stderr=err,timeout=900).returncode
    verify();assert code==0
    artifacts=[r for r in map(json.loads,(audit/f'{stem}-build.jsonl').read_text().splitlines()) if r.get('reason')=='compiler-artifact']
    hr_rows=[r for r in artifacts if r['target']['name']=='hyperreal'];assert hr_rows and all(str(root/'hyperreal') in r['package_id'] for r in hr_rows)
    artifacts=[r for r in artifacts if r.get('executable') and r['target']['name']==crate];assert len(artifacts)==1 and not artifacts[0]['fresh']
    binary=audit/f'{stem}-libtest';shutil.copy2(artifacts[0]['executable'],binary)
    log=audit/f'{stem}-tests.log';start=time.monotonic()
    with log.open('w') as out: code=subprocess.run([str(binary),'--test-threads=2','--color','never'],cwd=root/crate,stdout=out,stderr=subprocess.STDOUT,timeout=300).returncode
    verify();output=log.read_text();print(crate,output[-1200:],flush=True);assert code==0 and '0 failed;' in output
    reports[crate]=dict(returncode=code,elapsed_seconds=time.monotonic()-start,binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),hyperreal_package_ids=[r['package_id'] for r in hr_rows])
reports.update(all_sources_unchanged=True,all_processes_reaped=True)
(audit/f'{prefix}-terminal.json').write_text(json.dumps(reports,indent=2)+'\n')
print('Terminal: both dependent layer suites qualified against the new Hyperreal source and reaped.',flush=True)
