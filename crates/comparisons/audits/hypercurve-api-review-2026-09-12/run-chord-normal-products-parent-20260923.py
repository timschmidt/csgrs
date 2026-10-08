from pathlib import Path
import hashlib,json,os,shutil,subprocess,time

audit=Path(__file__).resolve().parent
root=Path('/tmp/hyperreal-chord-normal-products-2026-09-23')
prefix='chord-normal-products-20260923-parent2'
cargo='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
env=dict(os.environ,**json.loads((audit/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
env.update(CARGO_PROFILE_DEV_DEBUG='0',CARGO_PROFILE_TEST_DEBUG='0')
report={}
for kind in ['parent']:
    repo=root/kind/'hyperreal'
    bindings=json.loads((audit/f'chord-normal-products-20260923-{kind}-sources.json').read_text())
    def verify():
        for n,h in bindings.items(): assert hashlib.sha256((repo/n).read_bytes()).hexdigest()==h,n
    verify()
    stem=f'{prefix}-{kind}'
    command=[cargo,'test','--lib','--all-features','--no-run','--message-format=json','--locked','--offline']
    with (audit/f'{stem}-build.jsonl').open('w') as out,(audit/f'{stem}-build.log').open('w') as err:
        code=subprocess.run(command,cwd=repo,env=env,stdout=out,stderr=err,timeout=900).returncode
    verify()
    assert code==0
    artifacts=[r for r in map(json.loads,(audit/f'{stem}-build.jsonl').read_text().splitlines()) if r.get('reason')=='compiler-artifact' and r.get('executable') and r['target']['name']=='hyperreal']
    assert len(artifacts)==1 and not artifacts[0]['fresh']
    binary=audit/f'{stem}-libtest';shutil.copy2(artifacts[0]['executable'],binary)
    command=[str(binary),'quadratic_tower','--test-threads=2','--nocapture','--color','never'] if kind=='candidate' else [str(binary),'independently_normalized_chord_normals_replay_exactly','--test-threads=1','--nocapture','--color','never']
    log=audit/f'{stem}-tests.log';start=time.monotonic()
    with log.open('w') as out: code=subprocess.run(command,cwd=repo,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=90).returncode
    verify();output=log.read_text()
    report[kind]=dict(returncode=code,elapsed_seconds=time.monotonic()-start,binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),log=log.name)
    print(kind,output[-2500:],flush=True)
report.update(all_sources_unchanged=True,all_processes_reaped=True)
(audit/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
print('Terminal: both Hyperreal candidates reaped.',flush=True)
