from pathlib import Path
import concurrent.futures, hashlib, json, os, shutil, subprocess, time
A=Path(__file__).resolve().parent
root=Path('/tmp/hypercurve-normalized-straight-corners-v1-2026-09-24')
repo=root/'hypercurve'
prefix='direct-endpoints-20260924-parent-path'
env=dict(os.environ, **json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
bindings=json.loads((A/'normalized-straight-corners-20260924-v1-sources.json').read_text())
def verify():
    for name,sha in bindings.items(): assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,name
verify()
os.utime(repo/'src/lib.rs',None)
cargo='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
selected={'hypercurve_path_closure':{'homogeneous_boundary_closes_through_boolean_corners_and_offset'}}

command=[cargo,'test','--release','--all-features','--no-run','--message-format=json','--locked','--offline']
for target in selected: command.extend(['--test',target])
with (A/f'{prefix}-build.jsonl').open('w') as out, (A/f'{prefix}-build.log').open('w') as err:
    code=subprocess.run(command,cwd=repo,env=env,stdout=out,stderr=err,timeout=1200).returncode
verify()
assert code==0
assert 'warning:' not in (A/f'{prefix}-build.log').read_text()
artifacts={}
for line in (A/f'{prefix}-build.jsonl').read_text().splitlines():
    row=json.loads(line)
    if row.get('reason')=='compiler-artifact' and row.get('executable') and row['target']['name'] in selected:
        assert not row['fresh']
        artifacts[row['target']['name']]=row
assert set(artifacts)==set(selected)
binaries={};jobs=[]
for target,row in artifacts.items():
    binary=A/f'{prefix}-{target}'
    shutil.copy2(row['executable'],binary)
    binaries[target]=dict(path=str(binary),sha256=hashlib.sha256(binary.read_bytes()).hexdigest())
    names={line[:-6] for line in subprocess.check_output([str(binary),'--list'],text=True).splitlines() if line.endswith(': test')}
    names_to_run=names if selected[target] is None else selected[target]
    assert names_to_run<=names
    jobs.extend((target,name) for name in sorted(names_to_run))
(A/f'{prefix}-selection.json').write_text(json.dumps(dict(selected=jobs,binaries=binaries),indent=2)+'\n')
def run(job):
    index,(target,name)=job
    binary=Path(binaries[target]['path']);log=A/f'{prefix}-case-{index:02d}.log'
    start=time.monotonic();limit=90
    with log.open('w') as out:
        try: code=subprocess.run([str(binary),'--exact',name,'--nocapture','--test-threads=1','--color','never'],cwd=repo,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=limit).returncode
        except subprocess.TimeoutExpired: code='timeout'
    output=log.read_text();assert 'running 1 test' in output
    return dict(target=target,name=name,returncode=code,passed=code==0 and '1 passed;' in output,elapsed_seconds=time.monotonic()-start,limit_seconds=limit,log=log.name)
rows=[]
print('Built relevant public API integrations:',len(jobs),flush=True)
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
    for row in pool.map(run,enumerate(jobs)):
        rows.append(row)
        (A/f'{prefix}-cases.json').write_text(json.dumps(rows,indent=2)+'\n')
        print(row,flush=True)
verify()
for target,b in binaries.items(): assert hashlib.sha256(Path(b['path']).read_bytes()).hexdigest()==b['sha256']
report=dict(command=command,binaries=binaries,cases=rows,passed=sum(row['passed'] for row in rows),all_sources_unchanged=True,all_processes_reaped=True)
(A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
print('Terminal integrations:',report['passed'],'/',len(rows),flush=True)
