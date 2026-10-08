from pathlib import Path
import concurrent.futures, hashlib, json, os, re, shutil, subprocess, time

audit=Path(__file__).resolve().parent
root=Path('/tmp/hypercurve-chord-normal-witness-v2-2026-09-23')
repo=root/'hypercurve'
prefix='chord-normal-witness-20260923-broad1'
env=dict(os.environ,**json.loads((audit/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
cargo='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
focused='chord-normal-witness-20260923-focused2'
bindings=json.loads((audit/f'{focused}-sources.json').read_text())
def verify():
    for name,sha in bindings.items():
        assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,name
verify()
(audit/f'{prefix}-sources.json').write_text(json.dumps(bindings,indent=2)+'\n')
summary=json.loads((audit/f'{focused}-terminal.json').read_text())
assert summary['all_processes_reaped']
assert summary['hypercurve']['cases'][0]['passed']
assert all(r['passed'] for r in summary['hypersolve']['cases'])
binary=audit/f'{focused}-hypercurve-libtest'
binary_hash=hashlib.sha256(binary.read_bytes()).hexdigest()
assert binary_hash==summary['hypercurve']['binary_sha256']
print('Qualifying focused candidate executable',binary_hash,flush=True)
checks=[]
hs_summary=json.loads((audit/'low-degree-witness-20260923-all-tests-terminal.json').read_text())
assert hs_summary['all_processes_reaped'] and hs_summary['passed']==506
assert hs_summary['binary_sha256']==summary['hypersolve']['binary_sha256']
print('Reusing source-bound Hypersolve qualification: all 506 library tests passed.',flush=True)

names=[line[:-6] for line in subprocess.check_output([str(binary),'--list'],text=True).splitlines() if line.endswith(': test')]
parent_rows={r['name']:r for r in json.loads((audit/'algebraic-query-ray-ownership-20260923-broad1-cases.json').read_text())}
parent_summary=json.loads((audit/'algebraic-query-ray-ownership-20260923-broad1-terminal.json').read_text())
assert parent_summary['all_processes_reaped']
assert hashlib.sha256((audit/'algebraic-query-ray-ownership-20260923-focused1-libtest').read_bytes()).hexdigest()==parent_summary['binary_sha256']
new_names=set(names)-set(parent_rows)
assert new_names == {'bezier_offset::conversion_tests::chord_normal_recursive_frame_retains_center_and_oriented_unit_normal'}, new_names
assert not set(parent_rows)-set(names)
def run(job):
    index,name=job;log=audit/f'{prefix}-case-{index:04d}.log';start=time.monotonic()
    with log.open('w') as out:
        try:
            code=subprocess.run([str(binary),'--exact',name,'--test-threads=1','--nocapture','--color','never'],cwd=repo,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=75).returncode
        except subprocess.TimeoutExpired:code='timeout'
    output=log.read_text();assert 'running 1 test' in output
    row=dict(name=name,returncode=code,passed=code==0 and '1 passed;' in output,ignored=code==0 and '1 ignored;' in output,elapsed_seconds=time.monotonic()-start,limit_seconds=75,log=log.name)
    if not row['passed'] and not row['ignored']:
        parent=parent_rows.get(name)
        print('Not passed',name,code,'parent status',None if parent is None else parent['returncode'],output[-1200:],flush=True)
    return row
rows=[]
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
    jobs=[pool.submit(run,(i,name)) for i,name in enumerate(names)]
    for future in concurrent.futures.as_completed(jobs):
        rows.append(future.result());(audit/f'{prefix}-cases.json').write_text(json.dumps(rows,indent=2)+'\n')
        if len(rows)%100==0:print('Completed',len(rows),'of',len(names),'cases',flush=True)
for crate in ['hypersolve','hypercurve']:
    for feature in ['--all-features','--no-default-features']:
        cmd=[cargo,'check','--all-targets',feature,'--locked','--offline']
        log=audit/f'{prefix}-check-{len(checks)}.log'
        with log.open('w') as out:
            code=subprocess.run(cmd,cwd=root/crate,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=1200).returncode
        verify();checks.append(dict(crate=crate,command=cmd,returncode=code,log=log.name));assert code==0
        assert 'warning:' not in log.read_text(),log.name
        print('Passed all-target',crate,feature,'without warnings',flush=True)
verify();assert hashlib.sha256(binary.read_bytes()).hexdigest()==binary_hash
def detail(row):
    output=(audit/row['log']).read_text();pos=output.find(' panicked at ')
    if pos<0:return None
    output=output[pos:].split('note: run with')[0].split('failures:')[0].strip()
    return re.sub(r'(?:/tmp/[^/]+/hypercurve/)?(src/[^:\n]+):\d+:\d+',r'\1:<location>',output)
failed=[r for r in rows if not r['passed'] and not r['ignored']]
new_failures=[r for r in failed if r['name'] not in parent_rows or parent_rows[r['name']]['passed']]
changed=[r for r in failed if r['name'] in parent_rows and not parent_rows[r['name']]['passed'] and (r['returncode']!=parent_rows[r['name']]['returncode'] or detail(r)!=detail(parent_rows[r['name']]))]
report=dict(hypersolve_passed=506,hypersolve_binary_sha256=summary['hypersolve']['binary_sha256'],passed=sum(r['passed'] for r in rows),ignored=sum(r['ignored'] for r in rows),failed=failed,new_failures=new_failures,changed_existing_nonpasses=changed,improved=[r['name'] for r in rows if r['passed'] and r['name'] in parent_rows and not parent_rows[r['name']]['passed']],checks=checks,binary_sha256=binary_hash,parent_binary_sha256=parent_summary['binary_sha256'],all_sources_unchanged=True,all_processes_reaped=True)
(audit/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
print('Terminal',{'passed':report['passed'],'ignored':report['ignored'],'nonpasses':len(failed),'new_failures':[r['name'] for r in new_failures],'changed_existing_nonpasses':[r['name'] for r in changed],'improved':report['improved']},flush=True)
