from pathlib import Path
import concurrent.futures,hashlib,json,os,re,shutil,subprocess,time

audit=Path(__file__).resolve().parent
root=Path('/tmp/hypercurve-rational-witness-axis-2026-09-23')
repo=root/'hypercurve'
prefix='chord-normal-frame-20260923-broad2'
cargo='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
env=dict(os.environ,**json.loads((audit/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
bindings=json.loads((audit/'rational-witness-axis-20260923-candidate-sources.json').read_text())
def verify():
    for n,h in bindings.items():assert hashlib.sha256((root/n).read_bytes()).hexdigest()==h,n
verify();(audit/f'{prefix}-sources.json').write_text(json.dumps(bindings,indent=2)+'\n')
focused=json.loads((audit/'rational-witness-axis-20260923-hypercurve-focused1-terminal.json').read_text());assert focused['all_processes_reaped'] and all(r['passed'] for r in focused['cases'])
binary=audit/'rational-witness-axis-20260923-hypercurve-focused1-libtest';binary_hash=hashlib.sha256(binary.read_bytes()).hexdigest();assert binary_hash==focused['binary_sha256']
parent_rows={r['name']:r for r in json.loads((audit/'algebraic-query-ray-ownership-20260923-broad1-cases.json').read_text())}
parent_summary=json.loads((audit/'algebraic-query-ray-ownership-20260923-broad1-terminal.json').read_text());assert parent_summary['all_processes_reaped']
names=[line[:-6] for line in subprocess.check_output([str(binary),'--list'],text=True).splitlines() if line.endswith(': test')]
new_names=set(names)-set(parent_rows);expected='bezier_offset::conversion_tests::chord_normal_recursive_frame_retains_center_and_oriented_unit_normal';assert new_names=={expected} and not set(parent_rows)-set(names)
def run(job):
    index,name=job;log=audit/f'{prefix}-case-{index:04d}.log';start=time.monotonic()
    with log.open('w') as out:
        try:code=subprocess.run([str(binary),'--exact',name,'--test-threads=1','--nocapture','--color','never'],cwd=repo,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=75).returncode
        except subprocess.TimeoutExpired:code='timeout'
    output=log.read_text();assert 'running 1 test' in output
    row=dict(name=name,returncode=code,passed=code==0 and '1 passed;' in output,ignored=code==0 and '1 ignored;' in output,elapsed_seconds=time.monotonic()-start,limit_seconds=75,log=log.name)
    if not row['passed'] and not row['ignored']:print('Not passed:',name,code,'parent:',parent_rows.get(name,{}).get('returncode'),flush=True)
    return row
first=run((names.index(expected),expected));assert first['passed'];rows=[first];print('New exact frame regression passed; testing all',len(names),'cases.',flush=True)
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
    jobs=[pool.submit(run,(i,n)) for i,n in enumerate(names) if n!=expected]
    for future in concurrent.futures.as_completed(jobs):
        rows.append(future.result());(audit/f'{prefix}-cases.json').write_text(json.dumps(rows,indent=2)+'\n')
        if len(rows)%100==0:print('Completed',len(rows),'of',len(names),flush=True)
verify();checks=[]
for feature in ['--all-features','--no-default-features']:
    command=[cargo,'check','--all-targets',feature,'--locked','--offline'];log=audit/f'{prefix}-check-{len(checks)}.log'
    with log.open('w') as out:code=subprocess.run(command,cwd=repo,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=1200).returncode
    verify();assert code==0 and 'warning:' not in log.read_text();checks.append(dict(command=command,returncode=code,log=log.name));print('Hypercurve check passed:',feature,flush=True)
def detail(row):
    output=(audit/row['log']).read_text();pos=output.find(' panicked at ')
    if pos<0:return None
    output=output[pos:].split('note: run with')[0].split('failures:')[0].strip()
    return re.sub(r'(?:/tmp/[^/]+/hypercurve/)?(src/[^:\n]+):\d+:\d+',r'\1:<location>',output)
failed=[r for r in rows if not r['passed'] and not r['ignored']]
new_failures=[r for r in failed if r['name'] not in parent_rows or parent_rows[r['name']]['passed']]
changed=[r for r in failed if r['name'] in parent_rows and not parent_rows[r['name']]['passed'] and (r['returncode']!=parent_rows[r['name']]['returncode'] or detail(r)!=detail(parent_rows[r['name']]))]
verify();assert hashlib.sha256(binary.read_bytes()).hexdigest()==binary_hash
report=dict(attempted=len(rows),passed=sum(r['passed'] for r in rows),ignored=sum(r['ignored'] for r in rows),failed=failed,new_failures=new_failures,changed_existing_nonpasses=changed,improved=[r['name'] for r in rows if r['passed'] and r['name'] in parent_rows and not parent_rows[r['name']]['passed']],checks=checks,binary_sha256=binary_hash,parent_binary_sha256=parent_summary['binary_sha256'],all_sources_unchanged=True,all_processes_reaped=True)
(audit/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
print('Terminal:',dict(attempted=len(rows),passed=report['passed'],ignored=report['ignored'],nonpasses=len(failed),new_failures=[r['name'] for r in new_failures],changed_existing_nonpasses=[r['name'] for r in changed],improved=report['improved']),flush=True)
