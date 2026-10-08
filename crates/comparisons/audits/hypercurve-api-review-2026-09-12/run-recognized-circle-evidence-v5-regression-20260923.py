from pathlib import Path
import concurrent.futures, hashlib, json, os, subprocess, time
A=Path(__file__).resolve().parent
root=Path('/tmp/hypercurve-recognized-circle-evidence-v5-2026-09-23')
repo=root/'hypercurve'
prefix='recognized-circle-evidence-20260923-v5-regression'
env=dict(os.environ, **json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
bindings=json.loads((A/'recognized-circle-evidence-20260923-v5-sources.json').read_text())
def verify():
    for name,sha in bindings.items(): assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,name
verify()
focused=json.loads((A/'recognized-circle-evidence-20260923-v5-terminal.json').read_text())
assert focused['all_processes_reaped'] and focused['all_sources_unchanged']
assert all(row['passed'] for row in focused['cases'])
binary=A/'recognized-circle-evidence-20260923-v5-libtest'
binary_hash=hashlib.sha256(binary.read_bytes()).hexdigest()
assert binary_hash==focused['binary_sha256']
parents={r['name']:r for r in json.loads((A/'chord-normal-frame-20260923-broad2-cases.json').read_text())}
for filename in ['selected-overlap-cache-20260923-regression-cases.json','selected-overlap-cache-20260923-v2-cases.json','selected-overlap-cache-20260923-v3-cases.json']:
    for row in json.loads((A/filename).read_text()): parents[row['name']]=dict(ignored=False,**row) if 'ignored' not in row else row
names=[line[:-6] for line in subprocess.check_output([str(binary),'--list'],text=True).splitlines() if line.endswith(': test')]
assert set(names)==set(parents)
focused_names={row['name'] for row in focused['cases']}
selected=[name for name in names if name not in focused_names and (parents[name]['passed'] or parents[name]['ignored'])]
skipped=[parents[name] for name in names if name not in focused_names and name not in selected]
(A/f'{prefix}-selection.json').write_text(json.dumps(dict(selected=selected,focused=sorted(focused_names),not_repeated_existing_nonpasses=skipped),indent=2)+'\n')
def run(job):
    index,name=job
    log=A/f'{prefix}-case-{index:04d}.log'
    start=time.monotonic()
    with log.open('w') as out:
        try: code=subprocess.run([str(binary),'--exact',name,'--test-threads=1','--nocapture','--color','never'],cwd=repo,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=75).returncode
        except subprocess.TimeoutExpired: code='timeout'
    output=log.read_text()
    assert 'running 1 test' in output
    row=dict(name=name,returncode=code,passed=code==0 and '1 passed;' in output,ignored=code==0 and '1 ignored;' in output,elapsed_seconds=time.monotonic()-start,limit_seconds=75,log=log.name)
    if not row['passed'] and not row['ignored']: print('Not passed:',name,code,flush=True)
    return row
rows=[]
print('Testing remaining previous passes and migrated callers:',len(selected),flush=True)
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
    futures=[pool.submit(run,job) for job in enumerate(selected)]
    for future in concurrent.futures.as_completed(futures):
        rows.append(future.result())
        (A/f'{prefix}-cases.json').write_text(json.dumps(rows,indent=2)+'\n')
        if len(rows)%100==0: print('Completed',len(rows),'of',len(selected),flush=True)
verify()
assert hashlib.sha256(binary.read_bytes()).hexdigest()==binary_hash
failed=[row for row in rows if not row['passed'] and not row['ignored']]
report=dict(attempted=len(rows),passed=sum(row['passed'] for row in rows),ignored=sum(row['ignored'] for row in rows),failed=failed,focused_cases=focused['cases'],not_repeated_existing_nonpasses=skipped,binary_sha256=binary_hash,all_sources_unchanged=True,all_processes_reaped=True)
(A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
print('Terminal:',dict(attempted=len(rows),passed=report['passed'],ignored=report['ignored'],failed=[row['name'] for row in failed]),flush=True)
