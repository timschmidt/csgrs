from pathlib import Path
import concurrent.futures,hashlib,io,json,os,shutil,subprocess,tarfile,time

audit=Path(__file__).resolve().parent; workspace=audit.parent
trial=Path('/tmp/hypercurve-contact-isolation-2026-09-23')
root=Path('/tmp/hypercurve-rational-sturm-2026-09-23'); root.mkdir()
baseline=Path('/tmp/hypercurve-boundary-api-2026-09-23')
prefix='rational-sturm-hypercurve-20260923-final1'
for path in baseline.iterdir():
    if path.is_dir() and path.name not in ['hypercurve','hypersolve']:
        (root/path.name).symlink_to(path.resolve(),target_is_directory=True)
repo=root/'hypercurve'; repo.mkdir()
archive=subprocess.check_output(['git','archive','a34c0feffa0d7dba5d5e99f1c7e20882a5fd6ff4'],cwd=workspace/'hypercurve')
with tarfile.open(fileobj=io.BytesIO(archive)) as stream: stream.extractall(repo,filter='data')
solver=root/'hypersolve'; solver.mkdir()
files=subprocess.check_output(['git','ls-files','-z'],cwd=workspace/'hypersolve').decode().split('\0')
files=[p for p in files if p]+['src/integer_interpolation/bivariate.rs','tests/data/nonph_fillet_circle_contact.json']
for name in files:
    source=trial/'hypersolve'/name
    if source.is_file():
        destination=solver/name; destination.parent.mkdir(parents=True,exist_ok=True); shutil.copy2(source,destination)
qualified=json.loads((audit/'rational-sturm-20260923-final2-sources.json').read_text())
for name in files:
    p=solver/name
    if p.is_file(): assert hashlib.sha256(p.read_bytes()).hexdigest()==qualified['hypersolve/'+name],name
bindings={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for d in [repo,solver] for p in d.rglob('*') if p.is_file()}
deps=json.loads((audit/'boundary-api-20260923-check5-sources.json').read_text())['isolated']
for name,sha in deps.items():
    if not name.startswith(('hypercurve/','hypersolve/')): bindings[name]=sha
(audit/(prefix+'-sources.json')).write_text(json.dumps(bindings,indent=2)+'\n')
control=audit/'root-charts-20260923-final2-libtest'
assert hashlib.sha256(control.read_bytes()).hexdigest()=='c47f0501a4cf7bd9d7304b672c6e3157954220beadec01a67175c39bad8f64c1'
def verify():
    for name,sha in bindings.items(): assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,name
verify()
env=dict(os.environ,**json.loads((audit/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
cargo='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
command=[cargo,'test','--lib','--release','--all-features','--no-run','--message-format=json','--locked','--offline']
start=time.monotonic()
with (audit/(prefix+'-build.jsonl')).open('w') as out,(audit/(prefix+'-build.log')).open('w') as err:
    code=subprocess.run(command,cwd=repo,env=env,stdout=out,stderr=err,timeout=1200).returncode
verify();assert code==0,(audit/(prefix+'-build.log')).read_text()[-5000:]
binary=audit/(prefix+'-libtest')
for line in (audit/(prefix+'-build.jsonl')).read_text().splitlines():
    row=json.loads(line)
    if row.get('reason')=='compiler-artifact' and row.get('executable') and row['target']['name']=='hypercurve':
        assert not row['fresh'];shutil.copy2(row['executable'],binary)
print('Fresh Hypercurve built',time.monotonic()-start,hashlib.sha256(binary.read_bytes()).hexdigest(),flush=True)
command=[cargo,'check','--all-targets','--all-features','--locked','--offline']
with (audit/(prefix+'-check.log')).open('w') as out:
    code=subprocess.run(command,cwd=repo,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=1200).returncode
verify();check=dict(command=command,returncode=code);assert code==0
print('All-target check passed',flush=True)
listing=subprocess.check_output([str(binary),'--list'],text=True)
names=[line[:-6] for line in listing.splitlines() if line.endswith(': test')]
# Revisit the four formerly held-out curve compositions, bounded like all other cases.
def run(job):
    label,exe,name,index=job
    log=audit/(prefix+'-'+label+f'-case-{index:04d}.log')
    start=time.monotonic()
    with log.open('w') as out:
        try: code=subprocess.run([str(exe),'--exact',name,'--test-threads=1','--nocapture','--color','never'],cwd=repo,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=75).returncode
        except subprocess.TimeoutExpired:code='timeout'
    output=log.read_text()
    row=dict(label=label,name=name,returncode=code,elapsed_seconds=time.monotonic()-start,passed=code==0 and '1 passed;' in output,ignored=code==0 and '1 ignored;' in output,log=log.name)
    if not row['passed'] and not row['ignored']: print('Not passed',label,name,code,output[-1300:],flush=True)
    return row
rows=[]
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
    jobs={pool.submit(run,('candidate',binary,name,index)):name for index,name in enumerate(names)}
    for future in concurrent.futures.as_completed(jobs):
        rows.append(future.result());(audit/(prefix+'-cases.json')).write_text(json.dumps(rows,indent=2)+'\n')
        if len(rows)%100==0:print('Cases completed',len(rows),'/',len(names),flush=True)
verify()
failed=[row for row in rows if not row['passed'] and not row['ignored']]
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
    replays=list(pool.map(run,[('parent',control,row['name'],index) for index,row in enumerate(failed)]))
verify()
report=dict(passed=sum(r['passed'] for r in rows),ignored=sum(r['ignored'] for r in rows),failed=failed,parent_replays=replays,new_failures=[r['name'] for r in replays if r['passed']],check=check,binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),parent_binary_sha256=hashlib.sha256(control.read_bytes()).hexdigest(),all_sources_unchanged=True,all_processes_reaped=True)
(audit/(prefix+'-terminal.json')).write_text(json.dumps(report,indent=2)+'\n')
print('Terminal', {k:v for k,v in report.items() if k not in ['failed','parent_replays']},flush=True)
