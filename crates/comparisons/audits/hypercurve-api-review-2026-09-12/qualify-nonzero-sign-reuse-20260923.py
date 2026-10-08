from pathlib import Path
import hashlib,json,os,subprocess,shutil,sys,time
root=Path('/tmp/hypercurve-nonzero-sign-reuse-2026-09-23'); repo=root/'hypersolve'; audit=Path(__file__).resolve().parent
label=sys.argv[1]; prefix='nonzero-sign-reuse-20260923-'+label
outdir=audit/(prefix+'-tests'); outdir.mkdir()
bindings={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in repo.rglob('*') if p.is_file()}
deps=json.loads((audit/'boundary-api-20260923-check5-sources.json').read_text())['isolated']
for p,sha in deps.items():
    if not p.startswith(('hypercurve/','hypersolve/')): bindings[p]=sha
(audit/(prefix+'-sources.json')).write_text(json.dumps(bindings,indent=2)+'\n')
def verify():
    for p,sha in bindings.items(): assert hashlib.sha256((root/p).read_bytes()).hexdigest()==sha,p
verify()
env=dict(os.environ,**json.loads((audit/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
cargo='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
cmd=[cargo,'test','--release','--lib','--all-features','--no-run','--message-format=json','--locked','--offline']
with (audit/(prefix+'-build.jsonl')).open('w') as out, (audit/(prefix+'-build.log')).open('w') as err:
    code=subprocess.run(cmd,cwd=repo,env=env,stdout=out,stderr=err,timeout=900).returncode
verify()
if code:
    print((audit/(prefix+'-build.log')).read_text(),flush=True)
    print((audit/(prefix+'-build.jsonl')).read_text()[-8000:],flush=True)
    sys.exit(code)
for line in (audit/(prefix+'-build.jsonl')).read_text().splitlines():
    row=json.loads(line)
    if row.get('reason')=='compiler-artifact' and row.get('executable') and row['target']['name']=='hypersolve':
        binary=audit/(prefix+'-libtest'); shutil.copy2(row['executable'],binary)
print('Fresh library built',hashlib.sha256(binary.read_bytes()).hexdigest(),flush=True)
listing=subprocess.run([str(binary),'--list','--format','terse'],capture_output=True,text=True,check=True).stdout
names=[line[:-6] for line in listing.splitlines() if line.endswith(': test')]
focused=[n for n in names if '::certified_nonzero_signs_' in n]; assert len(focused)==2,focused
results=[]
for name in focused+[n for n in names if n not in focused]:
    start=time.monotonic()
    with (outdir/(name.replace('::','__')+'.log')).open('w') as out:
        try: code=subprocess.run([str(binary),'--exact',name,'--nocapture','--test-threads=1'],cwd=repo,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=75).returncode
        except subprocess.TimeoutExpired: code='timeout'
    results.append(dict(name=name,returncode=code,elapsed_seconds=time.monotonic()-start))
    (audit/(prefix+'-results.json')).write_text(json.dumps(results,indent=2)+'\n')
    if name in focused or len(results)%50==0 or code:
        print(results[-1],flush=True)
    if name in focused and code:
        verify(); print('Focused failure; stopped and reaped.',flush=True); sys.exit(1)
verify()
checks=[]
for flags in [['--all-features'],['--no-default-features']]:
    command=[cargo,'check','--all-targets',*flags,'--locked','--offline']
    with (audit/(prefix+'-check-'+flags[0][2:]+'.log')).open('w') as out:
        code=subprocess.run(command,cwd=repo,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=900).returncode
    verify(); checks.append(dict(command=command,returncode=code)); print(checks[-1],flush=True)
summary=dict(tests=len(results),failures=[r for r in results if r['returncode']!=0],checks=checks,binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),sources_unchanged=True,all_processes_reaped=True)
(audit/(prefix+'-summary.json')).write_text(json.dumps(summary,indent=2)+'\n')
print(summary,flush=True)
