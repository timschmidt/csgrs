from pathlib import Path
import os, sys, subprocess, time, json, hashlib, shutil
root=Path('/tmp/hypercurve-contact-isolation-2026-09-23')
audit=Path(__file__).resolve().parent
repo=root/'hypersolve'
prefix='contact-fiber-20260923-'+sys.argv[1]
name='contact_fiber_probe_20260923'
bindings={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in repo.rglob('*') if p.is_file()}
deps=json.loads((audit/'boundary-api-20260923-check5-sources.json').read_text())['isolated']
for name_,sha in deps.items():
    if not name_.startswith(('hypercurve/','hypersolve/')): bindings[name_]=sha
(audit/(prefix+'-sources.json')).write_text(json.dumps(bindings,indent=2)+'\n')
def verify():
    for path,sha in bindings.items(): assert hashlib.sha256((root/path).read_bytes()).hexdigest()==sha, path
verify()
env=dict(os.environ,**json.loads((audit/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
start=time.monotonic()
with (audit/(prefix+'-build.jsonl')).open('w') as out, (audit/(prefix+'-build.log')).open('w') as err:
    code=subprocess.run(['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo','build','--release','--all-features','--example',name,'--message-format=json','--locked','--offline'],cwd=repo,env=env,stdout=out,stderr=err,timeout=900).returncode
verify()
if code:
    print((audit/(prefix+'-build.log')).read_text(),flush=True)
    print((audit/(prefix+'-build.jsonl')).read_text()[-6000:],flush=True)
    sys.exit(code)
print('Build complete',time.monotonic()-start,flush=True)
binary=audit/(prefix+'-probe')
for line in (audit/(prefix+'-build.jsonl')).read_text().splitlines():
    row=json.loads(line)
    if row.get('reason')=='compiler-artifact' and row.get('executable') and row['target']['name']==name:
        shutil.copy2(row['executable'],binary)
start=time.monotonic()
with (audit/(prefix+'-run.log')).open('w') as out:
    try: code=subprocess.run([str(binary)],cwd=repo,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=45).returncode
    except subprocess.TimeoutExpired: code='timeout'
verify()
result=dict(returncode=code,elapsed_seconds=time.monotonic()-start,binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),sources_unchanged=True,all_processes_reaped=True)
(audit/(prefix+'-result.json')).write_text(json.dumps(result,indent=2)+'\n')
print(result,flush=True)
print((audit/(prefix+'-run.log')).read_text()[-7000:],flush=True)
