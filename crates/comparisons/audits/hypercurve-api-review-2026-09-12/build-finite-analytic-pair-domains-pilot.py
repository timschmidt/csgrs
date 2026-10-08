from pathlib import Path
import hashlib,io,json,os,subprocess,tarfile,time
w=Path('/home/tim/Documents/GitHub/workspace');audit=w/'hypercurve-api-review-2026-09-12';root=Path('/tmp/hypercurve-pair-domains-qualification');root.mkdir()
for name in ['hypercurve','hyperbrep']:
 d=root/name;d.mkdir()
 data=subprocess.check_output(['git','archive','HEAD'],cwd=w/name)
 with tarfile.open(fileobj=io.BytesIO(data)) as archive:archive.extractall(d,filter='data')
for name in ['hyperreal','hyperlattice','hyperlimit','hypersolve','hypertri']:(root/name).symlink_to(Path('/tmp/hypercurve-pruning-qualification')/name,target_is_directory=True)
changed=subprocess.check_output(['git','diff','--name-only'],cwd=w/'hypercurve',text=True).splitlines()
assert set(changed)=={'src/bezier_offset.rs','src/curve.rs','src/curve_support_intersection.rs'}
for relative in changed:(root/'hypercurve'/relative).write_bytes((w/'hypercurve'/relative).read_bytes())
prefix='finite-analytic-pair-domains-pilot'
(audit/(prefix+'-source.patch')).write_bytes(subprocess.check_output(['git','diff'],cwd=w/'hypercurve'))
manifest=[]
for name in ['hypercurve','hyperbrep','hyperreal','hyperlattice','hyperlimit','hypersolve','hypertri']:
 for f in sorted((root/name).rglob('*')):
  if f.is_file() and 'target' not in f.parts:manifest.append(dict(file=str(f.relative_to(root)),sha256=hashlib.sha256(f.read_bytes()).hexdigest()))
(audit/(prefix+'-isolated-sources.json')).write_text(json.dumps(manifest,indent=2)+'\n')
settings=json.loads((audit/'finite-parallel-admission-build-settings.json').read_text());env=dict(os.environ,**settings)
cargo='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
records=[]
for kind,args in [('check',['check','--all-targets','--no-default-features','--locked','--offline']),('test-build',['test','--release','--all-features','--locked','--offline','--lib','--test','hypercurve_bezier_fit_offset','--no-run','--message-format=json'])]:
 start=time.monotonic();stem=prefix+'-'+kind
 with (audit/(stem+'.log')).open('w') as log,(audit/(stem+'.jsonl')).open('w') as out:
  r=subprocess.run([cargo,*args],cwd=root/'hypercurve',env=env,stdout=out if kind=='test-build' else log,stderr=log,timeout=900)
 records.append(dict(kind=kind,command=[cargo,*args],returncode=r.returncode,elapsed_seconds=time.monotonic()-start))
 (audit/(stem+'.exit')).write_text(str(r.returncode)+'\n');(audit/(prefix+'-builds.json')).write_text(json.dumps(records,indent=2)+'\n')
 print(stem,r.returncode,flush=True)
 if r.returncode:print((audit/(stem+'.log')).read_text()[-4000:],flush=True);raise SystemExit(r.returncode)
for row in manifest:assert hashlib.sha256((root/row['file']).read_bytes()).hexdigest()==row['sha256'],row['file']
print('pilot sources unchanged',flush=True)
