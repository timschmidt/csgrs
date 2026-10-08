from pathlib import Path
import hashlib,json,os,shutil,subprocess,time
workspace=Path('/home/tim/Documents/GitHub/workspace');audit=Path(__file__).resolve().parent;root=Path('/tmp/hypercurve-region-admission-qualification');prefix='normalized-traversal-candidate2'
selected=subprocess.check_output(['git','diff','--name-only'],cwd=workspace/'hypercurve',text=True).splitlines();selected.remove('src/bezier_offset.rs')
assert selected==['benches/bezier_region.rs','fuzz/fuzz_targets/bezier_region.rs','src/bezier_region.rs','src/curve_region_boolean.rs','tests/hypercurve_bezier_region.rs'],selected
for name in selected:shutil.copy2(workspace/'hypercurve'/name,root/'hypercurve'/name)
(audit/(prefix+'-source.patch')).write_bytes(subprocess.check_output(['git','diff','--',*selected],cwd=workspace/'hypercurve'))
manifest=[]
for repo in ['hypercurve','hyperbrep','hyperreal','hyperlimit','hyperlattice','hypersolve','hypertri']:
    for p in sorted((root/repo).rglob('*')):
        if p.is_file() and 'target' not in p.parts:manifest.append(dict(file=str(p.relative_to(root)),sha256=hashlib.sha256(p.read_bytes()).hexdigest()))
working=[dict(file=p,sha256=hashlib.sha256((workspace/'hypercurve'/p).read_bytes()).hexdigest()) for p in selected+['src/bezier_offset.rs']]
(audit/(prefix+'-isolated-sources.json')).write_text(json.dumps(manifest,indent=2)+'\n');(audit/(prefix+'-working-sources.json')).write_text(json.dumps(working,indent=2)+'\n')
settings=json.loads((audit/'opposed-endpoint-contact-full1-build-settings.json').read_text());env=dict(os.environ,**settings);cargo='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
records=[]
for name,args,cwd in [('check',['check','--all-targets','--no-default-features','--locked','--offline'],root/'hypercurve'),('test-build',['test','--release','--all-features','--test','hypercurve_bezier_region','--no-run','--locked','--offline','--message-format=json'],root/'hypercurve'),('fuzz-check',['check','--bin','bezier_region','--locked','--offline'],root/'hypercurve/fuzz')]:
    cmd=[cargo,*args];start=time.monotonic()
    with (audit/(prefix+'-'+name+'.log')).open('w') as err,(audit/(prefix+'-'+name+'.jsonl')).open('w') as out:
        r=subprocess.run(cmd,cwd=cwd,env=env,stdout=out if name=='test-build' else err,stderr=err,timeout=900)
    records.append(dict(kind=name,returncode=r.returncode,elapsed_seconds=time.monotonic()-start,command=cmd));print(name,r.returncode,round(records[-1]['elapsed_seconds'],2),flush=True)
    if r.returncode:print((audit/(prefix+'-'+name+'.log')).read_text()[-5000:],flush=True);break
for row in manifest:assert hashlib.sha256((root/row['file']).read_bytes()).hexdigest()==row['sha256'],row['file']
for row in working:assert hashlib.sha256((workspace/'hypercurve'/row['file']).read_bytes()).hexdigest()==row['sha256'],row['file']
(audit/(prefix+'-builds.json')).write_text(json.dumps(records,indent=2)+'\n')
if all(r['returncode']==0 for r in records):
    artifacts=[json.loads(x) for x in (audit/(prefix+'-test-build.jsonl')).read_text().splitlines() if x.startswith('{')]
    artifact=next(x for x in artifacts if x.get('reason')=='compiler-artifact' and x.get('executable') and x['target']['name']=='hypercurve_bezier_region')
    binary=audit/(prefix+'-test');shutil.copy2(artifact['executable'],binary);start=time.monotonic()
    with (audit/(prefix+'-test.log')).open('w') as log:r=subprocess.run([str(binary),'--test-threads=1'],stdout=log,stderr=subprocess.STDOUT,timeout=300)
    record=dict(returncode=r.returncode,elapsed_seconds=time.monotonic()-start,binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest());(audit/(prefix+'-test.json')).write_text(json.dumps(record,indent=2)+'\n');print('tests',record,flush=True);print((audit/(prefix+'-test.log')).read_text()[-9000:],flush=True)
