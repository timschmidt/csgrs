from pathlib import Path
import hashlib,json,os,shutil,subprocess,time
w=Path('/home/tim/Documents/GitHub/workspace');a=Path(__file__).resolve().parent;r=Path('/tmp/hypercurve-region-admission-qualification');prefix='material-components-candidate1'
selected={'hypercurve':['src/bezier_region.rs','tests/hypercurve_bezier_region.rs'],'hyperbrep':['src/builder.rs'],'hyperdrc':['src/lib.rs']}
working=[]
for repo,files in selected.items():
    changed=subprocess.check_output(['git','diff','--name-only'],cwd=w/repo,text=True).splitlines()
    assert sorted(x for x in changed if not (repo=='hypercurve' and x=='src/bezier_offset.rs'))==sorted(files),(repo,changed)
    for file in files:
        shutil.copy2(w/repo/file,r/repo/file);working.append(dict(file=repo+'/'+file,sha256=hashlib.sha256((w/repo/file).read_bytes()).hexdigest()))
    (a/(prefix+'-'+repo+'-source.patch')).write_bytes(subprocess.check_output(['git','diff','--',*files],cwd=w/repo))
working.append(dict(file='hypercurve/src/bezier_offset.rs',sha256=hashlib.sha256((w/'hypercurve/src/bezier_offset.rs').read_bytes()).hexdigest()))
manifest=[]
for repo in sorted(r.iterdir()):
    if repo.is_dir():
        for p in sorted(repo.rglob('*')):
            if p.is_file() and 'target' not in p.parts:manifest.append(dict(file=str(p.relative_to(r)),sha256=hashlib.sha256(p.read_bytes()).hexdigest()))
(a/(prefix+'-isolated-sources.json')).write_text(json.dumps(manifest,indent=2)+'\n');(a/(prefix+'-working-sources.json')).write_text(json.dumps(working,indent=2)+'\n')
settings=json.loads((a/'opposed-endpoint-contact-full1-build-settings.json').read_text());env=dict(os.environ,**settings);cargo='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo';builds=[];tests=[];archive=a/(prefix+'-libraries');archive.mkdir()
def verify():
    for row in manifest:assert hashlib.sha256((r/row['file']).read_bytes()).hexdigest()==row['sha256'],row['file']
    for row in working:assert hashlib.sha256((w/row['file']).read_bytes()).hexdigest()==row['sha256'],row['file']
for repo in selected:
    for kind in ['check','test-build']:
        args=['check','--all-targets','--no-default-features'] if kind=='check' else ['test','--release','--all-features','--lib','--no-run','--message-format=json']
        if kind=='test-build' and repo=='hypercurve':args+=['--test','hypercurve_bezier_region']
        cmd=[cargo,*args,'--locked','--offline'];stem=prefix+'-'+repo+'-'+kind;start=time.monotonic()
        with (a/(stem+'.log')).open('w') as err,(a/(stem+'.jsonl')).open('w') as out:
            result=subprocess.run(cmd,cwd=r/repo,env=env,stdout=out if kind=='test-build' else err,stderr=err,timeout=900)
        row=dict(repo=repo,kind=kind,returncode=result.returncode,elapsed_seconds=time.monotonic()-start,command=cmd);builds.append(row);(a/(prefix+'-builds.json')).write_text(json.dumps(builds,indent=2)+'\n');print(repo,kind,row['returncode'],round(row['elapsed_seconds'],2),flush=True)
        if result.returncode:print((a/(stem+'.log')).read_text()[-7000:],flush=True);verify();raise SystemExit(1)
    artifacts=[json.loads(x) for x in (a/(prefix+'-'+repo+'-test-build.jsonl')).read_text().splitlines()]
    for item in artifacts:
        if item.get('reason')!='compiler-artifact' or not item.get('executable') or not item['profile']['test']:continue
        path=Path(item['executable']);binary=archive/path.name;shutil.copy2(path,binary)
        filters=['bezier_region::tests::material_component_'] if repo=='hypercurve' and item['target']['kind']==['lib'] else ['geometry::tests::'] if repo=='hyperdrc' else ['']
        for filter_ in filters:
            cmd=[str(binary),'--test-threads=2','--color','never']+([filter_] if filter_ else []);stem=prefix+'-'+repo+'-'+item['target']['name']+'-test';start=time.monotonic()
            with (a/(stem+'.log')).open('w') as log:result=subprocess.run(cmd,cwd=r/repo,stdout=log,stderr=subprocess.STDOUT,timeout=300)
            row=dict(repo=repo,target=item['target']['name'],command=cmd,returncode=result.returncode,elapsed_seconds=time.monotonic()-start,sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),log=stem+'.log');tests.append(row);(a/(prefix+'-tests.json')).write_text(json.dumps(tests,indent=2)+'\n');print(repo,'tests',item['target']['name'],result.returncode,round(row['elapsed_seconds'],2),flush=True);print((a/(stem+'.log')).read_text()[-5000:],flush=True)
            if result.returncode:verify();raise SystemExit(1)
verify();print('all builds/tests complete; bound sources unchanged',flush=True)
