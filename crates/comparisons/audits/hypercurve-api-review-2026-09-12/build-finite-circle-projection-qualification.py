from pathlib import Path
import hashlib,json,os,shutil,subprocess,time

workspace=Path('/home/tim/Documents/GitHub/workspace')
root=Path('/tmp/hypercurve-circle-projection-qualification')
audit=workspace/'hypercurve-api-review-2026-09-12'
prefix='finite-circle-projection'
changed=subprocess.check_output(['git','diff','--name-only'],cwd=workspace/'hypercurve',text=True).splitlines()
assert set(changed)=={'src/bezier_offset.rs'}
for relative in changed: shutil.copy2(workspace/'hypercurve'/relative,root/'hypercurve'/relative)
(audit/(prefix+'-source.patch')).write_bytes(subprocess.check_output(['git','diff'],cwd=workspace/'hypercurve'))
working=[]
for name in ['hypercurve','hyperbrep']:
    paths=subprocess.check_output(['git','ls-files','-c','-o','--exclude-standard','-z'],cwd=workspace/name).decode().split('\0')
    for relative in sorted(set(filter(None,paths))):
        path=workspace/name/relative
        if path.is_file(): working.append(dict(file=str(path.relative_to(workspace)),sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
(audit/(prefix+'-source-before-tests.json')).write_text(json.dumps(working,indent=2)+'\n')
manifest=[]
for name in ['hypercurve','hyperbrep','hyperlattice','hyperlimit','hyperreal','hypersolve','hypertri']:
    for path in sorted((root/name).rglob('*')):
        if path.is_file() and 'target' not in path.parts:
            manifest.append(dict(file=str(path.relative_to(root)),sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
(audit/(prefix+'-isolated-sources.json')).write_text(json.dumps(manifest,indent=2)+'\n')
toolchain=Path('/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin')
settings=dict(CARGO_BUILD_JOBS='2',RUSTC=str(toolchain/'rustc'),RUSTDOC=str(toolchain/'rustdoc'),CARGO_TARGET_DIR='/tmp/hypercurve-pruning-qualification/hyperbrep/target',CCACHE_DIR='/tmp/hypercurve-pruning-qualification/ccache',CCACHE_TEMPDIR='/tmp/hypercurve-pruning-qualification/ccache-tmp')
env=dict(os.environ,**settings)
(audit/(prefix+'-build-settings.json')).write_text(json.dumps(settings,indent=2)+'\n')
records=[]
for name in ['hypercurve','hyperbrep']:
    suffix='' if name=='hypercurve' else '-hyperbrep'
    for kind,args in [('check',['check','--all-targets','--no-default-features','--locked','--offline']),('test-build',['test','--release','--all-features','--locked','--offline','--lib','--tests','--no-run','--message-format=json'])]:
        stem=prefix+suffix+'-'+kind
        command=[str(toolchain/'cargo'),*args]; start=time.monotonic()
        with (audit/(stem+'.log')).open('w') as err,(audit/(stem+'.jsonl')).open('w') as out:
            result=subprocess.run(command,cwd=root/name,env=env,stdout=out if kind=='test-build' else err,stderr=err,timeout=900)
        (audit/(stem+'.exit')).write_text(str(result.returncode)+'\n')
        record=dict(repository=name,kind=kind,command=command,returncode=result.returncode,elapsed_seconds=time.monotonic()-start,log=stem+'.log'); records.append(record)
        print(stem,result.returncode,round(record['elapsed_seconds'],2),flush=True)
        assert result.returncode==0,(audit/(stem+'.log')).read_text()[-6000:]
for row in manifest: assert hashlib.sha256((root/row['file']).read_bytes()).hexdigest()==row['sha256'],row['file']
for row in working: assert hashlib.sha256((workspace/row['file']).read_bytes()).hexdigest()==row['sha256'],row['file']
(audit/(prefix+'-builds.json')).write_text(json.dumps(records,indent=2)+'\n')
print('builds complete; both source manifests unchanged',flush=True)
