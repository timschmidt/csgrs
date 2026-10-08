from pathlib import Path
import hashlib, json, os, shutil, subprocess, time

workspace = Path('/home/tim/Documents/GitHub/workspace')
root = Path('/tmp/hypercurve-admission-qualification')
audit = workspace/'hypercurve-api-review-2026-09-12'
archive = audit/'finite-parallel-admission-baseline'
archive.mkdir(exist_ok=False)
shutil.copy2(workspace/'hypercurve/src/bezier_region.rs',root/'hypercurve/src/bezier_region.rs')
patch = subprocess.check_output(['git','diff'],cwd=workspace/'hypercurve')
(archive/'source.patch').write_bytes(patch)
manifest = []
for name in ['hypercurve','hyperbrep','hyperlattice','hyperlimit','hyperreal','hypersolve','hypertri']:
    for path in sorted((root/name).rglob('*')):
        if path.is_file() and 'target' not in path.parts:
            manifest.append(dict(file=str(path.relative_to(root)),sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
(archive/'sources.json').write_text(json.dumps(manifest,indent=2)+'\n')
toolchain=Path('/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin')
env=dict(os.environ,CARGO_BUILD_JOBS='2',RUSTC=str(toolchain/'rustc'),RUSTDOC=str(toolchain/'rustdoc'),CARGO_TARGET_DIR='/tmp/hypercurve-pruning-qualification/hyperbrep/target',CCACHE_DIR='/tmp/hypercurve-pruning-qualification/ccache',CCACHE_TEMPDIR='/tmp/hypercurve-pruning-qualification/ccache-tmp')
command=[str(toolchain/'cargo'),'test','--release','--all-features','--locked','--offline','--lib','--no-run','--message-format=json']
start=time.monotonic()
with (archive/'build.jsonl').open('w') as out,(archive/'build.log').open('w') as err:
    result=subprocess.run(command,cwd=root/'hypercurve',env=env,stdout=out,stderr=err,timeout=900)
report=dict(parent=subprocess.check_output(['git','rev-parse','HEAD'],cwd=workspace/'hypercurve',text=True).strip(),patch_sha256=hashlib.sha256(patch).hexdigest(),build_command=command,build_exit=result.returncode,build_elapsed_seconds=time.monotonic()-start)
(archive/'result.json').write_text(json.dumps(report,indent=2)+'\n')
print('baseline build',result.returncode,round(report['build_elapsed_seconds'],2),flush=True)
assert result.returncode==0,(archive/'build.log').read_text()[-5000:]
items=[json.loads(line) for line in (archive/'build.jsonl').read_text().splitlines()]
jobs=[v for v in items if v.get('reason')=='compiler-artifact' and v.get('executable') and v['profile']['test']]
assert len(jobs)==1
exe=Path(jobs[0]['executable']); shutil.copy2(exe,archive/exe.name)
report['binary_sha256']=hashlib.sha256(exe.read_bytes()).hexdigest(); report['tests']=[]
for name in ['retained_fragment_turn_certificates_own_the_consumed_range','zero_distance_parallel_admission_excludes_source_poles']:
    with (archive/(name+'.log')).open('w') as out:
        result=subprocess.run([str(exe),name,'--test-threads=1','--nocapture'],cwd=root/'hypercurve',stdout=out,stderr=subprocess.STDOUT,timeout=120)
    report['tests'].append(dict(name=name,returncode=result.returncode,log=name+'.log'))
    print(name,result.returncode,flush=True)
for row in manifest: assert hashlib.sha256((root/row['file']).read_bytes()).hexdigest()==row['sha256'],row['file']
(archive/'result.json').write_text(json.dumps(report,indent=2)+'\n')
print('baseline evidence complete; sources may be edited',flush=True)
