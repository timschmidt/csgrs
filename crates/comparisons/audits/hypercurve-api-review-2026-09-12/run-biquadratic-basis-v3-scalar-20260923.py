from pathlib import Path
import hashlib,json,os,subprocess

A=Path(__file__).resolve().parent
root=Path('/tmp/hypercurve-biquadratic-basis-v3-2026-09-23');repo=root/'hyperreal'
prefix='biquadratic-basis-20260923-v3-scalar-attempt2'
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
bindings=json.loads((A/'biquadratic-basis-20260923-v3-sources.json').read_text())
def verify():
    for name,sha in bindings.items():assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,name
verify()
command=['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo','build','--lib','--release','--all-features','--message-format=json','--locked','--offline']
with (A/f'{prefix}-build.jsonl').open('w') as out,(A/f'{prefix}-build.log').open('w') as err:
    code=subprocess.run(command,cwd=repo,env=env,stdout=out,stderr=err,timeout=600).returncode
verify();assert code==0
artifacts=[json.loads(line) for line in (A/f'{prefix}-build.jsonl').read_text().splitlines()]
artifact=next(row for row in artifacts if row.get('reason')=='compiler-artifact' and row['target']['name']=='hyperreal')
assert str(repo) in artifact['package_id']
library=next(Path(name) for name in artifact['filenames'] if name.endswith('.rlib'))
report=dict(library_package=artifact['package_id'],library_sha256=hashlib.sha256(library.read_bytes()).hexdigest(),cases=[])
for label,filename in [('sum','biquadratic-parent-probe-20260923.rs'),('shared-zero','selected-chamfer-scalar-shared-20260923.rs')]:
    source=A/filename;binary=A/f'{prefix}-{label}';source_sha=hashlib.sha256(source.read_bytes()).hexdigest()
    command=['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustc','--edition=2024','--crate-name',label.replace('-','_'),'-C','opt-level=3',str(source),'--extern',f'hyperreal={library}','-L',f'dependency={library.parent}', '-L', f'dependency={library.parent / "deps"}','-o',str(binary)]
    subprocess.run(command,check=True,cwd=A,env=env,timeout=120)
    result=subprocess.run([str(binary)],capture_output=True,text=True,cwd=A,env=env,timeout=30)
    log=A/f'{prefix}-{label}.log';log.write_text(result.stdout+result.stderr)
    assert hashlib.sha256(source.read_bytes()).hexdigest()==source_sha
    report['cases'].append(dict(label=label,source_sha256=source_sha,returncode=result.returncode,binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),log=log.name))
    print(label,result.returncode,result.stdout+result.stderr,flush=True)
verify();report.update(all_processes_reaped=True,all_sources_unchanged=True)
(A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
