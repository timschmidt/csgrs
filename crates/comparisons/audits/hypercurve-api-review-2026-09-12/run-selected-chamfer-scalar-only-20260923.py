from pathlib import Path
import hashlib, json, os, subprocess

A=Path(__file__).resolve().parent
prefix='selected-chamfer-scalar-20260923'
terminal=json.loads((A/f'{prefix}-probe1-terminal.json').read_text())
assert terminal['all_processes_reaped'] and terminal['all_sources_unchanged']
rows=[json.loads(line) for line in (A/f'{prefix}-probe1-build.jsonl').read_text().splitlines()]
libraries=[row for row in rows if row.get('reason')=='compiler-artifact' and row['target']['name']=='hyperreal' and 'lib' in row['target']['kind']]
assert len(libraries)==1
artifact=libraries[0]
assert 'hypercurve-selected-chamfer-scalar-replay-2026-09-23/hyperreal' in artifact['package_id']
library=next(Path(name) for name in artifact['filenames'] if name.endswith('.rlib'))
source=A/'selected-chamfer-scalar-replay.rs'
fixture=A/f'{prefix}-replay.json'
assert fixture.stat().st_size<=1024*1024
inputs={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [source,fixture,library]}
binary=A/f'{prefix}-standalone'
command=['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustc','--edition=2024','--crate-name','selected_chamfer_scalar_replay','-C','opt-level=3',str(source),'--extern',f'hyperreal={library}','-L',f'dependency={library.parent}','-o',str(binary)]
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
with (A/f'{prefix}-standalone-build.log').open('w') as out:
    code=subprocess.run(command,cwd=A,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=180).returncode
assert code==0,(A/f'{prefix}-standalone-build.log').read_text()[-2000:]
with (A/f'{prefix}-standalone.log').open('w') as out:
    try: code=subprocess.run([str(binary),str(fixture)],cwd=A,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=60).returncode
    except subprocess.TimeoutExpired: code='timeout'
for path,sha in inputs.items(): assert hashlib.sha256(Path(path).read_bytes()).hexdigest()==sha,path
report=dict(inputs=inputs,library_package=artifact['package_id'],command=command,returncode=code,binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),all_processes_reaped=True,diagnostic_only=True)
(A/f'{prefix}-standalone.json').write_text(json.dumps(report,indent=2)+'\n')
print((A/f'{prefix}-standalone.log').read_text())
