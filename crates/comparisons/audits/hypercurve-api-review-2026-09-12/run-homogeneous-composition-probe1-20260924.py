from pathlib import Path
import hashlib,json,os,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent;prefix='homogeneous-composition-20260924-probe1'
root=Path('/tmp/hypercurve-direct-endpoints-v3-2026-09-24')
bindings=json.loads((A/'direct-endpoints-20260924-v3-sources.json').read_text())
src=A/f'{prefix}.rs';source_sha=hashlib.sha256(src.read_bytes()).hexdigest()
def verify():
    for name,sha in bindings.items():
        assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,name
        assert hashlib.sha256((W/name).read_bytes()).hexdigest()==sha,name
    assert hashlib.sha256(src.read_bytes()).hexdigest()==source_sha
verify()
rows=[json.loads(line) for line in (A/'direct-endpoints-20260924-v3-integrations-build.jsonl').read_text().splitlines()]
artifact=next(row for row in rows if row.get('reason')=='compiler-artifact' and row['target']['name']=='hypercurve' and row['target']['kind']==['lib'])
lib=Path(next(p for p in artifact['filenames'] if p.endswith('.rlib')))
binary=A/prefix
command=['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustc','--edition=2024','-O',str(src),'-L',f'dependency={lib.parent}','--extern',f'hypercurve={lib}','-o',str(binary)]
with (A/f'{prefix}-build.log').open('w') as out:code=subprocess.run(command,stdout=out,stderr=subprocess.STDOUT,timeout=120).returncode
assert code==0
start=time.monotonic();limit=150
with (A/f'{prefix}.log').open('w') as out:
    try:code=subprocess.run([str(binary)],cwd=root/'hypercurve',stdout=out,stderr=subprocess.STDOUT,timeout=limit).returncode
    except subprocess.TimeoutExpired:code='timeout'
verify()
report=dict(returncode=code,elapsed_seconds=time.monotonic()-start,limit_seconds=limit,source_sha256=source_sha,binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),library_sha256=hashlib.sha256(lib.read_bytes()).hexdigest(),all_sources_unchanged=True,all_processes_reaped=True)
(A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
print(report,flush=True)
print((A/f'{prefix}.log').read_text()[-5000:],flush=True)
