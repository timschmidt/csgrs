import hashlib,json,os,subprocess,sys,time
from pathlib import Path
source=Path('docs/evidence/2026-10-09-hyperdrc-native-linear-strokes/native-area-compaction-gate/sources.json')
out=Path(sys.argv[1]).resolve();out.mkdir(parents=True,exist_ok=False)
record=json.loads(source.read_text())
def verify():
 assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==h for p,h in record['source_hashes'].items())
verify()
overrides={'CARGO_HOME':'/tmp/csgrs-properties-cargo','CCACHE_DIR':'/tmp/csgrs-properties-ccache','CARGO_NET_OFFLINE':'true','CARGO_TARGET_DIR':str(Path.cwd()/'target/native-area-profile'),'RUSTFLAGS':'-Cforce-frame-pointers=yes','CARGO_PROFILE_RELEASE_DEBUG':'1','DEBUGINFOD_URLS':''}
cache=Path(overrides['CARGO_TARGET_DIR'])
assert not cache.exists()
cache.mkdir(parents=True)
with (out/'copy-build-cache.log').open('w') as log:
 subprocess.run(['cp','-a','--reflink=auto','/tmp/drc-native-frame-pointer-target/.',str(cache)],stdout=log,stderr=subprocess.STDOUT,check=True)
verify()
command=['cargo' ,'test','--locked','-p','hyperdrc','--release','--lib','--no-run','--message-format=json']
start=time.monotonic()
with (out/'build.log').open('w') as log:code=subprocess.run(command,env={**os.environ,**overrides},stdout=log,stderr=subprocess.STDOUT).returncode
verify()
executables=[]
for line in (out/'build.log').read_text().splitlines():
 try:a=json.loads(line)
 except ValueError:continue
 if a.get('reason')=='compiler-artifact' and a.get('target',{}).get('name')=='hyperdrc' and a.get('executable'):executables.append(a['executable'])
assert code==0 and len(executables)==1
binary=Path(executables[0])
(out/'reproduce_build.py').write_text(Path(__file__).read_text())
(out/'build.json').write_text(json.dumps(dict(source_reference=str(source),source_unchanged=True,environment=overrides,command=command,exit_code=code,wall_seconds=time.monotonic()-start,binary=str(binary),binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),note='Profiling-only build with frame pointers and line tables. Its timing is not release admission evidence.'),indent=2)+'\n')
print('frame-pointer build complete',flush=True)
