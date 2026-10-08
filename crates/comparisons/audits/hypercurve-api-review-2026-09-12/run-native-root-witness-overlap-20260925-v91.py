from pathlib import Path
import hashlib,json,subprocess,time
A=Path(__file__).resolve().parent
W=A.parent
prefix='native-root-witness-overlap-20260925-v91'
source=A/(prefix+'.rs')
exe=A/prefix
manifest=json.loads((A/'local-chord-complete-replay-20260924-v91-sources.json').read_text())
root=A/'source-archives/hypercurve-local-chord-complete-replay-v91-20260924'
terminal=json.loads((A/'selected-frame-import-20260925-v91-terminal.json').read_text())
assert terminal['all_processes_reaped'] and terminal['all_sources_unchanged']
artifacts=[json.loads(line) for line in (A/'selected-frame-import-20260925-v91-build.jsonl').read_text().splitlines()]
hashfile=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
libs={name:Path(next(f for row in artifacts if row.get('reason')=='compiler-artifact' and row['target']['name']==name and not row['profile']['test'] for f in row['filenames'] if f.endswith('.rlib'))) for name in ['hypercurve','hyperreal']}
inputs={str(source):hashfile(source)}
for row in artifacts:
 if row.get('reason')=='compiler-artifact':
  for name in row['filenames']:
   path=Path(name)
   if path.is_file(): inputs[name]=hashfile(path)
def verify():
 for name,sha in manifest.items():
  assert hashfile(W/name)==sha,name
  assert hashfile(root/name)==sha,name
 for name,sha in inputs.items(): assert hashfile(Path(name))==sha,name
verify()
command=['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustc','--edition=2024','-O',str(source),'-L','dependency='+str(libs['hypercurve'].parent),'-o',str(exe)]
for name,path in libs.items():command.extend(['--extern',name+'='+str(path)])
with (A/(prefix+'-compile.log')).open('w') as log:
 code=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,timeout=120).returncode
verify()
assert code==0,(A/(prefix+'-compile.log')).read_text()[-3000:]
record=dict(source_inputs=inputs,workspace_manifest='local-chord-complete-replay-20260924-v91-sources.json',compile_command=command,compile_returncode=code,executable=str(exe),executable_sha256=hashfile(exe))
start=time.monotonic()
with (A/(prefix+'.log')).open('w') as log:
 try:code=subprocess.run([str(exe)],stdout=log,stderr=subprocess.STDOUT,timeout=75).returncode
 except subprocess.TimeoutExpired:code='timeout'
verify()
record.update(returncode=code,elapsed_seconds=time.monotonic()-start,all_processes_reaped=True,all_inputs_unchanged=True)
(A/(prefix+'-terminal.json')).write_text(json.dumps(record,indent=2)+'\n')
print('compile',record['compile_returncode'],'probe',code,'seconds',record['elapsed_seconds'],flush=True)
print((A/(prefix+'.log')).read_text()[-14000:],flush=True)
