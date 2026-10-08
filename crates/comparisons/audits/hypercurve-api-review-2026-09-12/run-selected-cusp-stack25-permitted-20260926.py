from pathlib import Path
import hashlib, json, os, signal, subprocess, time
A=Path(__file__).resolve().parent
prefix='selected-cusp-20260926-v155-stack25-permitted'
binary=A/'selected-cusp-arrangement-20260926-v155-hypercurve'
sha=hashlib.sha256(binary.read_bytes()).hexdigest()
command=['/usr/bin/gdb','-nx','-nh','--batch']
for option in ['set pagination off','set confirm off','set debuginfod enabled off','set print frame-arguments none','set print entry-values no','run','thread apply all bt -50','kill','quit']:
 command.extend(['-ex',option])
command.extend(['--args',str(binary),'--exact','curve_region_boolean::certified_successor_tests::selected_parallel_arrangement_splits_interior_cusps','--test-threads=1','--nocapture'])
manifest=json.loads((A/'selected-cusp-arrangement-20260926-v155-sources.json').read_text())
roots=[A.parent,A/'source-archives'/'selected-cusp-arrangement-20260926-v155',A/'build-workspace-20260925']
def verify():
 for name,digest in manifest.items():
  for root in roots:
   assert hashlib.sha256((root/name).read_bytes()).hexdigest()==digest,name
 assert hashlib.sha256(binary.read_bytes()).hexdigest()==sha
verify()
start=time.monotonic(); interrupted=False
with (A/f'{prefix}.log').open('w') as out:
 p=subprocess.Popen(command,stdout=out,stderr=subprocess.STDOUT,start_new_session=True)
 try: code=p.wait(timeout=25)
 except subprocess.TimeoutExpired:
  interrupted=True
  os.killpg(p.pid,signal.SIGINT)
  try: code=p.wait(timeout=20)
  except subprocess.TimeoutExpired:
   os.killpg(p.pid,signal.SIGKILL);p.wait();raise
verify()
report=dict(sources_unchanged=True,command=command,returncode=code,elapsed_seconds=time.monotonic()-start,interrupted=interrupted,binary_sha256=sha,debugger_reaped=True)
(A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
print(report)
print((A/f'{prefix}.log').read_text()[-22000:])
