from pathlib import Path
import hashlib, json, os, signal, subprocess, time
A=Path(__file__).resolve().parent
prefix='conic-owned-contact-20260924-stack1'
binary=A/'conic-owned-contact-20260924-v3-libtest'
sha=hashlib.sha256(binary.read_bytes()).hexdigest()
command=['/usr/bin/gdb','-nx','-nh','--batch']
for option in ['set pagination off','set confirm off','set debuginfod enabled off','set print frame-arguments none','set print entry-values no','run','thread apply all bt -50','kill','quit']:
 command.extend(['-ex',option])
command.extend(['--args',str(binary),'--exact','bezier_region::tests::nonlinear_selected_corner_chamfers_through_retained_fixed_distance_image','--test-threads=1','--nocapture'])
start=time.monotonic(); interrupted=False
with (A/f'{prefix}.log').open('w') as out:
 p=subprocess.Popen(command,stdout=out,stderr=subprocess.STDOUT,start_new_session=True)
 try: code=p.wait(timeout=8)
 except subprocess.TimeoutExpired:
  interrupted=True
  os.killpg(p.pid,signal.SIGINT)
  try: code=p.wait(timeout=20)
  except subprocess.TimeoutExpired:
   os.killpg(p.pid,signal.SIGKILL);p.wait();raise
report=dict(command=command,returncode=code,elapsed_seconds=time.monotonic()-start,interrupted=interrupted,binary_sha256=sha,debugger_reaped=True)
(A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
print(report)
print((A/f'{prefix}.log').read_text()[-22000:])
