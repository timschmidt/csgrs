from pathlib import Path
import hashlib,json,os,signal,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent;prefix='fillet-tuple-replay-stack-20260926-v287';prior=json.loads((A/'selected-tuple-replay-20260926-v286-terminal.json').read_text());assert prior['qualification_complete'] and prior['all_processes_reaped']
manifest=json.loads((A/prior['source_manifest']).read_text());artifact=next(b['binary']for b in prior['builds']if b['binary']['path'].endswith('-hypercurve_curve_region_promotion'));binary=Path(artifact['path']);archive=Path(prior['source_directory']);driver_sha=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
def verify():
 for name,sha in manifest.items():
  for root in [W,archive,A/'build-workspace-20260925']:assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,name
 assert hashlib.sha256(binary.read_bytes()).hexdigest()==artifact['sha256']
 assert hashlib.sha256(Path(__file__).read_bytes()).hexdigest()==driver_sha
cmd=['/usr/bin/gdb','--nx','--quiet','--batch']
for instruction in ['set pagination off','set confirm off','set debuginfod enabled off','set print frame-arguments none','set print elements 8','set substitute-path '+str(A/'build-workspace-20260925')+' '+str(archive),'run','thread apply all bt 90','thread apply all bt -45','kill','quit']:cmd+=['-ex',instruction]
cmd+=['--args',str(binary),'--exact','approximate_512_trim_or_extend_analytic_parallel_support_corners_retain_algebraic_fillet_extensions','--nocapture','--test-threads=1']
report=dict(command=cmd,source_manifest=prior['source_manifest'],binary=artifact,driver_sha256=driver_sha,all_processes_reaped=False);verify();log=A/f'{prefix}.log';start=time.monotonic();owned=[]
with log.open('w')as out:
 process=subprocess.Popen(cmd,cwd=archive/'hypercurve',stdout=out,stderr=subprocess.STDOUT)
 try:process.wait(timeout=35);report['interrupt_sent']=False
 except subprocess.TimeoutExpired:
  children=Path(f'/proc/{process.pid}/task/{process.pid}/children');owned=[int(x)for x in children.read_text().split()] if children.exists() else [];report['owned_inferiors']=owned
  process.send_signal(signal.SIGINT);report['interrupt_sent']=True
  try:process.wait(timeout=20)
  except subprocess.TimeoutExpired:
   for pid in owned:
    try:os.kill(pid,signal.SIGKILL)
    except ProcessLookupError:pass
   process.kill();process.wait();report['forced_debugger_termination']=True
 report['returncode']=process.returncode
verify();report['all_processes_reaped']=all(not Path(f'/proc/{pid}').exists()for pid in owned);report['elapsed_seconds']=time.monotonic()-start;contents=log.read_text();report['stack_captured']='received signal SIGINT' in contents
(A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n');print('stack_captured',report['stack_captured'],'debugger_returncode',report['returncode'],'all_processes_reaped',report['all_processes_reaped'],flush=True)
frames=[s for s in contents.splitlines()if s.startswith('#')and any(x in s for x in ['hypercurve::','hypersolve::','hyperlimit::'])]
for line in frames[-38:]:print(line[:700],flush=True)
assert report['stack_captured'] and report['all_processes_reaped']
