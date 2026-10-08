from pathlib import Path
import hashlib,json,os,signal,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent;prefix='fillet-replay-stack-20260926-v253';prior=json.loads((A/'fillet-replay-diagnostic-20260926-v249-terminal.json').read_text());manifest=json.loads((A/prior['source_manifest']).read_text());guard={name:sha for name,sha in json.loads((A/prior['workspace_guard']).read_text()).items()if not name.startswith('fiber-probe/')};binary=Path(prior['binary']['path']);archive=Path(prior['source_directory'])
def verify():
 for name,sha in guard.items():assert hashlib.sha256((W/name).read_bytes()).hexdigest()==sha,name
 for name,sha in manifest.items():assert hashlib.sha256((archive/name).read_bytes()).hexdigest()==sha,name
 assert hashlib.sha256(binary.read_bytes()).hexdigest()==prior['binary']['sha256']
cmd=['/usr/bin/gdb','--nx','--quiet','--batch']
for instruction in ['set pagination off','set confirm off','set debuginfod enabled off','set print frame-arguments none','set print elements 8','set substitute-path '+str(A/'build-workspace-20260925')+' '+str(archive),'run','thread apply all bt 20','thread apply all bt -30','kill','quit']:
 cmd+=['-ex',instruction]
cmd+=['--args',str(binary),'--exact','approximate_512_trim_or_extend_analytic_parallel_support_corners_retain_algebraic_fillet_extensions','--nocapture','--test-threads=1']
report=dict(command=cmd,source_manifest=prior['source_manifest'],workspace_guard=prior['workspace_guard'],binary=prior['binary'],all_processes_reaped=False);verify();log=A/f'{prefix}.log';start=time.monotonic()
with log.open('w')as out:
 process=subprocess.Popen(cmd,cwd=archive/'hypercurve',stdout=out,stderr=subprocess.STDOUT)
 try:process.wait(timeout=20);report['interrupt_sent']=False
 except subprocess.TimeoutExpired:
  process.send_signal(signal.SIGINT);report['interrupt_sent']=True
  try:process.wait(timeout=30)
  except subprocess.TimeoutExpired:
   process.kill();process.wait();report['forced_debugger_termination']=True
 report['returncode']=process.returncode
verify();report['all_processes_reaped']=True;report['elapsed_seconds']=time.monotonic()-start;report['stack_captured']='received signal SIGINT' in log.read_text();(A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
print('stack_captured',report['stack_captured'],'debugger_returncode',report['returncode'],'elapsed',round(report['elapsed_seconds'],1),flush=True)
print(log.read_text()[-18000:],flush=True)
