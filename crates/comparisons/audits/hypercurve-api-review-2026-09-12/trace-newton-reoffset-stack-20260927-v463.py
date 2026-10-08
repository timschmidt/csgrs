from pathlib import Path
import hashlib,json,os,shutil,signal,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent;prefix='newton-reoffset-stack-20260927-v463';qualified='owned-root-newton-reoffset-20260927-v461'
r=json.loads((A/f'{qualified}-terminal.json').read_text());assert r['all_processes_reaped'] and any(c['returncode']==124 for c in r['cases'])
sealed=json.loads((A/'normalized-root-replay-20260927-v454-committed.json').read_text());assert sealed['clean_repositories']==30
manifest=json.loads((A/r['source_manifest']).read_text());archive=Path(r['source_directory']);build=A/'build-workspace-20260925'
selected=next(b for b in r['builds'] if '--test' in b['command'] and b['command'][b['command'].index('--test')+1]=='hypercurve_curve_region_promotion');binary=Path(selected['binary']['path'])
def verify():
 for name,sha in manifest.items():
  for root in [W,archive,build]:assert hashlib.sha256((root/name).read_bytes()).hexdigest()==(sha if root==W else r['trial'].get(name,sha)),(root,name)
 assert hashlib.sha256(binary.read_bytes()).hexdigest()==selected['binary']['sha256']
verify()
command=['/usr/bin/gdb','--nx','--quiet','--batch','-ex','set pagination off','-ex','set confirm off','-ex','set debuginfod enabled off','-ex','set print frame-arguments none','-ex','set print elements 0','-ex','run','-ex','thread apply all bt 90','-ex','thread apply all bt -50','-ex','kill','-ex','quit','--args',str(binary),'--exact','strict_trim_or_extend_analytic_parallel_support_corners_retain_algebraic_fillet_extensions','--nocapture','--test-threads=1']
report=dict(source_manifest=r['source_manifest'],source_directory=str(archive),binary=selected['binary'],command=command,all_processes_reaped=False,interrupt_sent=False)
start=time.monotonic();log=A/f'{prefix}.log'
with log.open('w')as out:
 process=subprocess.Popen(command,cwd=archive/'hypercurve',stdout=out,stderr=subprocess.STDOUT,env=dict(os.environ,DEBUGINFOD_URLS='',XDG_CACHE_HOME='/tmp/hypercurve-gdb-cache'),start_new_session=True)
 try:
  try:process.wait(timeout=240)
  except subprocess.TimeoutExpired:
   report['interrupt_sent']=True;process.send_signal(signal.SIGINT)
   try:process.wait(timeout=15)
   except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
 except BaseException:
  os.killpg(process.pid,signal.SIGKILL);process.wait();raise
verify();report.update(returncode=process.returncode,elapsed_seconds=time.monotonic()-start,all_processes_reaped=True,stack_captured='#0 ' in log.read_text())
(A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n');print({k:report[k]for k in ['returncode','elapsed_seconds','all_processes_reaped','stack_captured']},flush=True)
raise SystemExit(0 if report['stack_captured'] else 1)
