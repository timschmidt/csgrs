from pathlib import Path
import hashlib,json,os,subprocess,signal,time
A=Path(__file__).resolve().parent;W=A.parent;prefix='retained-fillet-reoffset-20260927-v351';qualified='selected-generator-order-20260927-v348'
manifest=json.loads((A/f'{qualified}-sources.json').read_text());qualification=json.loads((A/f'{qualified}-terminal.json').read_text())
selected_build=next(b for b in qualification['builds'] if '--test' in b['command'] and b['command'][b['command'].index('--test')+1]=='hypercurve_curve_region_promotion')
def verify():
 for name,sha in manifest.items():
  for root in [W,Path(qualification['source_directory']),A/'build-workspace-20260925']:
   assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,(root,name)
 binary=Path(selected_build['binary']['path']);assert hashlib.sha256(binary.read_bytes()).hexdigest()==selected_build['binary']['sha256']
 return binary
binary=verify();start=time.monotonic()
command=['gdb','--quiet','--batch','-ex','set pagination off','-ex','set confirm off','-ex','set debuginfod enabled off','-ex','run','-ex','thread apply all bt -65','-ex','quit','--args',str(binary),'--exact','strict_trim_or_extend_analytic_parallel_support_corners_retain_algebraic_fillet_extensions','--nocapture','--test-threads=1']
with (A/f'{prefix}.log').open('w') as log:
 process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,env=dict(os.environ,DEBUGINFOD_URLS='',XDG_CACHE_HOME='/tmp/collapsed-selected-circle-gdb-cache'))
 try:process.wait(timeout=30)
 except subprocess.TimeoutExpired:
  process.send_signal(signal.SIGINT)
  try:process.wait(timeout=15)
  except subprocess.TimeoutExpired:process.kill();process.wait()
verify();report=dict(source_manifest=f'{qualified}-sources.json',binary=selected_build['binary'],command=command,returncode=process.returncode,elapsed_seconds=time.monotonic()-start,all_processes_reaped=True)
(A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
print((A/f'{prefix}.log').read_text()[-18000:])
