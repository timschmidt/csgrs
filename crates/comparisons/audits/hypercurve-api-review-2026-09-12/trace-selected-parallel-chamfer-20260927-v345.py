from pathlib import Path
import hashlib,json,os,subprocess,signal,time
A=Path(__file__).resolve().parent;W=A.parent;prefix='selected-parallel-chamfer-20260927-v345';qualified='collapsed-selected-circle-contact-20260927-v343'
manifest=json.loads((A/f'{qualified}-sources.json').read_text());qualification=json.loads((A/f'{qualified}-terminal.json').read_text())
def verify():
 for name,sha in manifest.items():
  for root in [W,Path(qualification['source_directory']),A/'build-workspace-20260925']:
   assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,(root,name)
 binary=Path(qualification['builds'][0]['binary']['path']);assert hashlib.sha256(binary.read_bytes()).hexdigest()==qualification['builds'][0]['binary']['sha256']
 return binary
binary=verify();start=time.monotonic()
command=['gdb','--quiet','--batch','-ex','set pagination off','-ex','set confirm off','-ex','set debuginfod enabled off','-ex','run','-ex','thread apply all bt 55','-ex','quit','--args',str(binary),'--exact','bezier_region::tests::one_fragment_nonzero_parallel_loop_extends_chamfer_cuts_on_one_finite_envelope','--nocapture','--test-threads=1']
with (A/f'{prefix}.log').open('w') as log:
 process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,env=dict(os.environ,DEBUGINFOD_URLS='',XDG_CACHE_HOME='/tmp/collapsed-selected-circle-gdb-cache'))
 try:process.wait(timeout=30)
 except subprocess.TimeoutExpired:
  process.send_signal(signal.SIGINT)
  try:process.wait(timeout=15)
  except subprocess.TimeoutExpired:process.kill();process.wait()
verify();report=dict(source_manifest=f'{qualified}-sources.json',binary=qualification['builds'][0]['binary'],command=command,returncode=process.returncode,elapsed_seconds=time.monotonic()-start,all_processes_reaped=True)
(A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
print((A/f'{prefix}.log').read_text()[-18000:])
