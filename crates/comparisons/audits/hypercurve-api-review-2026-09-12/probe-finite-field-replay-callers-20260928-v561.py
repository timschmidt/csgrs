from pathlib import Path
import hashlib,json,os,signal,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent;prefix='finite-field-replay-callers-20260928-v561'
prior=json.loads((A/'finite-field-refinement-20260928-v559-terminal.json').read_text());assert prior['all_processes_reaped']
production=json.loads((A/'finite-field-refinement-20260928-v559-production-sources.json').read_text());manifest=json.loads((A/prior['source_manifest']).read_text());archive=Path(prior['source_directory'])
binary=prior['builds'][0]['binaries']['hypercurve_curve_region_promotion'];assert hashlib.sha256(Path(binary['path']).read_bytes()).hexdigest()==binary['sha256']
def verify():
 for name,sha in production.items():assert hashlib.sha256((W/name).read_bytes()).hexdigest()==sha,name
 for name,sha in manifest.items():assert hashlib.sha256((archive/name).read_bytes()).hexdigest()==sha,name
verify()
name='strict_trim_or_extend_analytic_parallel_support_corners_retain_algebraic_fillet_extensions'
report=dict(diagnostic_only=True,binary=binary,source_directory=str(archive),production_sources='finite-field-refinement-20260928-v559-production-sources.json',all_processes_reaped=False)
process=None;started=time.monotonic()
try:
 with (A/f'{prefix}-case.log').open('w') as output:
  process=subprocess.Popen([binary['path'],'--exact',name,'--nocapture','--test-threads=1'],cwd=archive/'hypercurve',stdout=output,stderr=subprocess.STDOUT,start_new_session=True)
  report['pid']=process.pid
  try:process.wait(timeout=15)
  except subprocess.TimeoutExpired:
   actual=Path(f'/proc/{process.pid}/cmdline').read_bytes().split(bytes([0]));assert actual[:3]==[binary['path'].encode(),b'--exact',name.encode()]
   cmd=['/usr/bin/gdb','-q','-nx','--batch','-iex','set auto-load off','-iex','set debuginfod enabled off','-iex','set sysroot /','--se='+binary['path'],'-ex','set pagination off','-ex','set print frame-arguments none','-ex','thread apply all bt -50','-ex','detach','-p',str(process.pid)]
   with (A/f'{prefix}-stack.log').open('w') as stack:
    result=subprocess.run(cmd,stdout=stack,stderr=subprocess.STDOUT,timeout=20)
   report['gdb_returncode']=result.returncode
finally:
 if process is not None:
  if process.poll() is None:os.killpg(process.pid,signal.SIGKILL)
  report['diagnostic_case_returncode']=process.wait()
 verify();report['all_processes_reaped']=True;report['elapsed_seconds']=time.monotonic()-started
 (A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
 print(json.dumps(report),flush=True)
