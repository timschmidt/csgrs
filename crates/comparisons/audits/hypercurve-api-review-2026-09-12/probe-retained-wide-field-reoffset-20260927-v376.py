from pathlib import Path
import hashlib,json,os,subprocess,signal,time
A=Path(__file__).resolve().parent;W=A.parent;prefix='retained-wide-field-reoffset-20260927-v376';qualified='retained-wide-field-proofs-20260927-v374'
qualification=json.loads((A/f'{qualified}-terminal.json').read_text());assert qualification['all_processes_reaped'] and qualification['qualification_complete']
manifest=json.loads((A/qualification['source_manifest']).read_text());build=next(b for b in qualification['builds'] if '--test' in b['command'] and b['command'][b['command'].index('--test')+1]=='hypercurve_curve_region_promotion');binary=Path(build['binary']['path'])
def verify():
 for name,sha in manifest.items():
  for root in [W,Path(qualification['source_directory']),A/'build-workspace-20260925']:assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,(root,name)
 assert hashlib.sha256(binary.read_bytes()).hexdigest()==build['binary']['sha256']
verify();env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()));report=dict(source_manifest=qualification['source_manifest'],source_directory=qualification['source_directory'],binary=build['binary'],cases=[],all_processes_reaped=False)
def save():(A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
for name in ['strict_trim_or_extend_analytic_parallel_support_corners_retain_algebraic_fillet_extensions','approximate_512_trim_or_extend_analytic_parallel_support_corners_retain_algebraic_fillet_extensions']:
 start=time.monotonic();log=A/f'{prefix}-{name}.log'
 with log.open('w')as out:
  process=subprocess.Popen([str(binary),'--exact',name,'--nocapture','--test-threads=1'],cwd=Path(qualification['source_directory'])/'hypercurve',env=env,stdout=out,stderr=subprocess.STDOUT,start_new_session=True)
  try:code=process.wait(timeout=240)
  except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait();code=124
  except BaseException:os.killpg(process.pid,signal.SIGKILL);process.wait();raise
 report['cases'].append(dict(name=name,returncode=code,elapsed_seconds=time.monotonic()-start,log=log.name));save();print(name,code,round(time.monotonic()-start,2),flush=True)
verify();report['all_sources_unchanged']=True;report['all_processes_reaped']=True;report['all_cases_passed']=all(c['returncode']==0 for c in report['cases']);save()
print('Recheck complete; individual case outcomes are recorded.',flush=True)
