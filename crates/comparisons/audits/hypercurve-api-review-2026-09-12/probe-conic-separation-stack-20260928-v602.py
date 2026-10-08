from pathlib import Path
import hashlib,json,os,signal,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent;prefix='conic-separation-stack-20260928-v602'
old=json.loads((A/'common-point-incidence-broad-20260928-v593-terminal.json').read_text());assert old['qualification_complete'] and old['all_processes_reaped']
current=json.loads((A/'parameter-construction-20260928-v598-terminal.json').read_text());assert current['all_processes_reaped'] and not current['qualification_complete']
assert json.loads((A/'parameter-construction-20260928-v598-reaped.json').read_text())['outer_exit_code']==1
manifest=json.loads((A/current['source_manifest']).read_text());baseline=json.loads((A/old['source_manifest']).read_text());binary=old['builds'][0]['binaries']['hypercurve'];name='rational_bezier_general::tests::conic_rational_image_separation_refines_past_the_old_limit'
assert hashlib.sha256(Path(binary['path']).read_bytes()).hexdigest()==binary['sha256']
def verify():
 for n,sha in manifest.items():
  for root in [W,Path(current['source_directory']),A/'build-workspace-20260925']:assert hashlib.sha256((root/n).read_bytes()).hexdigest()==sha,(root,n)
 for n,sha in baseline.items():assert hashlib.sha256((Path(old['source_directory'])/n).read_bytes()).hexdigest()==sha,n
verify();env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()));log=A/f'{prefix}.log';command=[binary['path'],'--exact',name,'--nocapture','--test-threads=1'];start=time.monotonic()
report=dict(baseline_qualification='common-point-incidence-broad-20260928-v593-terminal.json',source_manifest=old['source_manifest'],production_source_manifest=current['source_manifest'],binary=binary,command=command,cwd=str(Path(old['source_directory'])/'hypercurve'),all_processes_reaped=False)
def save():(A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
with log.open('w')as out:
 process=subprocess.Popen(command,cwd=report['cwd'],stdout=out,stderr=subprocess.STDOUT,env=env,start_new_session=True);report['pid']=process.pid;save();print('Baseline process',process.pid,flush=True)
 try:code=process.wait(timeout=120)
 except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait();code=124
 except BaseException:os.killpg(process.pid,signal.SIGKILL);process.wait();raise
report.update(returncode=code,elapsed_seconds=time.monotonic()-start,log=log.name,passed=code==0 and 'test result: ok. 1 passed;'in log.read_text(),all_processes_reaped=True);verify();save();print('Baseline result',code,round(report['elapsed_seconds'],3),flush=True)
raise SystemExit(0 if report['passed']else 1)
