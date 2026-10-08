from pathlib import Path
import hashlib,json,os,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent;prefix='retained-fillet-reoffset-20260927-v349';qualified='selected-generator-order-20260927-v348'
manifest=json.loads((A/f'{qualified}-sources.json').read_text());qualification=json.loads((A/f'{qualified}-terminal.json').read_text());assert qualification['qualification_complete'] and qualification['all_processes_reaped']
binary=next(b['binary'] for b in qualification['builds'] if '--test' in b['command'] and b['command'][b['command'].index('--test')+1]=='hypercurve_curve_region_promotion');case='strict_trim_or_extend_analytic_parallel_support_corners_retain_algebraic_fillet_extensions'
def verify():
 for name,sha in manifest.items():
  for root in [W,Path(qualification['source_directory']),A/'build-workspace-20260925']:
   assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,(root,name)
 assert hashlib.sha256(Path(binary['path']).read_bytes()).hexdigest()==binary['sha256']
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()));verify();start=time.monotonic();command=[binary['path'],'--exact',case,'--nocapture','--test-threads=1']
with (A/f'{prefix}.log').open('w') as log:
 try:code=subprocess.run(command,cwd=A,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=240).returncode
 except subprocess.TimeoutExpired:code=124
elapsed=time.monotonic()-start;verify();report=dict(source_manifest=f'{qualified}-sources.json',binary=binary,command=command,name=case,returncode=code,elapsed_seconds=elapsed,all_processes_reaped=True)
(A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2));print((A/f'{prefix}.log').read_text()[-2500:]);raise SystemExit(code)
