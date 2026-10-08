from pathlib import Path
import hashlib,json,os,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent;prefix='collapsed-selected-circle-contact-20260927-v344';qualified='collapsed-selected-circle-contact-20260927-v343'
manifest=json.loads((A/f'{qualified}-sources.json').read_text());qualification=json.loads((A/f'{qualified}-terminal.json').read_text());assert qualification['qualification_complete'] and qualification['all_processes_reaped']
binary=qualification['builds'][0]['binary'];case='bezier_region::tests::one_fragment_nonzero_parallel_loop_extends_chamfer_cuts_on_one_finite_envelope'
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
