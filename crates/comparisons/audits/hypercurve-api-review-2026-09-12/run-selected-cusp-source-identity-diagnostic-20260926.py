from pathlib import Path
import hashlib, json, os, subprocess, time
A=Path(__file__).resolve().parent
W=A.parent
prefix='selected-cusp-20260926-v161-pair-diagnostic'
r=json.loads((A/'selected-cusp-arrangement-20260926-v161-terminal.json').read_text())
assert r['all_processes_reaped']
manifest=json.loads((A/r['source_manifest']).read_text())
binary=r['binaries']['hypercurve']
def verify():
 for name,sha in manifest.items():
  for root in [W,Path(r['source_directory']),Path(r['build_source_directory'])]:
   assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,name
 assert hashlib.sha256(Path(binary['path']).read_bytes()).hexdigest()==binary['sha256']
verify()
env=dict(os.environ,HYPERCURVE_DEBUG_RATIONAL_BLOCKER='1')
log=A/(prefix+'.log')
command=[binary['path'],'--exact','curve_region_boolean::certified_successor_tests::selected_parallel_arrangement_splits_interior_cusps','--test-threads=1','--nocapture','--color','never']
start=time.monotonic()
with log.open('w') as out:
 try: code=subprocess.run(command,cwd=Path(r['build_source_directory'])/'hypercurve',env=env,stdout=out,stderr=subprocess.STDOUT,timeout=90).returncode
 except subprocess.TimeoutExpired: code='timeout'
verify()
report=dict(source_manifest=r['source_manifest'],binary=binary,command=command,returncode=code,elapsed_seconds=time.monotonic()-start,log=log.name,all_sources_unchanged=True,all_processes_reaped=True,diagnostic_only=True)
(A/(prefix+'-terminal.json')).write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report),flush=True)
print(log.read_text()[-5000:],flush=True)
