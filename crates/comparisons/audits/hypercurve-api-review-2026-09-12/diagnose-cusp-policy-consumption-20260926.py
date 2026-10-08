from pathlib import Path
import hashlib,json,os,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent
r=json.loads((A/'circle-contact-evidence-20260926-v200-terminal.json').read_text());q=json.loads((A/'circle-endpoint-source-20260926-v202-terminal.json').read_text())
assert r['all_processes_reaped'] and q['all_processes_reaped']
prefix='circle-contact-evidence-20260926-v200-policy30'
manifest=json.loads((A/r['source_manifest']).read_text());build_manifest=json.loads((A/q['source_manifest']).read_text());binary=r['binaries']['hypercurve']
def verify():
 for n,h in manifest.items():
  for root in [W,Path(r['source_directory'])]:assert hashlib.sha256((root/n).read_bytes()).hexdigest()==h,n
 for n,h in build_manifest.items():assert hashlib.sha256((Path(q['build_source_directory'])/n).read_bytes()).hexdigest()==h,n
 assert hashlib.sha256(Path(binary['path']).read_bytes()).hexdigest()==binary['sha256']
verify();report=dict(source_manifest=r['source_manifest'],build_source_manifest=q['source_manifest'],binary=binary,diagnostic_only=True,all_processes_reaped=False)
assert not (A/(prefix+'-terminal.json')).exists()
command=[binary['path'],'--exact','curve_region_boolean::certified_successor_tests::selected_parallel_arrangement_splits_interior_cusps','--nocapture','--test-threads=1']
env=dict(os.environ,HYPERCURVE_DEBUG_APPROXIMATE_CONSUMPTION='1');start=time.monotonic()
with (A/(prefix+'.log')).open('w') as out:
 try:code=subprocess.run(command,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=30).returncode
 except subprocess.TimeoutExpired:code='timeout'
verify();report.update(command=command,returncode=code,elapsed_seconds=time.monotonic()-start,all_processes_reaped=True,all_sources_unchanged=True)
(A/(prefix+'-terminal.json')).write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report),flush=True);print((A/(prefix+'.log')).read_text()[-7000:],flush=True)
