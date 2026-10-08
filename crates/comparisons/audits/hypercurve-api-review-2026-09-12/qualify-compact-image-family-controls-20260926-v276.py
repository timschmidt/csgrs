from pathlib import Path
import hashlib,json,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent;prefix='compact-image-family-controls-20260926-v276';prior=json.loads((A/'compact-image-coefficients-20260926-v275-terminal.json').read_text());assert prior['qualification_complete'] and prior['all_processes_reaped'];manifest=json.loads((A/prior['source_manifest']).read_text());binary=prior['builds'][2]['binary'];archive=Path(prior['source_directory']);build=A/'build-workspace-20260925'
report=dict(source_manifest=prior['source_manifest'],source_directory=prior['source_directory'],binary=binary,cases=[],all_processes_reaped=False)
def verify():
 for name,sha in manifest.items():
  for root in [W,archive,build]:assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,name
 assert hashlib.sha256(Path(binary['path']).read_bytes()).hexdigest()==binary['sha256']
verify();code=0
for name in ['curve::curve_fillet::tests::joined_path_selects_and_replays_a_continuous_fillet_family','curve::curve_fillet::tests::normalized_region_selects_and_reuses_a_continuous_fillet_family','curve::tests::distinct_parallel_sources_keep_a_shared_fillet_center_family']:
 log=A/f'{prefix}-{len(report["cases"])}.log';start=time.monotonic()
 with log.open('w')as out:
  try:rc=subprocess.run([binary['path'],'--exact',name,'--nocapture','--test-threads=1'],cwd=archive,stdout=out,stderr=subprocess.STDOUT,timeout=180).returncode
  except subprocess.TimeoutExpired:rc=124
 report['cases'].append(dict(name=name,returncode=rc,log=log.name,elapsed_seconds=time.monotonic()-start));verify();print(name,rc,round(time.monotonic()-start,2),flush=True)
 if rc!=0 or '1 passed; 0 failed' not in log.read_text():code=1;print(log.read_text()[-1600:],flush=True)
report['all_processes_reaped']=True;report['qualification_complete']=code==0;verify();(A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n');raise SystemExit(code)
