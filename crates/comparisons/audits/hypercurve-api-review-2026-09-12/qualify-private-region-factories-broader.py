from pathlib import Path
import hashlib,json,os,shutil,subprocess,time,concurrent.futures
w=Path('/home/tim/Documents/GitHub/workspace'); a=Path(__file__).resolve().parent; r=Path('/tmp/hypercurve-region-admission-qualification')
prefix='private-region-factories-broader1'
bound='private-region-factories-candidate3'
working=json.loads((a/(bound+'-working-sources.json')).read_text()); manifest=json.loads((a/(bound+'-isolated-sources.json')).read_text())
settings=json.loads((a/'opposed-endpoint-contact-full1-build-settings.json').read_text()); env=dict(os.environ,**settings)
cargo='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
archive=a/(prefix+'-libraries'); archive.mkdir()
def verify():
 for base,rows in [(w,working),(r,manifest)]:
  for row in rows: assert hashlib.sha256((base/row['file']).read_bytes()).hexdigest()==row['sha256'],row['file']
verify()
targets=['hypercurve_curve_region_promotion','hypercurve_curve_range','hypercurve_path_closure','hypercurve_curve_intersection']
cmd=[cargo,'test','--release','--all-features','--lib']
for target in targets: cmd+=['--test',target]
cmd+=['--no-run','--message-format=json','--locked','--offline']
start=time.monotonic()
with (a/(prefix+'-build.jsonl')).open('w') as out,(a/(prefix+'-build.log')).open('w') as err:
 result=subprocess.run(cmd,cwd=r/'hypercurve',env=env,stdout=out,stderr=err,timeout=900)
(a/(prefix+'-build.json')).write_text(json.dumps(dict(command=cmd,returncode=result.returncode,elapsed_seconds=time.monotonic()-start),indent=2)+'\n')
print('build',result.returncode,round(time.monotonic()-start,2),flush=True)
assert result.returncode==0,(a/(prefix+'-build.log')).read_text()[-5000:]
binaries={}
for line in (a/(prefix+'-build.jsonl')).read_text().splitlines():
 item=json.loads(line)
 if item.get('reason')=='compiler-artifact' and item.get('executable'):
  binary=archive/Path(item['executable']).name; shutil.copyfile(item['executable'],binary); binary.chmod(0o755); binaries[item['target']['name']]=binary
unit=binaries['hypercurve']
prior_units=list((a/(bound+'-libraries')).glob('hypercurve-*'))
assert len(prior_units)==1
assert hashlib.sha256(unit.read_bytes()).hexdigest()==hashlib.sha256(prior_units[0].read_bytes()).hexdigest()
held=['selected_parallel_normal_circle_intersects_genuinely_analytic_parallel_in_one_fiber','independent_oblique_chord_pair_fillets_extend_on_infinite_supports','selected_circle_and_analytic_parallel_extend_on_full_supports','pair_native_boolean_algebraic_chord_corner_publishes_a_third_generation_fillet']
known=json.loads((a/'normalized-promotion-region-qualification.json').read_text())['baseline_failure_comparison']
known_names=[row['name'] for row in known]
runs=[]
def run(target):
 binary=binaries[target]; cmd=[str(binary),'--test-threads=4','--color','never']
 skips=held if target=='hypercurve' else known_names if target=='hypercurve_curve_region_promotion' else []
 for name in skips: cmd+=['--skip',name]
 listing=subprocess.check_output([str(binary),'--list'],cwd=r/'hypercurve',text=True)
 listed=[line.removesuffix(': test') for line in listing.splitlines() if line.endswith(': test')]
 for name in skips: assert any(name in full for full in listed),name
 logname=prefix+'-'+target+'.log'; start=time.monotonic()
 with (a/logname).open('w') as log:
  try: code=subprocess.run(cmd,cwd=r/'hypercurve',stdout=log,stderr=subprocess.STDOUT,timeout=300).returncode
  except subprocess.TimeoutExpired: code='timeout'
 text=(a/logname).read_text()
 row=dict(target=target,command=cmd,skipped=skips,returncode=code,elapsed_seconds=time.monotonic()-start,sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),log=logname,summary=next((line for line in text.splitlines() if line.startswith('test result:')),None))
 print(target,code,round(row['elapsed_seconds'],2),row['summary'],flush=True)
 if code: print(text[-6000:],flush=True)
 return row
try:
 with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
  for row in pool.map(run,['hypercurve',*targets]):
   runs.append(row); (a/(prefix+'-runs.json')).write_text(json.dumps(runs,indent=2)+'\n')
 for entry in known:
  name=entry['name']; binary=binaries['hypercurve_curve_region_promotion']
  cmd=[str(binary),'--exact',name,'--test-threads=1','--nocapture','--color','never']
  logname=prefix+'-known-'+name+'.log'; start=time.monotonic()
  with (a/logname).open('w') as log:
   try: code=subprocess.run(cmd,cwd=r/'hypercurve',stdout=log,stderr=subprocess.STDOUT,timeout=300).returncode
   except subprocess.TimeoutExpired: code='timeout'
  output=(a/logname).read_text()
  def panic(text): return text.split('panicked at ',1)[1].split('\n',1)[1].split('note: run with',1)[0]
  same=code==101 and panic(output)==panic((a/entry['candidate']['log']).read_text())
  row=dict(target=name,command=cmd,returncode=code,known_failure_matches=same,elapsed_seconds=time.monotonic()-start,sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),log=logname)
  runs.append(row); (a/(prefix+'-runs.json')).write_text(json.dumps(runs,indent=2)+'\n')
  print('known failure',name,code,'matches',same,flush=True)
finally:
 verify(); print('Bound sources unchanged.',flush=True)
raise SystemExit(1 if any(row['returncode']!=0 and not row.get('known_failure_matches') for row in runs) else 0)
