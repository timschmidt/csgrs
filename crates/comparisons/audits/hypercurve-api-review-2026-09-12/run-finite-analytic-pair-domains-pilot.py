from pathlib import Path
import hashlib,json,subprocess,time
p=Path('.').resolve();prefix='finite-analytic-pair-domains-pilot';items=[json.loads(l) for l in (p/(prefix+'-test-build.jsonl')).read_text().splitlines()]
libtest=Path(next(x['executable'] for x in items if x.get('reason')=='compiler-artifact' and x['target']['name']=='hypercurve' and x.get('executable')))
library=Path(next(f for x in items if x.get('reason')=='compiler-artifact' and x['target']['name']=='hypercurve' and not x['profile']['test'] for f in x['filenames'] if f.endswith('.rlib')))
records=[]
for name in ['finite_analytic_pairs_replay_transverse_and_tangent_contacts','finite_analytic_pairs_retain_components_in_the_original_charts']:
 command=[str(libtest),'--exact','curve_intersection::curve_support_intersection::analytic_dispatch_tests::'+name,'--test-threads=1','--nocapture'];start=time.monotonic();log=p/(prefix+'-'+name+'.log')
 with log.open('w') as out:
  try:code=subprocess.run(command,stdout=out,stderr=subprocess.STDOUT,timeout=120).returncode
  except subprocess.TimeoutExpired:code=124
 record=dict(name=name,command=command,returncode=code,elapsed_seconds=time.monotonic()-start,binary_sha256=hashlib.sha256(libtest.read_bytes()).hexdigest(),log=log.name);records.append(record);print(record,flush=True)
 print(log.read_text()[-1800:],flush=True)
source=p/'finite-analytic-pair-domains-public.rs';stem=prefix+'-public'
command=['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustc','--edition=2024','-O',str(source),'--extern','hypercurve='+str(library),'-L','dependency='+str(library.parent),'-o',str(p/stem)]
r=subprocess.run(command,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=120);(p/(stem+'-compile.log')).write_text(r.stdout);assert r.returncode==0,r.stdout
start=time.monotonic()
with (p/(stem+'.log')).open('w') as out:
 try:code=subprocess.run([str(p/stem)],stdout=out,stderr=subprocess.STDOUT,timeout=120).returncode
 except subprocess.TimeoutExpired:code=124
record=dict(name='public',command=command,returncode=code,elapsed_seconds=time.monotonic()-start,source=source.name,source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),binary_sha256=hashlib.sha256((p/stem).read_bytes()).hexdigest(),normal_library_sha256=hashlib.sha256(library.read_bytes()).hexdigest(),log=stem+'.log');records.append(record);print(record,flush=True);print((p/(stem+'.log')).read_text()[-1600:],flush=True)
(p/(prefix+'-results.json')).write_text(json.dumps(records,indent=2)+'\n')
