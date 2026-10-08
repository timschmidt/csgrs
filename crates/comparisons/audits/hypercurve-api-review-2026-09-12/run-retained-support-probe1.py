from pathlib import Path
import subprocess,json,hashlib,time,concurrent.futures
root=Path.cwd(); audit=root.parent/'hypercurve-api-review-2026-09-12'
manifest=json.loads((audit/'retained-support-intersection-source1.json').read_text())
artifacts=[json.loads(l) for l in (audit/'retained-support-intersection-build1.jsonl').read_text().splitlines()]
binary=next(x['executable'] for x in artifacts if x.get('executable') and x['target']['name']=='hypercurve_curve_intersection')
def lib(name):
 return next(Path(f) for x in artifacts if x.get('reason')=='compiler-artifact' and x['target']['name']==name for f in x['filenames'] if f.endswith('.rlib'))
rlib=lib('hypercurve'); probe=audit/'retained-support-intersection-generated-pairs-probe1'
subprocess.run(['rustc','--edition=2024','-O',str(audit/'parabola-chamfer-independent-pairs-probe.rs'),'-L',f'dependency={root}/target/release/deps','--extern',f'hypercurve={rlib}','--extern',f'hyperreal={lib("hyperreal")}','-o',str(probe)],check=True)
def run(label,command):
 start=time.monotonic()
 with (audit/(label+'.log')).open('w') as out:
  try: code=subprocess.run(command,stdout=out,stderr=subprocess.STDOUT,timeout=180).returncode
  except subprocess.TimeoutExpired: code=124
 log=(audit/(label+'.log')).read_text()
 record={'returncode':code,'seconds':time.monotonic()-start,'command':command,'rlib_sha256':hashlib.sha256(rlib.read_bytes()).hexdigest(),'unresolved_pairs':[l for l in log.splitlines() if l.startswith('pair ') and ('error ' in l or 'blocker ' in l)],'summary':[l for l in log.splitlines() if l.startswith('test result:')]}
 (audit/(label+'.json')).write_text(json.dumps(record,indent=2)+'\n')
 print(label,json.dumps(record),flush=True)
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
 futures=[pool.submit(run,'retained-support-intersection-tests1',[binary,'--test-threads=4','--color','never']),pool.submit(run,probe.name,[str(probe)])]
 for f in futures: f.result()
assert manifest=={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted((root/'src').rglob('*.rs'))}
