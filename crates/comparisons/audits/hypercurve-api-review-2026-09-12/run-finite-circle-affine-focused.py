from pathlib import Path
import hashlib,json,re,subprocess,sys,time
root=Path('/home/tim/Documents/GitHub/workspace')
audit=root/'hypercurve-api-review-2026-09-12'
attempt=sys.argv[1]
source=audit/('finite-circle-affine-'+attempt+'-build.jsonl')
binaries={}
for line in source.read_text().splitlines():
 row=json.loads(line)
 if row.get('reason')=='compiler-artifact' and row.get('executable') and row['profile']['test']:
  name='lib' if row['target']['kind']==['lib'] else row['target']['name']
  binaries[name]=row['executable']
rows=[]
for target,filter in [
 ('lib','selected_circle_affine_domains_keep_exterior_and_algebraic_parameters'),
 ('lib','selected_circle_affine_clipping_replays_exact_boundary_identity'),
 ('lib','selected_fillet_tangency_replays_an_exterior_affine_source_chart'),
 ('hypercurve_curve_intersection','selected_circle_tangency_reuses_retained_normal_evidence'),
 ('lib','curve_intersection::curve_support_intersection::circle_dispatch_tests'),
]:
 binary=binaries[target]
 stem='finite-circle-affine-'+attempt+'-'+filter.split('::')[-1]
 cmd=[binary,filter,'--test-threads=2','--color','never','--nocapture']
 start=time.monotonic()
 with (audit/(stem+'.log')).open('w') as out:
  try:
   code=subprocess.run(cmd,stdout=out,stderr=subprocess.STDOUT,timeout=120).returncode
  except subprocess.TimeoutExpired:
   code=124
 row={'target':target,'filter':filter,'command':cmd,'returncode':code,'elapsed_seconds':time.monotonic()-start,'log':stem+'.log','binary':binary,'sha256':hashlib.sha256(Path(binary).read_bytes()).hexdigest()}
 rows.append(row)
 (audit/('finite-circle-affine-'+attempt+'-focused.json')).write_text(json.dumps(rows,indent=2)+'\n')
 print(row,flush=True)
 print((audit/(stem+'.log')).read_text()[-3000:],flush=True)
 if code: raise SystemExit(code)
(audit/'finite-circle-affine-final-focused.json').write_text(json.dumps(rows,indent=2)+'\n')
