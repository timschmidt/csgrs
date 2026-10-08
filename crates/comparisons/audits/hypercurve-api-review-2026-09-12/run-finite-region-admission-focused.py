from pathlib import Path
import hashlib,json,subprocess,time
r=Path('/home/tim/Documents/GitHub/workspace');a=r/'hypercurve-api-review-2026-09-12';prefix='finite-region-admission'
artifacts=[json.loads(l) for l in (a/(prefix+'-test-build.jsonl')).read_text().splitlines()]
results=[]
for target,test in [
 ('hypercurve','curve_region_boolean::certified_successor_tests::noninjective_preimages_survive_curve_queries_and_cancel_from_regions'),
 ('hypercurve','curve_region_boolean::certified_successor_tests::regularization_removes_symmetric_polynomial_and_rational_retracing'),
 ('hypercurve','curve_region_boolean::certified_successor_tests::exterior_circular_split_preserves_major_arc_and_algebraic_endpoints'),
 ('hypercurve_curve_region_boolean','region_intersection_removes_authored_internal_and_canceled_boundaries'),
 ('hypercurve_curve_region_boolean','selected_fillet_region_intersection_closes_through_exterior_cap_booleans'),
]:
 b=next(x['executable'] for x in artifacts if x.get('reason')=='compiler-artifact' and x['target']['name']==target and x.get('executable'))
 cmd=[b,'--exact',test,'--test-threads=1','--nocapture'];stem=prefix+'-focused-'+test.rsplit('::',1)[-1];start=time.monotonic()
 with (a/(stem+'.log')).open('w') as log:
  try:code=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,timeout=120).returncode
  except subprocess.TimeoutExpired:code=124
 row={'command':cmd,'returncode':code,'elapsed_seconds':time.monotonic()-start,'binary':b,'sha256':hashlib.sha256(Path(b).read_bytes()).hexdigest(),'log':stem+'.log'};results.append(row);print(row,flush=True)
 if code:print((a/(stem+'.log')).read_text()[-6000:],flush=True)
(a/(prefix+'-focused.json')).write_text(json.dumps(results,indent=2)+'\n')
raise SystemExit(int(any(row['returncode'] for row in results)))
