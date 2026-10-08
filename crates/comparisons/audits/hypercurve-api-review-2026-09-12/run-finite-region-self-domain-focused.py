from pathlib import Path
import hashlib,json,subprocess,time
r=Path('/home/tim/Documents/GitHub/workspace');a=r/'hypercurve-api-review-2026-09-12';prefix='finite-region-self-domain'
import sys
build_stem=sys.argv[1] if len(sys.argv)>1 else prefix+'-test-build'
artifacts=[json.loads(l) for l in (a/(build_stem+'.jsonl')).read_text().splitlines()]
results=[]
for target,test in [('hypercurve', 'curve_support::tests::injectivity_certificates_own_their_domain_and_circle_chart'), ('hypercurve', 'curve_intersection::curve_support_intersection::analytic_dispatch_tests::finite_bezier_self_contacts_retain_domain_and_point_evidence'), ('hypercurve', 'curve_intersection::curve_support_intersection::analytic_dispatch_tests::finite_bezier_self_components_exclude_the_identity_diagonal'), ('hypercurve', 'curve_region_boolean::certified_successor_tests::finite_self_crossing_regions_retain_boundary_ownership_on_reentry'), ('hypercurve', 'curve_region_boolean::certified_successor_tests::regularization_removes_symmetric_polynomial_and_rational_retracing'), ('hypercurve', 'bezier_region::tests::selected_circle_and_retained_rational_arc_chamfer_extend_exactly'), ('hypercurve', 'curve_region_trim::tests::finite_bezier_carrier_preparation_preserves_trim_and_replay')]:
 b=next(x['executable'] for x in artifacts if x.get('reason')=='compiler-artifact' and x['target']['name']==target and x.get('executable'))
 cmd=[b,'--exact',test,'--test-threads=1','--nocapture'];stem=prefix+'-focused-'+test.rsplit('::',1)[-1];start=time.monotonic()
 with (a/(stem+'.log')).open('w') as log:
  try:code=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,timeout=120).returncode
  except subprocess.TimeoutExpired:code=124
 if code == 0: assert '1 passed; 0 failed;' in (a/(stem+'.log')).read_text(), test
 row={'command':cmd,'returncode':code,'elapsed_seconds':time.monotonic()-start,'binary':b,'sha256':hashlib.sha256(Path(b).read_bytes()).hexdigest(),'log':stem+'.log'};results.append(row);print(row,flush=True)
 if code:print((a/(stem+'.log')).read_text()[-6000:],flush=True)
(a/(prefix+'-focused.json')).write_text(json.dumps(results,indent=2)+'\n')
raise SystemExit(int(any(row['returncode'] for row in results)))
