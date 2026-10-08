from pathlib import Path
import hashlib,json,subprocess,time
r=Path('/home/tim/Documents/GitHub/workspace');a=r/'hypercurve-api-review-2026-09-12';prefix='finite-analytic-pair-domains'
import sys
build_stem=sys.argv[1] if len(sys.argv)>1 else prefix+'-test-build'
artifacts=[json.loads(l) for l in (a/(build_stem+'.jsonl')).read_text().splitlines()]
results=[]
for target,test in [('hypercurve', 'curve_intersection::curve_support_intersection::analytic_dispatch_tests::finite_analytic_pairs_replay_transverse_and_tangent_contacts'), ('hypercurve', 'curve_intersection::curve_support_intersection::analytic_dispatch_tests::finite_analytic_pairs_retain_components_in_the_original_charts'), ('hypercurve', 'curve_intersection::curve_support_intersection::analytic_dispatch_tests::finite_analytic_constant_images_retain_the_complete_parameter_fiber'), ('hypercurve', 'curve_intersection::curve_support_intersection::analytic_dispatch_tests::common_analytic_pairs_replay_contacts_in_both_orders_and_traversals'), ('hypercurve', 'curve_intersection::curve_support_intersection::analytic_dispatch_tests::common_analytic_overlaps_transport_distinct_source_charts'), ('hypercurve', 'curve_intersection::curve_support_intersection::analytic_dispatch_tests::coincident_constant_parallels_retain_the_parameter_rectangle'), ('hypercurve_bezier_fit_offset', 'finite_parallel_point_incidence_owns_poles_roots_and_normal_sheets'), ('hypercurve_bezier_fit_offset', 'parallel_point_incidence_uses_approximate_512_only_as_a_terminal_decision')]:
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
