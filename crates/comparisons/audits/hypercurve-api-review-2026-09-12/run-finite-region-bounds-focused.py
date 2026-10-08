from pathlib import Path
import hashlib,json,subprocess,time
r=Path('/home/tim/Documents/GitHub/workspace');a=r/'hypercurve-api-review-2026-09-12';prefix='finite-region-bounds'
import sys
build_stem=sys.argv[1] if len(sys.argv)>1 else prefix+'-test-build'
artifacts=[json.loads(l) for l in (a/(build_stem+'.jsonl')).read_text().splitlines()]
results=[]
for target,test in [('hypercurve', 'curve_support::tests::exterior_bounds_include_interior_extrema_and_parallel_displacement'), ('hypercurve', 'curve_support::tests::finite_bounds_exclude_poles_elsewhere_in_the_native_chart'), ('hypercurve', 'rational_bezier_general::tests::finite_line_roots_preserve_tangency_crossing_and_denominator_sign'), ('hypercurve', 'bezier_offset::conversion_tests::denominator_sign_excludes_a_pole_outside_selected_bounds'), ('hypercurve_curve_region_boolean', 'finite_bezier_charts_preserve_bounds_boundary_and_winding'), ('hypercurve_curve_region_boolean', 'native_chart_poles_do_not_block_finite_region_queries'), ('hypercurve', 'curve_region_boolean::certified_successor_tests::exterior_circular_split_preserves_major_arc_and_algebraic_endpoints'), ('hypercurve_curve_region_boolean', 'selected_fillet_region_intersection_closes_through_exterior_cap_booleans')]:
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
