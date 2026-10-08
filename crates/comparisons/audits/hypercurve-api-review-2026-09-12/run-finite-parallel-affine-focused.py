from pathlib import Path
import hashlib,json,subprocess,time
r=Path('/home/tim/Documents/GitHub/workspace');a=r/'hypercurve-api-review-2026-09-12';prefix='finite-parallel-affine'
import sys
build_stem=sys.argv[1] if len(sys.argv)>1 else prefix+'-test-build'
artifacts=[json.loads(l) for l in (a/(build_stem+'.jsonl')).read_text().splitlines()]
results=[]
for target,test in [('hypercurve', 'curve_support::tests::finite_parallel_derivatives_preserve_affine_charts_and_gauges'), ('hypercurve', 'curve_support::tests::finite_parallel_cusp_derivatives_retain_the_selected_normal_sheet'), ('hypercurve', 'curve_support::tests::parallel_source_tangents_exclude_poles_and_undefined_normals'), ('hypercurve', 'curve_support::tests::retained_parallel_representatives_use_their_finite_support_range'), ('hypercurve', 'bezier_offset::conversion_tests::finite_parallel_regularity_clips_against_selected_endpoint_authorities'), ('hypercurve', 'bezier_offset::conversion_tests::represented_derivative_scale_fast_path_matches_polynomial_certificate'), ('hypercurve', 'bezier_offset::conversion_tests::exact_parallel_similarity_transports_points_derivatives_and_structure'), ('hypercurve', 'bezier_offset::conversion_tests::exact_parallel_reflection_negates_scaled_left_distance'), ('hypercurve_bezier_fit_offset', 'quadratic_parallel_materializes_radical_cusp_parameter'), ('hypercurve_bezier_fit_offset', 'quadratic_parallel_isolates_distance_dependent_interior_cusp')]:
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
