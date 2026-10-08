from pathlib import Path
import hashlib,json,subprocess,time
r=Path('/home/tim/Documents/GitHub/workspace');a=r/'hypercurve-api-review-2026-09-12';prefix='finite-circle-projection'
import sys
build_stem=sys.argv[1] if len(sys.argv)>1 else prefix+'-test-build'
artifacts=[json.loads(l) for l in (a/(build_stem+'.jsonl')).read_text().splitlines()]
results=[]
for target,test in [('hypercurve', 'bezier_offset::conversion_tests::selected_axis_projection_owns_finite_ranges_and_incident_roots'), ('hypercurve', 'bezier_offset::conversion_tests::selected_projection_replays_selected_boundaries_under_the_original_policy'), ('hypercurve', 'bezier_offset::conversion_tests::finite_fiber_roots_keep_selected_boundary_ownership'), ('hypercurve', 'bezier_offset::conversion_tests::selected_fiber_interval_supports_exact_and_nonexact_exterior_roots'), ('hypercurve', 'bezier_offset::conversion_tests::selected_circle_parallel_incident_coincidence_is_positive_dimensional'), ('hypercurve', 'bezier_offset::conversion_tests::selected_affine_tangent_source_retains_incident_endpoint_and_contacts_locally'), ('hypercurve', 'bezier_offset::conversion_tests::selected_parallel_normal_positive_dimensional_projection_rejects_conjugate_components'), ('hypercurve', 'bezier_offset::conversion_tests::incident_selected_fiber_projection_honors_the_requested_resultant_schedule')]:
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
