from pathlib import Path
import hashlib,json,subprocess,time
r=Path('/home/tim/Documents/GitHub/workspace');a=r/'hypercurve-api-review-2026-09-12';prefix='finite-circle-inverse-domains'
import sys
build_stem=sys.argv[1] if len(sys.argv)>1 else prefix+'-test-build'
artifacts=[json.loads(l) for l in (a/(build_stem+'.jsonl')).read_text().splitlines()]
results=[]
for target,test in [('hypercurve', 'bezier_offset::conversion_tests::finite_circle_tangent_inverse_replays_independent_cuts'), ('hypercurve', 'bezier_offset::conversion_tests::tangent_inverse_retains_exterior_selected_domain_evidence'), ('hypercurve', 'bezier_offset::conversion_tests::selected_fiber_tangent_correspondence_retains_local_image'), ('hypercurve', 'bezier_offset::conversion_tests::chamfer_transported_mapped_cusp_cut_inverts_on_analytic_overlap'), ('hypercurve', 'bezier_offset::conversion_tests::transverse_chamfer_cut_inverts_by_radial_incidence'), ('hypercurve', 'bezier_offset::conversion_tests::nonrepresented_center_transverse_chamfer_inverts_by_correlated_point'), ('hypercurve', 'bezier_offset::conversion_tests::nonrepresented_center_nonrational_chamfer_inverts_with_retained_authority'), ('hypercurve', 'bezier_offset::conversion_tests::selected_fiber_overlap_inverts_transported_mapped_cusp_cut'), ('hypercurve', 'bezier_offset::conversion_tests::mapped_overlap_retains_distinct_selected_inverse_keys_and_rebuilds_expired_values'), ('hypercurve', 'bezier_offset::conversion_tests::nonlinear_parameter_component_maps_and_clips_its_exact_branch'), ('hypercurve', 'bezier_offset::conversion_tests::selected_fiber_mapped_cut_inverts_on_analytic_overlap'), ('hypercurve', 'bezier_offset::conversion_tests::selected_fiber_rational_circle_overlaps_complete_region_booleans'), ('hypercurve', 'bezier_offset::conversion_tests::selected_fiber_analytic_circle_overlaps_complete_region_booleans')]:
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
