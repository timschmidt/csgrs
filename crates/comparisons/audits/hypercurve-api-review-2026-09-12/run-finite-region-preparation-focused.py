from pathlib import Path
import hashlib,json,subprocess,time
r=Path('/home/tim/Documents/GitHub/workspace');a=r/'hypercurve-api-review-2026-09-12';prefix='finite-region-preparation'
import sys
build_stem=sys.argv[1] if len(sys.argv)>1 else prefix+'-test-build'
artifacts=[json.loads(l) for l in (a/(build_stem+'.jsonl')).read_text().splitlines()]
results=[]
for target,test in [('hypercurve', 'curve_intersection::curve_support_intersection::circle_dispatch_tests::retained_quadratic_circle_pairs_clip_components_and_endpoint_contacts'), ('hypercurve', 'bezier_region::tests::selected_circle_and_retained_rational_arc_chamfer_extend_exactly'), ('hypercurve', 'curve_region_trim::tests::finite_bezier_carrier_preparation_preserves_trim_and_replay'), ('hypercurve', 'curve_region_trim::tests::generated_circle_trim_preserves_selected_frame_and_reversal'), ('hypercurve', 'curve_region_trim::tests::generated_chord_trim_preserves_holes_and_exact_endpoint_replay'), ('hypercurve_curve_region_boolean', 'selected_fillet_region_intersection_closes_through_exterior_cap_booleans')]:
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
