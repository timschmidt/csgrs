from pathlib import Path
import hashlib,json,re,subprocess,sys,time
root=Path('/home/tim/Documents/GitHub/workspace')
audit=root/'hypercurve-api-review-2026-09-12'
attempt=sys.argv[1]
source=audit/('finite-fiber-promotion-'+attempt+'-build.jsonl')
binaries={}
for line in source.read_text().splitlines():
 row=json.loads(line)
 if row.get('reason')=='compiler-artifact' and row.get('executable') and row['profile']['test']:
  name='lib' if row['target']['kind']==['lib'] else row['target']['name']
  binaries[name]=row['executable']
rows=[]
for target,filter in [('lib', 'independently_encoded_nonrepresented_analytic_circle_publishes_overlap'), ('lib', 'nonrepresented_center_transverse_chamfer_inverts_by_correlated_point'), ('lib', 'selected_scalar_promotion_uses_its_finite_isolator'), ('lib', 'selected_norm_isolation_preserves_exterior_exact_and_algebraic_roots'), ('lib', 'degenerate_fiber_projection_cannot_discard_an_exterior_selected_root'), ('lib', 'finite_fiber_roots_keep_selected_boundary_ownership'), ('lib', 'bounded_general_resultant_isolates_only_the_requested_selected_fiber_range'), ('lib', 'selected_fiber_scalar_is_compact_exact_and_ordered_without_a_norm'), ('lib', 'selected_fiber_import_and_refinement_retain_the_known_scalar_representation'), ('lib', 'selected_fiber_equality_requires_selected_root_containment'), ('lib', 'selected_fiber_diagonal_identity_preserves_distinct_root_ownership'), ('lib', 'selected_fiber_norm_isolation_refines_past_the_old_limit'), ('lib', 'selected_fiber_interval_supports_exact_and_nonexact_exterior_roots'), ('lib', 'fixed_distance_retains_selected_candidates_in_both_range_orientations'), ('lib', 'algebraic_center_fixed_distance_projects_complete_high_degree_fiber'), ('lib', 'selected_fiber_positive_dimensional_projection_reports_authored_continuum'), ('lib', 'selected_parallel_normal_positive_dimensional_projection_rejects_conjugate_components'), ('lib', 'selected_fiber_affine_chart_round_trip_avoids_global_projection'), ('lib', 'selected_fiber_projective_chart_round_trip_avoids_global_projection'), ('lib', 'finite_parallel_pair_components_obey_exact_domain_boundaries')]:
 binary=binaries[target]
 stem='finite-fiber-promotion-'+attempt+'-'+filter.split('::')[-1]
 cmd=[binary,filter,'--test-threads=2','--color','never','--nocapture']
 start=time.monotonic()
 with (audit/(stem+'.log')).open('w') as out:
  try:
   code=subprocess.run(cmd,stdout=out,stderr=subprocess.STDOUT,timeout=120).returncode
  except subprocess.TimeoutExpired:
   code=124
 row={'target':target,'filter':filter,'command':cmd,'returncode':code,'elapsed_seconds':time.monotonic()-start,'log':stem+'.log','binary':binary,'sha256':hashlib.sha256(Path(binary).read_bytes()).hexdigest()}
 rows.append(row)
 (audit/('finite-fiber-promotion-'+attempt+'-focused.json')).write_text(json.dumps(rows,indent=2)+'\n')
 print(row,flush=True)
 print((audit/(stem+'.log')).read_text()[-3000:],flush=True)
 if code: raise SystemExit(code)
(audit/'finite-fiber-promotion-final-focused.json').write_text(json.dumps(rows,indent=2)+'\n')
