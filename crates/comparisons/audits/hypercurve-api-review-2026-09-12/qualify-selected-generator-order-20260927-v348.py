from pathlib import Path
import hashlib,json,os,re,shutil,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent;prefix='selected-generator-order-20260927-v348';build=A/'build-workspace-20260925';archive=A/'source-archives'/prefix
prior=json.loads((A/'selected-generator-order-20260927-v347-sources.json').read_text());manifest={};assert not archive.exists()
for name,old_sha in prior.items():
 src=W/name;data=src.read_bytes();sha=hashlib.sha256(data).hexdigest();assert sha==old_sha,name
 dst=archive/name;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,dst);target=build/name
 if target.read_bytes()!=data:shutil.copy2(src,target);os.utime(target,None)
 assert dst.stat().st_ino!=target.stat().st_ino;manifest[name]=sha

(A/f'{prefix}-sources.json').write_text(json.dumps(manifest,indent=2)+'\n')
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()));cargo='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
focused_receipt='selected-generator-order-20260927-v347-terminal.json'
focused=json.loads((A/focused_receipt).read_text());assert focused['qualification_complete'] and focused['all_processes_reaped']
report=dict(source_manifest=f'{prefix}-sources.json',source_directory=str(archive),cases=[dict(c,reused_from=focused_receipt) for c in focused['cases']],builds=[],all_processes_reaped=False)
def verify():
 for name,sha in manifest.items():
  for root in [W,archive,build]:assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,name

def save():
 verify();(A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
def compile_binary(crate,target,selector):
 if target=='hypercurve':
  record=dict(focused['builds'][0],reused_from=focused_receipt);binary=Path(record['binary']['path']);assert hashlib.sha256(binary.read_bytes()).hexdigest()==record['binary']['sha256'];report['builds'].append(record);save();return binary
 cmd=[cargo,'test',*selector,'--release','--all-features','--no-run','--message-format=json','--locked','--offline'];log=A/f'{prefix}-{target}-build.log';start=time.monotonic()
 with log.open('w')as out:code=subprocess.run(cmd,cwd=build/crate,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=900).returncode
 record=dict(crate=crate,command=cmd,returncode=code,log=log.name,elapsed_seconds=time.monotonic()-start);report['builds'].append(record);save()
 if code:
  for line in log.read_text().splitlines():
   try:r=json.loads(line)
   except ValueError:continue
   if r.get('reason')=='compiler-message'and r['message']['level']=='error':print(r['message'].get('rendered','')[:3000],flush=True)
  raise RuntimeError('build failure')
 rows=[]
 for line in log.read_text().splitlines():
  try:rows.append(json.loads(line))
  except ValueError:pass
 row=next(r for r in rows if r.get('reason')=='compiler-artifact'and r['target']['name']==target and r.get('executable'))
 binary=A/f'{prefix}-{target}';shutil.copy2(row['executable'],binary);record['binary']=dict(path=str(binary),sha256=hashlib.sha256(binary.read_bytes()).hexdigest());save();return binary

def case(binary,name):
 log=A/f'{prefix}-case-{len(report["cases"]):03d}.log';start=time.monotonic()
 with log.open('w')as out:
  try:code=subprocess.run([str(binary),'--exact',name,'--nocapture','--test-threads=1'],cwd=archive,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=240).returncode
  except subprocess.TimeoutExpired:code=124
 report['cases'].append(dict(binary=binary.name,name=name,returncode=code,log=log.name,elapsed_seconds=time.monotonic()-start));save();print(name,code,round(time.monotonic()-start,2),flush=True)
 if code:print(log.read_text()[-3000:],flush=True);raise RuntimeError('case failure')
def run_record(label,cmd,cwd,timeout=900):
 log=A/f'{prefix}-{label}.log';start=time.monotonic()
 with log.open('w')as out:
  try:code=subprocess.run(cmd,cwd=cwd,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=timeout).returncode
  except subprocess.TimeoutExpired:code=124
 record=dict(label=label,command=cmd,returncode=code,log=log.name,elapsed_seconds=time.monotonic()-start);report['checks'].append(record);save();print(label,code,round(record['elapsed_seconds'],2),flush=True)
 if code:
  with log.open()as f:f.seek(max(0,log.stat().st_size-5000));print(f.read()[-3000:],flush=True)
  raise RuntimeError(label+' failed')
 return log,record
verify();code=0;report['checks']=[];report['qualification_scope']='Coefficient generator order and field reuse, local polynomial decisions, and fillet/chamfer/Boolean/offset compositions'
try:
 for label,features in [('clippy-all-features',['--all-features']),('clippy-no-default',['--no-default-features'])]:
  run_record(label,[cargo,'clippy','--all-targets',*features,'--locked','--offline','--','-D','warnings'],build/'hypercurve')
 changed=subprocess.check_output(['git','diff','--name-only'],cwd=W/'hypercurve',text=True).splitlines()
 rust=[p for p in changed if p.endswith('.rs')]
 run_record('format',['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustfmt','--edition','2024','--config','skip_children=true','--check',*rust],build/'hypercurve')
 report['known_unresolved_not_requalified']=['strict_trim_or_extend_analytic_parallel_support_corners_retain_algebraic_fillet_extensions','approximate_512_trim_or_extend_analytic_parallel_support_corners_retain_algebraic_fillet_extensions']
 targets=['hypercurve','hypercurve_curve','hypercurve_curve_region_promotion','hypercurve_path_closure','hypercurve_analytic_parallel_region','hypercurve_curve_intersection','hypercurve_curve_region_boolean']
 report['expected_cases']={};save()
 for target in targets:
  binary=compile_binary('hypercurve',target,['--lib'] if target=='hypercurve' else ['--test',target])
  names=subprocess.check_output([str(binary),'--list'],text=True).splitlines();names=[n.removesuffix(': test')for n in names if n.endswith(': test')]
  selected=[n for n in names if ('fillet' in n or 'chamfer' in n or 'point_incidence' in n or (target=='hypercurve' and ('parameter_component::' in n or 'circular_spline_contact_parameters_distinguish_repeated_point_visits' in n or ('rational_bezier_general::' in n and any(term in n for term in ['overlap', 'projective', 'parameter']))))) and n not in report['known_unresolved_not_requalified']]
  priority=['curve::curve_fillet::tests::collapsed_selected_circle_fillet_crosses_seams_and_reenters_region_editing','curve::curve_fillet::tests::collapsed_selected_circle_fillet_reuses_constrained_contacts_and_support','curve::curve_subdivision::tests::source_domain_nonlinear_fillet_crosses_circular_chart_infinity','curve::curve_fillet::tests::native_circle_fillet_retains_selected_contacts_through_reconstruction','curve::curve_fillet::tests::coincident_line_fillet_constraints_complete_the_authored_chart_solutions','curve::curve_fillet::tests::coincident_linear_fillet_domains_exclude_remote_endpoints','curve::curve_fillet::tests::coincident_retained_line_fillet_keeps_independent_endpoint_fields','bezier_offset::conversion_tests::point_incidence_checks_regular_frames_at_each_isolated_contact','curve::curve_fillet::tests::collapsed_parallel_fillet_requires_the_original_circle_normal','curve::curve_fillet::tests::collapsed_circle_parallel_fillet_replays_contacts_and_normal_sheets','curve::curve_fillet::tests::collapsed_fillet_point_contacts_preserve_distinct_bezier_preimages','curve::curve_fillet::tests::collapsed_circle_fillet_requires_each_free_contact_and_preserves_exact_trims','curve::curve_fillet::tests::collapsed_circular_spline_fillets_keep_authored_contact_parameters','bezier_region::tests::collapsed_pair_radial_fillet_rejects_the_excluded_contact']
  if target=='hypercurve':
   extra=['recursive_local_root_orders_against_close_coefficient_generators','retained_parameter_import_keeps_original_axes_and_tower','recursive_scalar_comparison_replays_exact_equality_before_approximation','recursive_local_isolation_reuses_exact_signs_below_interval_precision','recursive_parameter_projection_preserves_roots_through_refinement_and_charts','recursive_polynomial_predicates_reuse_the_retained_field_relation','recursive_polynomial_queries_reuse_a_selected_proper_factor','recursive_ordered_field_isolation_does_not_guess_polynomial_degree','recursive_polynomial_isolators_keep_the_selected_root_after_endpoint_deflation','recursive_polynomial_refinement_reuses_certified_bounds_and_identity','recursive_polynomial_crossing_signs_survive_exact_midpoint_collapse','recursive_chord_parallel_retains_owned_roots_across_exterior_ranges','recursive_chord_parallel_local_roots_reject_conjugates_and_clip_exactly','exterior_chord_contact_deflation_preserves_other_contacts','recursive_chord_kernel_retains_all_cubic_crossings_with_opposite_endpoint_sides','recursive_quadratic_endpoint_roots_keep_the_original_field','retained_chord_incidence_replays_independently_allocated_coefficient_fields','recursive_field_embeddings_do_not_consume_approximate_generator_equalities','retained_parallel_contact_keeps_local_root_through_angular_and_point_queries','recursive_parallel_expression_signs_keep_the_selected_speed_sheet','recursive_parallel_expression_signs_reject_poles_and_zero_speed','chord_normal_and_parallel_contacts_share_native_point_identity','selected_chord_normal_contact_accepts_complementary_radial_sign','dense_chord_normal_tangency_retains_a_finite_endpoint','independent_cusp_chord_contacts_order_on_the_shared_axis','procedural_endpoint_chord_replays_complete_parallel_contacts','finite_circle_components_replay_chord_normal_and_represented_inverses','rank_independent_chord_normal_circle_partitions_rational_overlap','pair_radial_circle_retains_independent_field_chord_tangency','nested_procedural_chord_reversals_match_exact_side_and_winding','algebraic_chord_retains_independent_endpoint_fields_under_both_policies','noninjective_collinear_chord_overlap_partitions_every_monotone_branch']
   extra=[f'bezier_offset::conversion_tests::{n}' for n in extra]
   assert all(n in names for n in extra)
   selected.extend(n for n in extra if n not in selected)
   priority=extra+priority
  selected=sorted(selected,key=lambda n:(priority.index(n) if n in priority else len(priority),n))
  assert selected,target
  report['expected_cases'][target]=selected;save()
  for name in selected:
   if any(c['name']==name and c['binary']==binary.name and c['returncode']==0 for c in report['cases']):continue
   case(binary,name)
 run_record('fuzz-check',[cargo,'check','--manifest-path','fuzz/Cargo.toml','--bin','curve_string_editing','--locked','--offline'],build/'hypercurve')
 env['RUSTDOCFLAGS']='-D warnings'
 run_record('documentation',[cargo,'doc','--no-deps','--all-features','--locked','--offline'],build/'hypercurve')
 run_record('hyperbrep-check',[cargo,'check','--all-targets','--all-features','--locked','--offline'],build/'hyperbrep')
except Exception as error:
 code=1;report['failure']=str(error)
finally:
 report['all_processes_reaped']=True;report['qualification_complete']=code==0;save();print('scoped_qualification_complete',code==0,flush=True)
raise SystemExit(code)
