from pathlib import Path
import hashlib,json,os,re,shutil,subprocess,time,signal
A=Path(__file__).resolve().parent;W=A.parent;prefix='point-similarity-composition-20260927-v410';build=A/'build-workspace-20260925';archive=A/'source-archives'/prefix
prior=json.loads((A/'retained-linear-component-replay-20260927-v407-sources.json').read_text());manifest={};assert not archive.exists()
for name,old_sha in prior.items():
 src=W/name;data=src.read_bytes();sha=hashlib.sha256(data).hexdigest();assert sha==old_sha or name in ['hypercurve/src/bezier_offset.rs'],name
 dst=archive/name;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,dst);target=build/name
 if target.read_bytes()!=data:shutil.copy2(src,target);os.utime(target,None)
 assert dst.stat().st_ino!=target.stat().st_ino;manifest[name]=sha

(A/f'{prefix}-sources.json').write_text(json.dumps(manifest,indent=2)+'\n')
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()));cargo='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
report=dict(source_manifest=f'{prefix}-sources.json',source_directory=str(archive),cases=[],builds=[],all_processes_reaped=False)
def verify():
 for name,sha in manifest.items():
  for root in [W,archive,build]:assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,name

def save():
 (A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
def run_owned(cmd, *, cwd, env, stdout, stderr, timeout):
 process=subprocess.Popen(cmd,cwd=cwd,env=env,stdout=stdout,stderr=stderr,start_new_session=True)
 try:return process.wait(timeout=timeout)
 except BaseException:
  os.killpg(process.pid,signal.SIGKILL);process.wait();raise

def compile_binary(crate,target,selector):
 cmd=[cargo,'test',*selector,'--release','--all-features','--no-run','--message-format=json','--locked','--offline'];log=A/f'{prefix}-{target}-build.log';start=time.monotonic()
 with log.open('w')as out:code=run_owned(cmd,cwd=build/crate,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=900)
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
  try:code=run_owned([str(binary),'--exact',name,'--nocapture','--test-threads=1'],cwd=archive,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=240)
  except subprocess.TimeoutExpired:code=124
 report['cases'].append(dict(binary=binary.name,name=name,returncode=code,log=log.name,elapsed_seconds=time.monotonic()-start));save();print(name,code,round(time.monotonic()-start,2),flush=True)
 if code or not re.search(r'test result: ok\. 1 passed;',log.read_text()):print(log.read_text()[-3000:],flush=True);raise RuntimeError('case failure or missing exact test')
def run_record(label,cmd,cwd,timeout=900):
 log=A/f'{prefix}-{label}.log';start=time.monotonic()
 with log.open('w')as out:
  try:code=run_owned(cmd,cwd=cwd,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=timeout)
  except subprocess.TimeoutExpired:code=124
 record=dict(label=label,command=cmd,returncode=code,log=log.name,elapsed_seconds=time.monotonic()-start);report['checks'].append(record);save();print(label,code,round(record['elapsed_seconds'],2),flush=True)
 if code:
  with log.open()as f:f.seek(max(0,log.stat().st_size-5000));print(f.read()[-3000:],flush=True)
  raise RuntimeError(label+' failed')
 return log,record
verify();code=0;report['checks']=[];report['qualification_scope']='Compose compatible retained point similarities; preserve exact source field identity, operation order, and policy barriers'
try:
 broad=json.loads((A/'retained-linear-component-replay-20260927-v407-terminal.json').read_text())
 focused=json.loads((A/'point-similarity-composition-20260927-v409-terminal.json').read_text())
 assert broad['qualification_complete'] and broad['all_processes_reaped'] and focused['qualification_complete'] and focused['all_processes_reaped']
 assert manifest==json.loads((A/focused['source_manifest']).read_text())
 expected={target:list(names)for target,names in broad['expected_cases'].items()}
 for name in ['bezier_offset::conversion_tests::repeated_point_similarities_keep_one_original_selected_field', 'bezier_offset::conversion_tests::point_similarity_composition_preserves_retained_policy_barriers', 'bezier_offset::conversion_tests::mapped_pair_angle_queries_observe_requested_policy_on_both_sides', 'bezier_offset::conversion_tests::circle_pair_parameter_order_reuses_angles_across_frames_and_radii', 'bezier_offset::conversion_tests::mapped_overlap_angle_queries_observe_requested_policy', 'bezier_offset::conversion_tests::represented_circle_incidence_replays_retained_similarity_points', 'bezier_offset::conversion_tests::represented_circle_retained_point_incidence_observes_requested_policy', 'bezier_offset::conversion_tests::algebraic_cusp_semicircle_similarities_transform_the_shared_frame_exactly', 'bezier_offset::conversion_tests::direct_algebraic_circle_similarities_retain_center_and_cardinal_evidence', 'bezier_offset::conversion_tests::pair_radial_circle_intersects_a_general_analytic_parallel_exactly', 'bezier_offset::conversion_tests::pair_radial_circle_extends_over_the_regular_analytic_incident_ray', 'bezier_offset::conversion_tests::algebraic_cusp_semicircle_replays_a_selected_circle_component', 'bezier_offset::conversion_tests::similarity_transported_mapped_cusp_cut_inverts_on_analytic_overlap', 'bezier_offset::conversion_tests::chamfer_transported_mapped_cusp_cut_inverts_on_analytic_overlap', 'bezier_offset::conversion_tests::selected_fiber_overlap_inverts_transported_mapped_cusp_cut', 'bezier_offset::conversion_tests::selected_fiber_mapped_cut_inverts_on_analytic_overlap', 'bezier_offset::conversion_tests::similarity_transported_transverse_mapped_cut_inverts_by_point', 'bezier_offset::conversion_tests::selected_fiber_transverse_mapped_cut_inverts_by_point', 'bezier_offset::conversion_tests::selected_fiber_analytic_transverse_cut_inverts_by_point', 'bezier_offset::conversion_tests::nonrepresented_center_nonrational_chamfer_inverts_with_retained_authority', 'bezier_offset::conversion_tests::algebraic_cusp_semicircle_pair_replays_independent_selected_roots', 'bezier_offset::conversion_tests::algebraic_cusp_semicircle_reoffsets_pair_mapped_lens', 'bezier_offset::conversion_tests::algebraic_cusp_semicircle_reoffsets_nested_chord_mapped_cap', 'bezier_offset::conversion_tests::algebraic_cusp_semicircle_pair_handles_tangent_disjoint_and_coincident_supports', 'bezier_offset::conversion_tests::algebraic_cusp_semicircle_pair_maps_full_partial_and_endpoint_only_overlap', 'bezier_offset::conversion_tests::curve_region_offset_transports_an_internal_correlated_circle_partition', 'bezier_offset::conversion_tests::selected_circle_partition_coalescing_obeys_terminal_policy', 'bezier_offset::conversion_tests::selected_fiber_circle_overlap_preserves_an_isolated_parameter_visit', 'bezier_offset::conversion_tests::rational_circle_component_clipping_keeps_shared_boundaries_in_their_cells', 'bezier_offset::conversion_tests::reconstructed_circle_overlap_reuses_parameter_identity_and_inverse', 'bezier_offset::conversion_tests::curve_region_boolean_clips_correlated_partial_cusp_overlaps', 'bezier_offset::conversion_tests::distinct_mapped_algebraic_cusp_endpoint_fields_classify_directly', 'bezier_offset::conversion_tests::recursive_projective_kernel_imports_similarity_of_algebraic_endpoints', 'bezier_offset::conversion_tests::recursively_pair_radial_circle_materializes_its_exact_frame', 'bezier_offset::conversion_tests::recursive_projective_oblique_chord_handles_even_contact_multiplicity', 'bezier_offset::conversion_tests::recursively_pair_radial_circles_intersect_and_recurse_exactly', 'bezier_offset::conversion_tests::independently_encoded_recursive_circles_intersect_and_recurse_exactly', 'bezier_offset::conversion_tests::independently_encoded_rotated_recursive_circles_intersect_exactly', 'bezier_offset::conversion_tests::independently_encoded_scaled_rotated_recursive_circles_intersect_exactly', 'bezier_offset::conversion_tests::nonstructural_recursive_centers_handle_exact_tangency', 'bezier_offset::conversion_tests::independently_encoded_recursive_circles_map_all_coincident_half_relations', 'bezier_offset::conversion_tests::pair_radial_authored_tangency_survives_scaled_reflection', 'bezier_offset::conversion_tests::recursively_pair_radial_circles_handle_tangent_and_disjoint_supports', 'bezier_offset::conversion_tests::recursive_line_contact_circles_reject_disjoint_supports_without_flattening', 'bezier_offset::conversion_tests::recursive_line_contact_circles_publish_transverse_contacts_without_flattening', 'bezier_offset::conversion_tests::recursive_line_contact_circles_publish_tangent_contacts_without_flattening', 'bezier_offset::conversion_tests::independently_pair_radial_circles_intersect_exactly', 'bezier_offset::conversion_tests::independently_pair_radial_circles_handle_tangent_disjoint_and_concentric_supports', 'bezier_offset::conversion_tests::independently_pair_radial_circles_map_all_coincident_half_relations', 'bezier_offset::conversion_tests::finite_circle_tangent_inverse_replays_independent_cuts', 'bezier_offset::conversion_tests::finite_circle_components_replay_pair_radial_and_recursive_inverses', 'bezier_offset::conversion_tests::exact_parallel_similarity_transports_points_derivatives_and_structure', 'bezier_offset::conversion_tests::exact_parallel_reflection_negates_scaled_left_distance', 'transform::tests::exact_similarity_preserves_translation_beyond_f64_resolution', 'transform::tests::exact_similarity_rejects_anisotropic_scale', 'transform::tests::point_transform_fuses_exact_affine_sums_and_preserves_symbolic_expression', 'transform::tests::finite_similarity_constructor_canonicalizes_the_accepted_linear_part', 'transform::tests::exact_similarity_composition_matches_sequential_application']:
  if name not in expected['hypercurve']:expected['hypercurve'].append(name)
 expected['hypercurve_curve_region_promotion'] += ['correlated_chord_pair_endpoints_survive_transform_and_offset','algebraic_chords_survive_nonsingular_exact_affine_transforms','similarity_rotation_preserves_unified_region_semantics_and_fast_path']
 for name in focused['expected_cases']['hypercurve']:
  if name not in expected['hypercurve']:expected['hypercurve'].append(name)
 report['expected_cases']=expected;report['known_unresolved']=broad['known_unresolved'];report['reused_source_qualification']='point-similarity-composition-20260927-v409-terminal.json';save()
 for label,features in [('hypercurve-clippy-all-features',['--all-features']),('hypercurve-clippy-no-default',['--no-default-features'])]:
  run_record(label,[cargo,'clippy','--all-targets',*features,'--locked','--offline','--','-D','warnings'],build/'hypercurve')
 run_record('hypercurve-format',['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustfmt','--edition','2024','--config','skip_children=true','--check','src/curve.rs','src/curve_fillet.rs','src/bezier_offset.rs','src/curve_corner_chain.rs'],build/'hypercurve')
 run_record('fuzz-check',[cargo,'check','--manifest-path','fuzz/Cargo.toml','--bin','curve_string_editing','--locked','--offline'],build/'hypercurve')
 env['RUSTDOCFLAGS']='-D warnings'
 run_record('hypercurve-documentation',[cargo,'doc','--no-deps','--all-features','--locked','--offline'],build/'hypercurve')
 run_record('hyperbrep-check',[cargo,'check','--all-targets','--all-features','--locked','--offline'],build/'hyperbrep')
 reusable={r['name']:r for r in focused['cases']}
 focused_binary=focused['builds'][0]['binary']
 for target,names in expected.items():
  binary=compile_binary('hypercurve',target,['--lib'] if target=='hypercurve' else ['--test',target])
  for name in names:
   if target=='hypercurve' and name in reusable:
    assert hashlib.sha256(binary.read_bytes()).hexdigest()==focused_binary['sha256']
    record=dict(reusable[name]);assert record['returncode']==0 and 'test result: ok. 1 passed;' in (A/record['log']).read_text();record['binary']=binary.name;record['reused_from']=report['reused_source_qualification'];report['cases'].append(record);save()
   else:case(binary,name)
except Exception as error:
 code=1;report['failure']=str(error)
finally:
 verify();report['all_processes_reaped']=True;report['qualification_complete']=code==0;save();print('broad_qualification_complete',code==0,flush=True)
raise SystemExit(code)
