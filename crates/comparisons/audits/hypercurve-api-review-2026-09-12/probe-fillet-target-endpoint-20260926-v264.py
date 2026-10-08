from pathlib import Path
import hashlib,json,os,shutil,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent
prefix='fillet-target-endpoint-20260926-v264';prior=json.loads((A/'center-locus-cusp-20260926-v260-terminal.json').read_text())
guard=json.loads((A/'center-locus-cusp-20260926-v260-sources.json').read_text());manifest=json.loads((A/prior['source_manifest']).read_text());guard={name:sha for name,sha in guard.items()if not name.startswith('fiber-probe/')};manifest=dict(guard);archive=A/'source-archives'/prefix;build=A/'build-workspace-20260925';assert not archive.exists()
for name,sha in manifest.items():
 src=Path(prior['source_directory'])/name;assert hashlib.sha256(src.read_bytes()).hexdigest()==sha
 dst=archive/name;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,dst)
 target=build/name
 if target.read_bytes()!=src.read_bytes():shutil.copy2(src,target);os.utime(target,None)
 assert dst.stat().st_ino!=target.stat().st_ino
file='hypercurve/src/bezier_offset.rs'
p=archive/file;t=p.read_text();start=t.index('    fn selected_parallel_normal_parallel_intersections(');end=t.index('    fn parallel_contact_at_certified_parameter(',start);part=t[start:end]
original='return Ok(Classification::Uncertain(reason))'
for i in range(part.count(original)):
 part=part.replace(original, f'{{ eprintln!("selected-normal unresolved-stage={i} reason={{reason:?}}"); RETURN_UNCERTAIN(reason) }}',1)
part=part.replace('RETURN_UNCERTAIN(reason)',original)
part=part.replace('        let frame_parameter = self.selected_frame_parameter()', '        eprintln!("selected-normal enter");\n        let frame_parameter = self.selected_frame_parameter()',1)
part=part.replace('        let diagonal_location = if other.source() == frame.center_support.source() {', '        eprintln!("selected-normal source same={} reversed={}", other.source() == frame.center_support.source(), other.source().is_reversal_of(frame.center_support.source()));\n        let diagonal_location = if other.source() == frame.center_support.source() {',1)
part=part.replace('        let (mut projection_identically_zero, mut candidates) =', '        eprintln!("selected-normal isolate-begin");\n        let (mut projection_identically_zero, mut candidates) =',1)
part=part.replace('        if !projection_identically_zero && let Some(incident) = incident {', '        eprintln!("selected-normal isolate-end candidates={} component={}", candidates.len(), projection_identically_zero);\n        if !projection_identically_zero && let Some(incident) = incident {',1)
part=part.replace('        for candidate in candidates {', '        for (candidate_index, candidate) in candidates.into_iter().enumerate() {\n            eprintln!("selected-normal candidate={} circle-begin", candidate_index);',1)
part=part.replace('            match circle_sign {', '            eprintln!("selected-normal candidate={} circle-end sign={:?}", candidate_index, circle_sign);\n            match circle_sign {',1)
part=part.replace('            let selected = match candidate.radical_sum_sign(', '            eprintln!("selected-normal candidate={} half-plane-begin", candidate_index);\n            let selected = match candidate.radical_sum_sign(',1)
part=part.replace('            let location = match selected {', '            eprintln!("selected-normal candidate={} half-plane-end sign={:?}", candidate_index, selected);\n            let location = match selected {',1)
part=part.replace('            let tangent_cross_source = match candidate.radical_sum_sign(', '            eprintln!("selected-normal candidate={} tangent-begin", candidate_index);\n            let tangent_cross_source = match candidate.radical_sum_sign(',1)
part=part.replace('            let derivative_scale = match other.parallel_derivative_scale_sign(', '            eprintln!("selected-normal candidate={} tangent-end sign={:?}", candidate_index, tangent_cross_source);\n            let derivative_scale = match other.parallel_derivative_scale_sign(',1)
part=part.replace('            retained.push((candidate, location, tangent_cross_sign));', '            eprintln!("selected-normal candidate={} retained", candidate_index);\n            retained.push((candidate, location, tangent_cross_sign));',1)

t=t[:start]+part+t[end:]
start=t.index('    pub(crate) fn parallel_intersections(\n        &self,\n        other: &BezierParallel2,\n        range: &CurveParameterRange2,');insert=t.index('        let frame_parallel = self.source_parallel();',start)
t=t[:insert]+'        eprintln!("circle-parallel entry selected-normal={} chord-normal={} recursive={}", self.uses_selected_parallel_normal_frame(), self.uses_selected_chord_normal_frame(), self.uses_retained_circle_parallel_system());\n'+t[insert:]
start=t.index('fn selected_fiber_parameters_in_range(');end=t.index('fn selected_fiber_parameters_on_incident_ray(',start);part=t[start:end];original='return Ok(Classification::Uncertain(reason))'
for i in range(part.count(original)):
 part=part.replace(original, f'{{ eprintln!("selected-range unresolved-stage={i} reason={{reason:?}}"); RETURN_UNCERTAIN(reason) }}',1)
part=part.replace('RETURN_UNCERTAIN(reason)',original);t=t[:start]+part+t[end:]
start=t.index('fn selected_fiber_root_intervals_in_interval(');end=t.index('fn selected_fiber_parameters_in_interval(',start);part=t[start:end]
needle='    if report.certainty == PredicateCertainty::Approximate {'
part=part.replace(needle,'    eprintln!("fiber-isolation rows={} columns={} rational={} status={:?} message={:?} sturm={} subdivisions={} refinements={}", incidence.coefficients.len(), incidence.coefficients.iter().map(Vec::len).max().unwrap_or(0), incidence.coefficients.iter().flatten().all(|coefficient| coefficient.exact_rational_ref().is_some()), report.status, report.message, report.sturm_sequence_length, report.subdivision_steps, report.retained_refinement_steps);\n'+needle,1)
t=t[:start]+part+t[end:]

start=t.index('fn algebraic_selected_fiber_root_predicate_sign(');end=t.index('fn validate_selected_fiber_pair_base(',start);part=t[start:end]
marker=') -> CurveResult<Classification<RealSign>> {';assert marker in part
part=part.replace(marker,marker+r"""
    let probe = authority.data.incidence.coefficients.len() == 6 && authority.data.incidence.coefficients.iter().map(Vec::len).max() == Some(21);
    if probe { eprintln!("selected-predicate begin rows={} cols={}", predicate.coefficients.len(), predicate.coefficients.iter().map(Vec::len).max().unwrap_or(0)); }
""",1)
part=part.replace('    loop {\n        if refinement_steps != 0 {', '    loop {\n        if probe { eprintln!("selected-predicate iteration refinements={}", refinement_steps); }\n        if refinement_steps != 0 {',1)
part=part.replace('            let zero_report = count_bivariate_common_fiber_roots_at_algebraic_parameter(', '            if probe { eprintln!("selected-predicate common-fiber begin refinements={}", refinement_steps); }\n            let zero_report = count_bivariate_common_fiber_roots_at_algebraic_parameter(',1)
part=part.replace('            exact_nonzero = zero_report.certainty', '            if probe { eprintln!("selected-predicate common-fiber end status={:?} count={:?}", zero_report.status, zero_report.distinct_root_count); }\n            exact_nonzero = zero_report.certainty',1)
t=t[:start]+part+t[end:]
start=t.index('fn algebraic_selected_fiber_root_interval_refined(');end=t.index('fn algebraic_selected_fiber_root_predicate_sign(',start);part=t[start:end]
marker=') -> CurveResult<Classification<IsolatedRootInterval>> {';assert marker in part
part=part.replace(marker,marker+r"""
    let probe = authority.data.incidence.coefficients.len() == 6 && authority.data.incidence.coefficients.iter().map(Vec::len).max() == Some(21);
    if probe { eprintln!("selected-refine begin steps={}", refinement_steps); }
""",1)
part=part.replace('    match report.status {', '    if probe { eprintln!("selected-refine end status={:?} retained-refinements={}", report.status, report.retained_refinement_steps); }\n    match report.status {',1)
t=t[:start]+part+t[end:]


start=t.index('    fn chord_intersections_in_domain_once(');end=t.index('        // A selected parallel-normal circle with an algebraic center retains',start);part=t[start:end]
part=part.replace('        chord.validate_policy(policy)?;', '        eprintln!("circle-chord begin finite={} selected-parallel={} selected-chord={} selected-radial={}", clip_to_finite_chord, self.uses_selected_parallel_normal_frame(), self.uses_selected_chord_normal_frame(), self.uses_selected_radial_frame());\n        chord.validate_policy(policy)?;',1)
old='            let broad_phase_refinements: &[usize] = if self.uses_selected_chord_normal_frame() {\n                &[0, 2]\n            } else {\n                &[0]\n            };'
assert old in part;part=part.replace(old,old,1)
needle_bounds='                let chord_bounds = chord.conservative_bounds_refined(refinement_steps, policy)?;'
part=part.replace(needle_bounds,needle_bounds+'\n                eprintln!("circle-chord bounds steps={} circle={} chord={}", refinement_steps, matches!(&circle_bounds, Classification::Decided(_)), matches!(&chord_bounds, Classification::Decided(_)));',1)
part=part.replace('                    return Ok(Classification::Decided(Vec::new()));','                    eprintln!("circle-chord finite separation steps={}", refinement_steps);\n                    return Ok(Classification::Decided(Vec::new()));',1)
t=t[:start]+part+t[end:]

start=t.index('    fn chord_intersections_in_domain_once(');pos=t.index('        chord.validate_policy(policy)?;',start);t=t[:pos]+'        let mut certified_endpoint_incidence = certified_endpoint_incidence;\n'+t[pos:];pos=t.index('        if self.data.frame.rational().is_some() && chord.certified_axis_direction().is_some() {',start)
t=t[:pos]+r"""
        if clip_to_finite_chord {
            let mut inside = true;
            for (index, endpoint) in [chord.start(), chord.end()].into_iter().enumerate() {
                eprintln!("finite-disk begin endpoint={}", index);
                let mut incidence = self.retained_point_incidence_sign_by_refinement(endpoint, policy, 4, false)?;
                eprintln!("finite-disk endpoint={} incidence={:?}", index, incidence);
                if matches!(incidence, Classification::Uncertain(_)) && PROBE_SELECTED_CONTACT_READY.load(std::sync::atomic::Ordering::Relaxed) {
                    let kind = match &endpoint.0 {
                        CurvePointData2::Exact(_) => "exact",
                        CurvePointData2::Algebraic(_) => "algebraic",
                        CurvePointData2::AlgebraicCuspChord(_) => "cusp-chord",
                        CurvePointData2::AlgebraicCuspChordDerived(_) => "cusp-chord-derived",
                        CurvePointData2::AlgebraicChordParallel(_) => "chord-parallel",
                        CurvePointData2::AnalyticParallel(_) => "analytic-parallel",
                        CurvePointData2::Endpoint(_) => "endpoint",
                        _ => "other",
                    };
                    eprintln!("finite-endpoint proof begin endpoint={} kind={}", index, kind);
                    if let CurvePointData2::AlgebraicChordParallel(point) = &endpoint.0 {
                        let source_kind = match &point.source_endpoint().0 {
                            CurvePointData2::Exact(_) => "exact",
                            CurvePointData2::Algebraic(_) => "algebraic",
                            CurvePointData2::AlgebraicCuspChord(_) => "cusp-chord",
                            CurvePointData2::AlgebraicCuspChordDerived(_) => "cusp-chord-derived",
                            CurvePointData2::AlgebraicChordParallel(_) => "chord-parallel",
                            CurvePointData2::AnalyticParallel(_) => "analytic-parallel",
                            CurvePointData2::Endpoint(_) => "endpoint",
                            _ => "other",
                        };
                        let distance = point.data.distance.exact_rational_ref().map(|r| r.to_string()).map(|s| s.chars().take(80).collect::<String>());
                        eprintln!("finite-endpoint displacement origin={} normal={} source-kind={} distance={:?}", point.data.source_point.is_some(), point.data.direction == BezierAlgebraicChordUnitDisplacement2::LeftNormal, source_kind, distance);
                    }

                    incidence = policy.bounded_exact_predicate_pass(|| self.strict_point_incidence_sign(endpoint, policy))?;
                    eprintln!("finite-endpoint proof end endpoint={} incidence={:?}", index, incidence);
                }
                if incidence == Classification::Decided(RealSign::Zero) && certified_endpoint_incidence.is_none() {
                    certified_endpoint_incidence = Some(if index == 0 { BezierCertifiedFiniteChordEndpointIncidence2::Start } else { BezierCertifiedFiniteChordEndpointIncidence2::End });
                }
                if incidence != Classification::Decided(RealSign::Negative) { inside = false; }
            }
            if inside {
                eprintln!("finite-disk both endpoints strictly inside");
                return Ok(Classification::Decided(Vec::new()));
            }
            if certified_endpoint_incidence.is_some() && PROBE_SELECTED_CONTACT_READY.load(std::sync::atomic::Ordering::Relaxed) {
                eprintln!("finite-endpoint direct factored solver begin");
                match self.recursive_projective_retained_chord_intersections(chord, true, certified_endpoint_incidence, policy)? {
                    Classification::Decided(Some(intersections)) => {
                        eprintln!("finite-endpoint direct factored solver decided");
                        return self.retain_chord_intersections(chord, intersections, policy);
                    }
                    Classification::Decided(None) | Classification::Uncertain(_) => {
                        eprintln!("finite-endpoint direct factored solver declined");
                    }
                }
            }
        }
"""+t[pos:]
start=t.index('    fn recursive_projective_chord_intersections(');pos=t.index('        if clip_to_finite_chord {',start);t=t[:pos]+t[pos:].replace('        if clip_to_finite_chord {','        if clip_to_finite_chord && certified_endpoint_incidence.is_none() {',1)



t += '\nstatic PROBE_SELECTED_CONTACT_READY: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);\n'
start=t.index('fn selected_fiber_root_intervals_in_interval(');pos=t.index('    if report.certainty == PredicateCertainty::Approximate {',start)
t=t[:pos]+'    if incidence.coefficients.len() == 6 && incidence.coefficients.iter().map(Vec::len).max() == Some(21) { PROBE_SELECTED_CONTACT_READY.store(true, std::sync::atomic::Ordering::Relaxed); }\n'+t[pos:]

p.write_text(t);shutil.copy2(p,build/file);manifest[file]=hashlib.sha256(p.read_bytes()).hexdigest()
file='hypercurve/src/curve_region_boolean.rs';p=archive/file;t=p.read_text();needle='            if let Some(blocker) = result.blockers.first() {';pos=t.index(needle,t.index('    fn build_split_topology('))+len(needle)
t=t[:pos]+'\n                eprintln!("pair-blocker indices={}:{} loops={}:{} fragments={}:{}", pair.first_carrier_index, pair.second_carrier_index, self.data.carriers[pair.first_carrier_index].loop_index, self.data.carriers[pair.second_carrier_index].loop_index, self.data.carriers[pair.first_carrier_index].fragment_index, self.data.carriers[pair.second_carrier_index].fragment_index);'+t[pos:]
p.write_text(t);shutil.copy2(p,build/file);manifest[file]=hashlib.sha256(p.read_bytes()).hexdigest()

file='hypercurve/src/curve_region_boolean.rs';p=archive/file;t=p.read_text();start=t.index('    fn build_split_topology(');pos=t.index('            let result = self.pair_result(pair)?;',start)
t=t[:pos]+'            eprintln!("pair-begin {}:{} loops={}:{} fragments={}:{} cusp-chord={}", pair.first_carrier_index, pair.second_carrier_index, self.data.carriers[pair.first_carrier_index].loop_index, self.data.carriers[pair.second_carrier_index].loop_index, self.data.carriers[pair.first_carrier_index].fragment_index, self.data.carriers[pair.second_carrier_index].fragment_index, matches!(&pair.context, RegionCarrierPairContext::CuspChord { .. }));\n'+t[pos:]
pos=t.index('            let result = self.pair_result(pair)?;',pos)+len('            let result = self.pair_result(pair)?;');t=t[:pos]+'\n            eprintln!("pair-end {}:{}", pair.first_carrier_index, pair.second_carrier_index);'+t[pos:]
p.write_text(t);shutil.copy2(p,build/file);manifest[file]=hashlib.sha256(p.read_bytes()).hexdigest()

(A/f'{prefix}-sources.json').write_text(json.dumps(manifest,indent=2)+'\n')
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
report=dict(source_manifest=f'{prefix}-sources.json',source_directory=str(archive),workspace_guard='center-locus-cusp-20260926-v260-sources.json',parent_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=W/'hypercurve',text=True).strip(),hypersolve_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=W/'hypersolve',text=True).strip(),cases=[],all_processes_reaped=False)
def verify():
 for name,sha in guard.items():assert hashlib.sha256((W/name).read_bytes()).hexdigest()==sha,name
 for name,sha in manifest.items():
  for root in [archive,build]:assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,name
verify();cmd=['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo','test','--test','hypercurve_curve_region_promotion','--release','--all-features','--no-run','--message-format=json','--locked','--offline'];report['command']=cmd
start=time.monotonic()
with(A/f'{prefix}-build.log').open('w')as log:code=subprocess.run(cmd,cwd=build/'hypercurve',env=env,stdout=log,stderr=subprocess.STDOUT,timeout=900).returncode
report['build_returncode']=code;report['build_elapsed_seconds']=time.monotonic()-start;verify();assert code==0
rows=[]
for line in(A/f'{prefix}-build.log').read_text().splitlines():
 try:rows.append(json.loads(line))
 except ValueError:pass
row=next(r for r in rows if r.get('reason')=='compiler-artifact'and r['target']['name']=='hypercurve_curve_region_promotion'and r.get('executable'))
binary=A/f'{prefix}-hypercurve_curve_region_promotion';shutil.copy2(row['executable'],binary);report['binary']=dict(path=str(binary),sha256=hashlib.sha256(binary.read_bytes()).hexdigest())
for name in ['approximate_512_trim_or_extend_analytic_parallel_support_corners_retain_algebraic_fillet_extensions']:
 log=A/f'{prefix}-{name}.log';start=time.monotonic()
 with log.open('w')as out:
  try:code=subprocess.run([str(binary),'--exact',name,'--nocapture','--test-threads=1'],cwd=archive/'hypercurve',env=env,stdout=out,stderr=subprocess.STDOUT,timeout=60).returncode
  except subprocess.TimeoutExpired:code=124
 report['cases'].append(dict(name=name,returncode=code,log=log.name,elapsed_seconds=time.monotonic()-start));print(name,code,log.read_text()[-2500:],flush=True)
verify();report['all_sources_unchanged']=True;report['all_processes_reaped']=True;(A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
print('Diagnostic audit complete; case failures are reported individually.',flush=True)
