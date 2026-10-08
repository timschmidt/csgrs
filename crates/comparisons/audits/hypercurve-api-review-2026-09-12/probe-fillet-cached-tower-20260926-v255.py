from pathlib import Path
import hashlib,json,os,shutil,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent
prefix='fillet-cached-tower-20260926-v255';prior=json.loads((A/'retained-fiber-refinement-20260926-v248-terminal.json').read_text())
guard=json.loads((A/'retained-fiber-refinement-20260926-v248-sources.json').read_text());manifest=json.loads((A/prior['source_manifest']).read_text());guard={name:sha for name,sha in guard.items()if not name.startswith('fiber-probe/')};manifest=dict(guard);archive=A/'source-archives'/prefix;build=A/'build-workspace-20260925';assert not archive.exists()
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


start=t.index('    fn recursive_projective_chord_intersections(');end=t.index('        let Some((anchor_x, anchor_y, anchor_denominator)) =',start);part=t[start:end]
old="""                let signs = [start_incidence, middle_incidence, end_incidence]
                    .map(|value| value.sign(&CurveContext::STRICT));
                let [
                    Ok(Classification::Decided(first)),
                    Ok(Classification::Decided(middle)),
                    Ok(Classification::Decided(last)),
                ] = signs
                else {
                    return Some(false);
                };
                Some(
                    first == middle
                        && middle == last
                        && matches!(first, RealSign::Negative | RealSign::Positive),
                )"""
new="""                eprintln!("finite-disjoint begin");
                let mut common_sign = None;
                for (index, value) in [start_incidence, middle_incidence, end_incidence].into_iter().enumerate() {
                    let sign = value.bounded_interval_sign(0..=16);
                    eprintln!("finite-disjoint index={index} sign={sign:?}");
                    let Some(sign @ (RealSign::Negative | RealSign::Positive)) = sign else {
                        return Some(false);
                    };
                    if common_sign.is_some_and(|prior| prior != sign) {
                        return Some(false);
                    }
                    common_sign = Some(sign);
                }
                Some(true)"""
assert old in part;part=part.replace(old,new,1);t=t[:start]+part+t[end:]
start=t.index('    fn recursive_projective_chord_intersections(');end=t.index('        let roots = if let Some((endpoint, derivative_sign)) = certified_endpoint {',start);part=t[start:end]
part=part.replace('        let a_sign = a.sign(&CurveContext::STRICT)?;', '        eprintln!("recursive-chord a-begin");\n        let a_sign = a.sign(&CurveContext::STRICT)?;\n        eprintln!("recursive-chord a-end={a_sign:?}");',1)
part=part.replace('        let discriminant_classification = match certified_endpoint {', '        eprintln!("recursive-chord discriminant-begin endpoint={}", certified_endpoint.is_some());\n        let discriminant_classification = match certified_endpoint {',1)
part=part.replace('        let discriminant_sign = match discriminant_classification {', '        eprintln!("recursive-chord discriminant-end={discriminant_classification:?}");\n        let discriminant_sign = match discriminant_classification {',1)
t=t[:start]+part+t[end:]

p.write_text(t);shutil.copy2(p,build/file);manifest[file]=hashlib.sha256(p.read_bytes()).hexdigest()
file='hypercurve/src/curve_region_boolean.rs';p=archive/file;t=p.read_text();needle='            if let Some(blocker) = result.blockers.first() {';pos=t.index(needle,t.index('    fn build_split_topology('))+len(needle)
t=t[:pos]+'\n                eprintln!("pair-blocker indices={}:{} loops={}:{} fragments={}:{}", pair.first_carrier_index, pair.second_carrier_index, self.data.carriers[pair.first_carrier_index].loop_index, self.data.carriers[pair.second_carrier_index].loop_index, self.data.carriers[pair.first_carrier_index].fragment_index, self.data.carriers[pair.second_carrier_index].fragment_index);'+t[pos:]
p.write_text(t);shutil.copy2(p,build/file);manifest[file]=hashlib.sha256(p.read_bytes()).hexdigest()

file='hyperreal/src/computable/node/quadratic_tower.rs';p=archive/file;t=p.read_text()
t += r"""
impl Computable {
    pub(crate) fn cached_quadratic_tower_value(&self) -> Option<Self> {
        fn quad(value: Quad) -> Option<Computable> {
            let rational = Computable::rational(value.rational);
            if value.scale.sign() == Sign::NoSign { return Some(rational); }
            Some(rational.add(Computable::rational(value.disc?).sqrt().multiply_rational(value.scale)))
        }
        let tower = self.internal.cache.quadratic_tower()?;
        let even = quad(tower.even)?;
        let mut value = if tower.odd.is_zero() { even } else {
            even.add(quad(tower.odd)?.multiply(quad(tower.radicand?)?.sqrt()))
        };
        value.signal = self.signal.clone();
        static PROBE_HITS: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
        let hits = PROBE_HITS.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        if hits == 0 || hits % 100000 == 99999 { eprintln!("cached-tower-enclosure hits={}", hits + 1); }
        Some(value)
    }
}
"""
p.write_text(t);shutil.copy2(p,build/file);manifest[file]=hashlib.sha256(p.read_bytes()).hexdigest()
file='hyperreal/src/real/arithmetic/facts.rs';p=archive/file;t=p.read_text()
needle='        let approximation = self.computable_clone().approx(precision);'
replacement='        let computable = self.computable_ref();\n        let approximation = computable.cached_quadratic_tower_value().unwrap_or_else(|| computable.clone()).approx(precision);'
assert needle in t;t=t.replace(needle,replacement,1)
p.write_text(t);shutil.copy2(p,build/file);manifest[file]=hashlib.sha256(p.read_bytes()).hexdigest()

(A/f'{prefix}-sources.json').write_text(json.dumps(manifest,indent=2)+'\n')
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
report=dict(source_manifest=f'{prefix}-sources.json',source_directory=str(archive),workspace_guard='retained-fiber-refinement-20260926-v248-sources.json',parent_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=W/'hypercurve',text=True).strip(),hypersolve_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=W/'hypersolve',text=True).strip(),cases=[],all_processes_reaped=False)
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
