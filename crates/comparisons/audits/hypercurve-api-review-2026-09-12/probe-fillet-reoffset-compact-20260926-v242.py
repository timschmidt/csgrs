from pathlib import Path
import hashlib,json,os,shutil,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent
prefix='fillet-reoffset-compact-20260926-v242';prior=json.loads((A/'public-fillet-families-full-20260926-v236-terminal.json').read_text())
guard=json.loads((A/'public-fillet-families-full-20260926-v236-sources.json').read_text());manifest=json.loads((A/prior['source_manifest']).read_text());archive=A/'source-archives'/prefix;build=A/'build-workspace-20260925';assert not archive.exists()
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
start=t.index('fn selected_fiber_root_intervals_in_interval(');pos=t.index('    if report.certainty == PredicateCertainty::Approximate {',start)
export_code=r"""    if report.status == AlgebraicFiberRootIsolationStatus::Undecided
        && let Some(path) = std::env::var_os("HYPERCURVE_FIBER_EXPORT")
    {
        let mut lines = vec![String::from("HFIBER1"), retained_root.polynomial_coefficients.len().to_string()];
        lines.extend(retained_root.polynomial_coefficients.iter().map(|value| value.__probe_compact().expect("coefficient tower").to_json()));
        lines.push(retained_root.interval.lower.__probe_compact().expect("lower tower").to_json());
        lines.push(retained_root.interval.upper.__probe_compact().expect("upper tower").to_json());
        lines.push(usize::from(retained_root.interval.exact_root.is_some()).to_string());
        if let Some(root) = &retained_root.interval.exact_root { lines.push(root.__probe_compact().expect("root tower").to_json()); }
        lines.push(lower.__probe_compact().expect("fiber lower tower").to_json());
        lines.push(upper.__probe_compact().expect("fiber upper tower").to_json());
        lines.push(incidence.coefficients.len().to_string());
        for row in &incidence.coefficients {
            lines.push(row.len().to_string());
            lines.extend(row.iter().map(|value| value.__probe_compact().expect("coefficient tower").to_json()));
        }
        let size: usize = lines.iter().map(|line| line.len()+1).sum();
        assert!(size <= 4*1024*1024, "bounded scalar fiber export: {size} bytes");
        std::fs::write(path, lines.join("\n") + "\n").expect("write exact scalar fiber fixture");
        eprintln!("exported-fiber bytes={size}");
    }
"""
t=t[:pos]+export_code+t[pos:]
p.write_text(t);shutil.copy2(p,build/file);manifest[file]=hashlib.sha256(p.read_bytes()).hexdigest()
file='hypercurve/src/curve_region_boolean.rs';p=archive/file;t=p.read_text();needle='            if let Some(blocker) = result.blockers.first() {';pos=t.index(needle,t.index('    fn build_split_topology('))+len(needle)
t=t[:pos]+'\n                eprintln!("pair-blocker indices={}:{} loops={}:{} fragments={}:{}", pair.first_carrier_index, pair.second_carrier_index, self.data.carriers[pair.first_carrier_index].loop_index, self.data.carriers[pair.second_carrier_index].loop_index, self.data.carriers[pair.first_carrier_index].fragment_index, self.data.carriers[pair.second_carrier_index].fragment_index);'+t[pos:]
p.write_text(t);shutil.copy2(p,build/file);manifest[file]=hashlib.sha256(p.read_bytes()).hexdigest()

def patch_source(file, transform):
 p=archive/file;t=transform(p.read_text());p.write_text(t);shutil.copy2(p,build/file);manifest[file]=hashlib.sha256(p.read_bytes()).hexdigest()
patch_source('hyperreal/src/computable/node/quadratic_tower.rs', lambda t: t + r"""
impl Computable {
    pub(crate) fn __probe_tower(&self) -> Option<[[Rational; 3]; 3]> {
        let tower = tower_from_computable(self)?;
        let quad = |q: Quad| [q.rational, q.scale, q.disc.unwrap_or_else(Rational::zero)];
        Some([quad(tower.even), quad(tower.odd), quad(tower.radicand.unwrap_or_else(Quad::zero))])
    }
}
""")
patch_source('hyperreal/src/real/arithmetic/quadratic_tower_sign.rs', lambda t: t + r"""
impl Real {
    pub fn __probe_compact(&self) -> Option<Self> {
        let parts = self.tower_computable().__probe_tower()?;
        let quad = |[a,b,d]: [Rational; 3]| {
            let a = Self::new(a);
            if b.is_zero() { a } else { a + Self::new(b) * Self::new(d).sqrt().unwrap() }
        };
        let [even, odd, radicand] = parts;
        let even = quad(even);
        let odd = quad(odd);
        Some(if odd.definitely_zero() { even } else { even + odd * quad(radicand).sqrt().unwrap() })
    }
}
""")
def trace_field(t):
 t=t.replace('Err(LocalFieldError::Undecided)', '{ eprintln!("local-field undecided line={}", line!()); Err(LocalFieldError::Undecided) }')
 t=t.replace('.ok_or(LocalFieldError::Undecided)', '.ok_or_else(|| { eprintln!("local-field missing line={}", line!()); LocalFieldError::Undecided })')
 t=t.replace('fn consume<T>', '#[track_caller]\n    fn consume<T>')
 t=t.replace('PredicateOutcome::Unknown { .. } =>', 'PredicateOutcome::Unknown { .. } => { eprintln!("consume unknown caller={}", std::panic::Location::caller());')
 t=t.replace('Err(LocalFieldError::Undecided) },\n        }\n    }\n\n    fn compare', 'Err(LocalFieldError::Undecided) } },\n        }\n    }\n\n    fn compare')
 t=t.replace('Err(error) => {\n                return fiber_root_isolation_error_report_with_progress(', 'Err(error) => {\n                eprintln!("fiber failed line={} error={:?}", line!(), error);\n                return fiber_root_isolation_error_report_with_progress(')
 return t
patch_source('hypersolve/src/algebraic_fiber.rs', trace_field)

(A/f'{prefix}-sources.json').write_text(json.dumps(manifest,indent=2)+'\n')
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
env['HYPERCURVE_FIBER_EXPORT']=str(A/f'{prefix}-fiber.jsonl')
report=dict(source_manifest=f'{prefix}-sources.json',source_directory=str(archive),workspace_guard='public-fillet-families-full-20260926-v236-sources.json',parent_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=W/'hypercurve',text=True).strip(),cases=[],all_processes_reaped=False)
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
  try:code=subprocess.run([str(binary),'--exact',name,'--nocapture','--test-threads=1'],cwd=archive/'hypercurve',env=env,stdout=out,stderr=subprocess.STDOUT,timeout=180).returncode
  except subprocess.TimeoutExpired:code=124
 report['cases'].append(dict(name=name,returncode=code,log=log.name,elapsed_seconds=time.monotonic()-start));print(name,code,log.read_text()[-2500:],flush=True)
verify();report['all_sources_unchanged']=True;report['all_processes_reaped']=True;(A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
fixture=A/f'{prefix}-fiber.jsonl'
assert fixture.exists()
print('Exported exact scalar fixture:',fixture.stat().st_size,'bytes; sha256',hashlib.sha256(fixture.read_bytes()).hexdigest(),flush=True)
