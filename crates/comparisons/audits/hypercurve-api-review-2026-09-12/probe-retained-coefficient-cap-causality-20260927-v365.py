from pathlib import Path
import hashlib,json,os,shutil,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent;prefix='retained-coefficient-cap-causality-20260927-v365';prior=json.loads((A/'initial-endpoint-root-replay-20260927-v362-terminal.json').read_text());assert prior['all_processes_reaped'] and all(b['returncode']==0 for b in prior['builds'])
guard=json.loads((A/prior['source_manifest']).read_text());manifest=dict(guard);archive=A/'source-archives'/prefix;build=A/'build-workspace-20260925';assert not archive.exists()
for name,sha in manifest.items():
 src=W/name;assert hashlib.sha256(src.read_bytes()).hexdigest()==sha,name
 dst=archive/name;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,dst)
 if (build/name).read_bytes()!=src.read_bytes():shutil.copy2(src,build/name);os.utime(build/name,None)
 assert dst.stat().st_ino!=(build/name).stat().st_ino
def patch_source(file, transform):
 p=archive/file;t=transform(p.read_text());p.write_text(t);shutil.copy2(p,build/file);manifest[file]=hashlib.sha256(p.read_bytes()).hexdigest()

def instrument_coefficients(t):
 start=t.index('pub(crate) fn compact_exact_coefficients(');end=t.index('fn checked_coefficient_count(',start);part=t[start:end]
 part=part.replace('    for coefficient in &mut coefficients {', '    let count=coefficients.len();\n    let batch_start=std::time::Instant::now();\n    for (index,coefficient) in coefficients.iter_mut().enumerate() {\n        let start=std::time::Instant::now();',1)
 part=part.replace('        if let Some(compact) = coefficient.compact_quadratic_tower() {','        eprintln!("coefficient-compact begin count={count} index={index}");\n        if let Some(compact) = coefficient.compact_quadratic_tower() {',1)
 part=part.replace('    }\n    coefficients','        eprintln!("coefficient-compact end count={count} index={index} elapsed_us={}",start.elapsed().as_micros());\n    }\n    eprintln!("coefficient-batch count={count} elapsed_us={}",batch_start.elapsed().as_micros());\n    coefficients',1)
 return t[:start]+part+t[end:]
# Coefficient timings are already recorded by V354.
def instrument_tower(t):
 start=t.index('fn tower_from_computable(value: &Computable) -> Option<Tower> {');body=t.index('\n',start)+1
 insert="""    struct Work { start: std::time::Instant, visits: usize, ready: usize, cached: usize }
    impl Drop for Work { fn drop(&mut self) { let us=self.start.elapsed().as_micros(); if us>=1000 { eprintln!(\"tower-work visits={} reduced={} cached={} elapsed_us={us}\",self.visits,self.ready,self.cached); } } }
    let mut work=Work { start:std::time::Instant::now(),visits:0,ready:0,cached:0 };
"""
 t=t[:body]+insert+t[body:];end=t.index('impl Tower {',start);part=t[start:end]
 part=part.replace('    while let Some((current, ready)) = pending.pop() {','    while let Some((current, ready)) = pending.pop() {\n        work.visits+=1;\n        work.ready+=usize::from(ready);',1)
 part=part.replace('            if let Some(tower) = current.internal.cache.quadratic_tower() {','            if let Some(tower) = current.internal.cache.quadratic_tower() {\n                work.cached+=1;',1)
 return t[:start]+part+t[end:]
# Tower timings are already recorded by V354.

def instrument_tower_failure(t):
 start=t.index('fn admit_rational(');end=t.index('\n}',start)+2;part=t[start:end]
 part=part.replace('        None','        static REJECTS: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);\n        if REJECTS.fetch_add(1, std::sync::atomic::Ordering::Relaxed) < 100 {\n            eprintln!("tower-coefficient-limit numerator={} denominator={}", value.numerator().bits(), value.denominator().bits());\n        }\n        None',1)
 t=t[:start]+part+t[end:]
 start=t.index('fn tower_from_computable(');end=t.index('\nimpl Tower {',start);part=t[start:end]
 a=part.index('        let tower = match &current.internal.approximation {');b=part.index('        // A completed shared reduction',a)
 calculation=part[a:b];assert calculation.endswith('        };\n')
 calculation=calculation.replace('let tower = match','let tower = (|| Some(match',1)[:-len('        };\n')]+'        }))();\n        if tower.is_none() {\n            static REJECTS: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);\n            if REJECTS.fetch_add(1, std::sync::atomic::Ordering::Relaxed) < 200 {\n                let kind = match &current.internal.approximation {\n                    Approximation::Add(_, _) => "add", Approximation::Multiply(_, _) => "multiply",\n                    Approximation::Inverse(_) => "inverse", Approximation::Square(_) => "square",\n                    Approximation::Sqrt(_) | Approximation::NthRoot(_, 2) => "sqrt",\n                    _ => "other",\n                };\n                eprintln!("tower-operation-unavailable kind={kind}");\n            }\n        }\n        let tower=tower?;\n'
 part=part[:a]+calculation+part[b:]
 return t[:start]+part+t[end:]
patch_source('hyperreal/src/computable/node/quadratic_tower.rs',instrument_tower_failure)
# Diagnostic only: this larger limit is not a proposed production change.
patch_source('hyperreal/src/computable/node/quadratic_tower.rs',lambda t:t.replace('const TOWER_RATIONAL_BIT_LIMIT: u64 = 8_192;', 'const TOWER_RATIONAL_BIT_LIMIT: u64 = 65_536;',1))



def instrument_circle(t):
 start=t.index('    fn recursive_projective_chord_intersections(');end=t.index('        let roots = if let Some((endpoint, derivative_sign)) = certified_endpoint {',start);part=t[start:end]
 part=part.replace('        let parent_field = authority.field.clone();','        let parent_field = authority.field.clone();\n        eprintln!("circle-chord begin finite={} endpoint={} depth={}", clip_to_finite_chord, certified_endpoint_incidence.is_some(), parent_field.base_and_extension_path().1.len());',1)
 part=part.replace('        let a_sign = a.sign(&CurveContext::STRICT)?;', '        eprintln!("circle-chord a begin");\n        let a_sign = a.sign(&CurveContext::STRICT)?;\n        eprintln!("circle-chord a end {a_sign:?}");',1)
 part=part.replace('        let certified_endpoint = match certified_endpoint {','        eprintln!("circle-chord endpoint derivative begin");\n        let certified_endpoint = match certified_endpoint {',1)
 part=part.replace('        let discriminant_classification = match certified_endpoint {','        eprintln!("circle-chord discriminant begin seeded={}", certified_endpoint.is_some());\n        let discriminant_classification = match certified_endpoint {',1)
 part=part.replace('        let discriminant_sign = match discriminant_classification {','        eprintln!("circle-chord discriminant end {discriminant_classification:?}");\n        let discriminant_sign = match discriminant_classification {',1)
 return t[:start]+part+t[end:]
patch_source('hypercurve/src/bezier_offset.rs',instrument_circle)

def instrument_signs(t):
 start=t.index('pub fn sign_at_selected_tuple(');body=t.index(' {',start)+2
 end=t.index('\n}\n',body)
 original=t[body:end]
 intro='\n    eprintln!("selected-tuple begin dimensions={:?} roots={:?}", polynomial.dimensions(), sources.iter().map(|s| (s.polynomial_coefficients.len(), s.exact_point_witness().is_some())).collect::<Vec<_>>());\n    let result = (|| {'
 tail='\n    })();\n    eprintln!("selected-tuple end {result:?}");\n    result'
 t=t[:body]+intro+original+tail+t[end:]
 start=t.index('pub fn sign_at_selected_root(');body=t.index(' {',start)+2;end=t.index('\n}\n',body);original=t[body:end]
 intro='\n    eprintln!("selected-root begin defining={} predicate={}", defining.len(), predicate.len());\n    let result = (|| {'
 tail='\n    })();\n    eprintln!("selected-root end {result:?}");\n    result'
 t=t[:body]+intro+original+tail+t[end:]
 t=t.replace('    let product_len = defining', '    eprintln!("selected-root endpoints admitted");\n    let product_len = defining',1)
 start=t.index('fn field_signed_sequence(');end=t.index('\nfn variations(',start);part=t[start:end]
 part=part.replace('    loop {','    loop {\n        eprintln!("selected-root sequence first={} second={}",first.len(),second.len());',1)
 return t[:start]+part+t[end:]
patch_source('hypersolve/src/root_sign.rs',instrument_signs)

def instrument_field_stage(t):
 start=t.index('fn field_signed_sequence(');end=t.index('\nfn variations(',start);part=t[start:end]
 part=part.replace('        sign(second.last()?)?;', '        let leading=sign(second.last()?);\n        eprintln!("selected-root leading={leading:?}");\n        leading?;',1)
 part=part.replace('        let remainder = polynomial_div_rem(first, &second, PredicatePolicy::STRICT)?.1;', '        let division=polynomial_div_rem(first, &second, PredicatePolicy::STRICT);\n        eprintln!("selected-root division={}",division.is_some());\n        let remainder=division?.1;',1)
 return t[:start]+part+t[end:]
patch_source('hypersolve/src/root_sign.rs',instrument_field_stage)

def instrument_tuple(t):
 start=t.index('fn dense_polynomial_tuple_sign_owned(');end=t.index('\nfn dense_two_positive_square_root_interval_with_coefficient_precision(',start);part=t[start:end]
 part=part.replace('    let Some((polynomial, sources)) =','    eprintln!("tuple-sign begin dimensions={:?} roots={:?}", polynomial.dimensions(), sources.iter().map(|s| (s.polynomial_coefficients.len(), s.exact_point_witness().is_some())).collect::<Vec<_>>());\n    let Some((polynomial, sources)) =',1)
 part=part.replace('    loop {','    loop {\n        eprintln!("tuple-sign refinement={refinement_steps}");',1)
 t=t[:start]+part+t[end:]
 start=t.index('    fn recursive_projective_chord_intersections(');end=t.index('        let roots = if let Some((endpoint, derivative_sign)) = certified_endpoint {',start);part=t[start:end]
 part=part.replace('        let parent_field = authority.field.clone();','        let parent_field = authority.field.clone();\n        let base=parent_field.base_and_extension_path().0;\n        eprintln!("circle-chord sources={:?} witnesses={:?}", base.sources.iter().map(|s| (s.polynomial_coefficients.len(),s.interval.exact_root.is_some())).collect::<Vec<_>>(), base.source_real_witnesses.iter().map(Option::is_some).collect::<Vec<_>>());',1)
 return t[:start]+part+t[end:]
patch_source('hypercurve/src/bezier_offset.rs',instrument_tuple)

(A/f'{prefix}-sources.json').write_text(json.dumps(manifest,indent=2)+'\n')
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
report=dict(source_manifest=f'{prefix}-sources.json',source_directory=str(archive),workspace_guard=prior['source_manifest'],parent_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=W/'hypercurve',text=True).strip(),hypersolve_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=W/'hypersolve',text=True).strip(),cases=[],all_processes_reaped=False)
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
for name in ['strict_trim_or_extend_analytic_parallel_support_corners_retain_algebraic_fillet_extensions']:
 log=A/f'{prefix}-{name}.log';start=time.monotonic()
 with log.open('w')as out:
  try:code=subprocess.run([str(binary),'--exact',name,'--nocapture','--test-threads=1'],cwd=archive/'hypercurve',env=env,stdout=out,stderr=subprocess.STDOUT,timeout=240).returncode
  except subprocess.TimeoutExpired:code=124
 report['cases'].append(dict(name=name,returncode=code,log=log.name,elapsed_seconds=time.monotonic()-start));print(name,code,log.read_text()[-2500:],flush=True)
verify();report['all_sources_unchanged']=True;report['all_processes_reaped']=True;(A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
print('Diagnostic audit complete; case failures are reported individually.',flush=True)
