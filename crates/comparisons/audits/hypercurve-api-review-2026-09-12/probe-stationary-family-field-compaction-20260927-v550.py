from pathlib import Path
import hashlib,json,os,shutil,signal,subprocess,time
A=Path(__file__).resolve().parent;W=A.parent;prefix='stationary-family-field-compaction-20260927-v550';archive=A/'source-archives'/prefix;build=A/'build-workspace-20260925'
prior=json.loads((A/'fillet-endpoint-bridge-20260927-v548-terminal.json').read_text());assert prior['qualification_complete'] and prior['all_processes_reaped']
assert (A/'fillet-endpoint-bridge-20260927-v548-committed.json').exists()
guard=json.loads((A/prior['source_manifest']).read_text());manifest={};production={};assert not archive.exists()
for name,sha in guard.items():
 src=W/name;current=hashlib.sha256(src.read_bytes()).hexdigest();assert current==sha,name;production[name]=current
 data=src.read_bytes()
 if name=='hypersolve/src/root_isolation.rs':
  source=data.decode()
  a=source.index('fn sign_preserving_primitive_polynomial(');b=source.index('\nfn sturm_count(',a);part=source[a:b]
  needle='    let polynomial = trim_polynomial(polynomial, policy)?;'
  assert part.count(needle)==1
  part=part.replace(needle, '''    let mut compacted = 0;
    let mut nonrational = 0;
    let polynomial = polynomial.into_iter().map(|coefficient| {
        if coefficient.exact_rational_ref().is_none() {
            nonrational += 1;
            if let Some(compact) = coefficient.compact_quadratic_tower() {
                compacted += 1;
                return compact;
            }
        }
        coefficient
    }).collect::<Vec<_>>();
    if nonrational != 0 {
        eprintln!("V550 compact Sturm member degree={} nonrational={} compacted={}", polynomial.len()-1, nonrational, compacted);
    }
    let polynomial = trim_polynomial(polynomial, policy)?;''')
  source=source[:a]+part+source[b:];data=source.encode()
 elif name=='hypercurve/src/error.rs':
  source=data.decode()
  source=source.replace('    pub(crate) const fn blocked(', '    #[track_caller]\n    pub(crate) fn blocked(')
  source=source.replace('        Self::Blocked(ExactCurveBlocker::new(operation, family, reason))', '        eprintln!("V550 blocked at {} op={:?} family={:?} reason={:?}", std::panic::Location::caller(), operation, family, reason);\n        Self::Blocked(ExactCurveBlocker::new(operation, family, reason))')
  source=source.replace('    pub(crate) fn with_operation(', '    #[track_caller]\n    pub(crate) fn with_operation(')
  data=source.encode()
 elif name=='hypercurve/src/curve_region_boolean.rs':
  source=data.decode().replace('    fn blocked(&self, carrier_index:', '    #[track_caller]\n    fn blocked(&self, carrier_index:')
  source=source.replace('            if let Some(blocker) = result.blockers.first() {', '''            if let Some(blocker) = result.blockers.first() {
                eprintln!("V550 pair blocker indices=({}, {}) context={:?} supports=({:?},{:?}) families=({:?},{:?}) adjacent={} reason={:?}",
                    pair.first_carrier_index, pair.second_carrier_index,
                    std::mem::discriminant(&pair.context),
                    std::mem::discriminant(&self.data.carriers[pair.first_carrier_index].geometry),
                    std::mem::discriminant(&self.data.carriers[pair.second_carrier_index].geometry),
                    self.data.carriers[pair.first_carrier_index].family,
                    self.data.carriers[pair.second_carrier_index].family,
                    self.authored_carriers_are_adjacent(pair), std::mem::discriminant(blocker));''')
  a=source.index('    fn parallel_arc_pair_result(');b=source.index('    fn parallel_exact_parameter_pair_result(',a);part=source[a:b]
  for reason in ['reason','UncertaintyReason::Unsupported','UncertaintyReason::Boundary']:
   token='return Ok(Classification::Uncertain('+reason+'))'
   part=part.replace(token,'return { eprintln!("V550 arc incidence line={} reason={:?}",line!(),'+reason+'); Ok(Classification::Uncertain('+reason+')) }')
  part=part.replace('return Ok(Classification::Decided(None))','return { eprintln!("V550 arc incidence declined line={}",line!()); Ok(Classification::Decided(None)) }')
  source=source[:a]+part+source[b:]
  data=source.encode()
 elif name in ['hypercurve/src/bezier_parameter.rs','hypercurve/src/bezier_split.rs']:
  source=data.decode()
  for reason in ['reason','UncertaintyReason::RealSign','UncertaintyReason::Ordering','UncertaintyReason::Unsupported']:
   token='return Ok(Classification::Uncertain('+reason+'))'
   source=source.replace(token,'return { eprintln!("V550 root source={} line={} reason={:?}",file!(),line!(),'+reason+'); Ok(Classification::Uncertain('+reason+')) }')
  if name=='hypercurve/src/bezier_parameter.rs':
   source=source.replace('None => Classification::Uncertain(UncertaintyReason::RealSign),','None => { eprintln!("V550 root classification unavailable line={}",line!()); Classification::Uncertain(UncertaintyReason::RealSign) },')
  data=source.encode()
 elif name=='hypercurve/src/bezier_offset.rs':
  source=data.decode()
  for method in ['circle_incidence_with_tangent_field','rational_quadratic_circle_intersections_fast_path_with_tangent_field','intersections_on_regular_range','intersections_with_tangent_field','intersection_candidate_system_with_tangent_field']:
   import re
   match=re.search(r'    (?:pub\(crate\) )?fn '+method+r'\(',source);assert match,method
   a=match.start();end=re.search(r'\n    (?:pub\(crate\) )?fn ',source[match.end():]);assert end,method
   b=match.end()+end.start();part=source[a:b]
   for reason in ['reason','UncertaintyReason::Unsupported','UncertaintyReason::Boundary']:
    token='return Ok(Classification::Uncertain('+reason+'))'
    part=part.replace(token, 'return { eprintln!("V550 incidence '+method+' line={} reason={:?}", line!(), '+reason+'); Ok(Classification::Uncertain('+reason+')) }')
   part=part.replace('return Ok(no_fast_path())','return { eprintln!("V550 circle fast decline line={}",line!()); Ok(no_fast_path()) }')
   source=source[:a]+part+source[b:]
  source=source.replace('        let candidate_polynomial = match polynomial_from_coefficients(eliminant, policy)? {', '        eprintln!("V550 circle polynomial degree={} certified={} primitive={} center_rational=({}, {})", eliminant.len()-1, certified_parameters.len(), tangent_field.is_some(), center.x().exact_rational_ref().is_some(), center.y().exact_rational_ref().is_some());\n        let candidate_polynomial = match polynomial_from_coefficients(eliminant, policy)? {')
  data=source.encode()
 elif name=='hypercurve/src/bezier_region.rs':
  source=data.decode()
  for reason in ['reason','UncertaintyReason::Unsupported','UncertaintyReason::Boundary']:
   token='return Ok(Classification::Uncertain('+reason+'))'
   source=source.replace(token, 'return { eprintln!("V550 region uncertain line={} reason={:?}", line!(), '+reason+'); Ok(Classification::Uncertain('+reason+')) }')
  data=source.encode()
 elif name=='hypercurve/src/curve_fillet.rs':
  fixture=(A/'stationary-family-composition-v541.rs').read_text()
  fixture=fixture.replace('                        let offset = chamfered', '                        eprintln!("V550 before round offset loops={}", chamfered.boundary_loops().len());\n                        let offset = chamfered')
  fixture=fixture.replace('                        let offset = chamfered', '                        hyperreal::dispatch_trace::reset();\n                        let _trace = hyperreal::dispatch_trace::recording_scope();\n                        let offset = chamfered')
  fixture=fixture.replace('                            .offset(q(1, 4096), &crate::OffsetCornerStyle2::Round, &policy)\n                            .unwrap();', '                            .offset(q(1, 4096), &crate::OffsetCornerStyle2::Round, &policy);\n                        if offset.is_err() { for event in hyperreal::dispatch_trace::snapshot().into_iter().take(500) { eprintln!("V550 trace {event:?}"); } }\n                        let offset = offset.unwrap();')
  data+=b'\n'+fixture.encode()
 manifest[name]=hashlib.sha256(data).hexdigest()
 dst=archive/name;dst.parent.mkdir(parents=True,exist_ok=True);dst.write_bytes(data)
 target=build/name
 if target.read_bytes()!=data:target.write_bytes(data);os.utime(target,None)
(A/f'{prefix}-sources.json').write_text(json.dumps(manifest,indent=2)+'\n')
(A/f'{prefix}-production-sources.json').write_text(json.dumps(production,indent=2)+'\n')
names={'hypercurve':['curve::curve_fillet::stationary_family_composition_regression::normalized_region_selects_and_reuses_a_stationary_fillet_family']}
report=dict(normal_production_build=False,source_manifest=f'{prefix}-sources.json',source_directory=str(archive),builds=[],cases=[],checks=[],expected_cases=names,all_processes_reaped=False,probe_complete=False,qualification_complete=False)
fixture_sha=hashlib.sha256((A/'stationary-family-composition-v541.rs').read_bytes()).hexdigest()
env=dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()));cargo='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
def verify():
 assert hashlib.sha256((A/'stationary-family-composition-v541.rs').read_bytes()).hexdigest()==fixture_sha
 for name,sha in manifest.items():
  for root in [archive,build]:assert hashlib.sha256((root/name).read_bytes()).hexdigest()==sha,(root,name)
 for name,sha in production.items():assert hashlib.sha256((W/name).read_bytes()).hexdigest()==sha,(W,name)
def save():(A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
def run(command,cwd,log,timeout):
 start=time.monotonic()
 with log.open('w')as out:
  process=subprocess.Popen(command,cwd=cwd,stdout=out,stderr=subprocess.STDOUT,env=env,start_new_session=True)
  try:code=process.wait(timeout=timeout)
  except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait();code=124
  except BaseException:os.killpg(process.pid,signal.SIGKILL);process.wait();raise
 return dict(command=command,returncode=code,log=log.name,elapsed_seconds=time.monotonic()-start)
verify();save();code=0
try:
 log=A/f'{prefix}-build.log';row=run([cargo,'test','--lib','--release','--all-features','--locked','--offline','--no-run','--message-format=json'],build/'hypercurve',log,900);report['builds'].append(row);save()
 if row['returncode']:
  for line in log.read_text().splitlines():
   try:v=json.loads(line)
   except ValueError:continue
   if v.get('reason')=='compiler-message'and v['message']['level']=='error':print(v['message'].get('rendered','')[:3000],flush=True)
  raise RuntimeError('fixture build failed')
 artifacts={}
 for line in log.read_text().splitlines():
  try:v=json.loads(line)
  except ValueError:continue
  target=v.get('target',{}).get('name')
  if v.get('reason')=='compiler-artifact' and target in names and v.get('executable'):artifacts[target]=v
 assert set(artifacts)==set(names)
 row['binaries']={}
 for target,artifact in artifacts.items():
  binary=A/f'{prefix}-{target}-tests';shutil.copy2(artifact['executable'],binary);row['binaries'][target]=dict(path=str(binary),sha256=hashlib.sha256(binary.read_bytes()).hexdigest());save()
 for target,cases in names.items():
  binary=Path(report['builds'][0]['binaries'][target]['path'])
  for name in cases:
   log=A/f'{prefix}-{name.split("::")[-1]}.log';case=run([str(binary),'--exact',name,'--nocapture','--test-threads=1'],archive/'hypercurve',log,240);case.update(target=target,name=name,passed=case['returncode']==0 and 'test result: ok. 1 passed;'in log.read_text());report['cases'].append(case);save();print(name,case['returncode'],round(case['elapsed_seconds'],3),flush=True)
   if not case['passed']:print(log.read_text()[-1800:],flush=True)
 report['probe_complete']=True;code=0 if all(row['passed']for row in report['cases'])else 1
 if code:raise RuntimeError('focused regression failure')
 report['qualification_complete']=False
except Exception as error:code=1;report['failure']=str(error)
finally:verify();report['all_processes_reaped']=True;save()
raise SystemExit(code)
