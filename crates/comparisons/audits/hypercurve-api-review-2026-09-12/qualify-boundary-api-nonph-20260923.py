from pathlib import Path
import concurrent.futures,hashlib,json,os,re,shutil,subprocess,time

a=Path(__file__).resolve().parent; w=a.parent; root=Path('/tmp/hypercurve-boundary-api-2026-09-23'); parent=Path('/tmp/hypercurve-closure-2026-09-23/hypercurve')
prefix='boundary-api-20260923-nonph'
assert json.loads((a/'boundary-api-20260923-runtime1-terminal.json').read_text())['hyperbrep_failures']==[]
s=(root/'hypercurve/tests/hypercurve_curve_region_promotion.rs').read_text()
def function(name):
 m=re.search(r'(?m)^fn '+re.escape(name)+r'(?:\(|<)',s); assert m,name
 end=s.index('\n}\n',m.start())+3; return s[m.start():end]
body=function('non_ph_bezier_pair_projective_fillet_retains_algebraic_extensions').replace('fn non_ph_bezier_pair_projective_fillet_retains_algebraic_extensions()', 'fn main()')
body=body.replace('''                let fragments = candidate.boundary_loops()[0].curves();''','''                let paths = candidate.boundary_paths(&policy).expect("exact boundary paths");
                assert_eq!(paths.certainty, CurveCertainty::Certified);
                let Classification::Decided(paths) = paths.value else { panic!("retained paths must be decided"); };
                let fragments = paths[0].curves();
                eprintln!("candidate curves: {:?}", fragments.iter().map(|curve| (curve.family(), curve.geometry().is_some(), curve.parameter_domain().scalar_endpoints().is_some())).collect::<Vec<_>>());''')
body=body.replace('''        for reversed in [false, true] {''','''        for reversed in [false, true] {
            eprintln!("case policy={policy:?}, reversed={reversed}");''')
body=body.replace('''            let result = source''','''            eprintln!("admitted; begin fillet");
            let result = source''')
body=body.replace('''            let candidates = match result.into_value() {''','''            eprintln!("fillet candidates={}", result.value.candidate_count());
            let candidates = match result.into_value() {''')
body=body.replace('''            assert_eq!(
                certified(filleted.classify_point''','''            eprintln!("selected candidate; begin point classification");
            assert_eq!(
                certified(filleted.classify_point''')
body=body.replace('''            let distant =''','''            eprintln!("point classified; admit distant region");
            let distant =''')
body=body.replace('''            let replay = filleted''','''            eprintln!("begin Boolean reentry");
            let replay = filleted''')
body=body.replace('''            assert_eq!(replay.union().boundary_loops().len(), 2);''','''            assert_eq!(replay.union().boundary_loops().len(), 2);
            eprintln!("case complete");''')
probe='use hypercurve::*;\n'+ '\n'.join(function(n) for n in ['p','q','square','certified'])+'\n'+body+'\n'
(a/(prefix+'-probe.rs')).write_text(probe)
roots={'parent':parent,'candidate':root/'hypercurve'}
name='boundary_nonph_reentry_probe_20260923'
for repo in roots.values():
 path=repo/'examples'/(name+'.rs'); assert not path.exists(); path.write_text(probe)
binding=json.loads((a/'boundary-api-20260923-check5-sources.json').read_text()); parent_binding=json.loads((a/'winding-contact-reuse-20260923-final1-sources.json').read_text())
def verify():
 for file,h in binding['working'].items(): assert hashlib.sha256((w/file).read_bytes()).hexdigest()==h,file
 for file,h in binding['isolated'].items(): assert hashlib.sha256((root/file).read_bytes()).hexdigest()==h,file
 for row in parent_binding['isolated']: assert hashlib.sha256((parent.parent/row['file']).read_bytes()).hexdigest()==row['sha256'],row['file']
 for repo in roots.values(): assert (repo/'examples'/(name+'.rs')).read_text()==probe
verify()
env=dict(os.environ,**json.loads((a/'opposed-endpoint-contact-full1-build-settings.json').read_text())); cargo='/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'; binaries={}; builds=[]
for variant,repo in roots.items():
 cmd=[cargo,'build','--release','--all-features','--example',name,'--message-format=json','--locked','--offline']; start=time.monotonic(); log=a/(prefix+'-'+variant+'-build.log'); out=a/(prefix+'-'+variant+'-build.jsonl')
 with out.open('w') as stdout,log.open('w') as stderr: code=subprocess.run(cmd,cwd=repo,env=env,stdout=stdout,stderr=stderr,timeout=900).returncode
 verify(); builds.append(dict(variant=variant,command=cmd,returncode=code,elapsed_seconds=time.monotonic()-start)); print('probe build',builds[-1],flush=True); assert code==0,log.read_text()[-5000:]
 for line in out.read_text().splitlines():
  row=json.loads(line)
  if row.get('reason')=='compiler-artifact' and row.get('executable') and row['target']['name']==name:
   assert not row['fresh']; binary=a/(prefix+'-'+variant+'-probe'); shutil.copy2(row['executable'],binary); binaries[variant]=binary
 assert variant in binaries

def run(variant):
 binary=binaries[variant]; log=a/(prefix+'-'+variant+'.log'); start=time.monotonic()
 with log.open('w') as out:
  try:code=subprocess.run([str(binary)],cwd=roots[variant],env=env,stdout=out,stderr=subprocess.STDOUT,timeout=75).returncode
  except subprocess.TimeoutExpired:code='timeout'
 row=dict(variant=variant,returncode=code,elapsed_seconds=time.monotonic()-start,binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),log=log.name)
 print(json.dumps(row),flush=True); print(log.read_text(),flush=True); return row
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool: results=list(pool.map(run,roots))
verify(); report=dict(builds=builds,runs=results,probe_sha256=hashlib.sha256(probe.encode()).hexdigest(),all_production_and_working_sources_unchanged=True)
(a/(prefix+'-results.json')).write_text(json.dumps(report,indent=2)+'\n')
for repo in roots.values(): (repo/'examples'/(name+'.rs')).unlink()
print('Common API probe terminal; all child processes reaped and temporary example targets removed.',flush=True)
