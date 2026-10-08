from pathlib import Path
import hashlib, json, os, re, shutil, subprocess, sys, time

A = Path(__file__).resolve().parent
W = A.parent
prefix = 'cusp-finite-chart-20260926-v169'
guard_name = 'local-chord-complete-replay-20260924-v168-sources.json'
guard = json.loads((A/guard_name).read_text())
original = guard
archive = A/'source-archives'/prefix
assert not archive.exists()
for name,sha in original.items():
    source,destination = W/name,archive/name
    assert hashlib.sha256(source.read_bytes()).hexdigest() == sha
    destination.parent.mkdir(parents=True,exist_ok=True)
    shutil.copy2(source,destination)
p=archive/'hypercurve/src/curve_region_boolean.rs'
s=p.read_text();a=s.index('    fn parallel_arc_pair_result(');b=s.index('\n    fn ',a+5);body=s[a:b]
body=body.replace('return Ok(Classification::Decided(None));', 'eprintln!("parallel arc declined at line {}", line!()); return Ok(Classification::Decided(None));')
body=body.replace('Classification::Uncertain(reason) => {', 'Classification::Uncertain(reason) => { eprintln!("parallel arc uncertain {:?} at line {}", reason, line!());')
body=body.replace('        let parameters = incidence;', '        eprintln!("parallel arc incidence count={}", incidence.len());\n        let parameters = incidence;')
special = """Classification::Uncertain(_) => {
                            return Ok(Classification::Decided(None));
                        }"""
# The prior generic decline logging has already changed this arm.
special = special.replace('return Ok(Classification::Decided(None));', 'eprintln!("parallel arc declined at line {}", line!()); return Ok(Classification::Decided(None));')
assert body.count(special)==1
body=body.replace(special, """Classification::Uncertain(reason) => {
                            if let Some(point) = point.coordinates() {
                                let approximate = |point: &crate::Point2| [point.x().to_f64_lossy(), point.y().to_f64_lossy()];
                                eprintln!("inverse failed {:?} query={:?} start={:?} end={:?} center={:?} clockwise={} endpoint-matches=({}, {})", reason, approximate(point), approximate(arc.start()), approximate(arc.end()), approximate(arc.center()), arc.is_clockwise(), point == arc.start(), point == arc.end());
                                eprintln!("sweep diagnostics start={:?} end={:?} kind={:?} contains={:?}", crate::classify::classify_oriented_line(arc.center(), arc.start(), point, &self.data.policy), crate::classify::classify_oriented_line(arc.center(), arc.end(), point, &self.data.policy), crate::arc_bezier::classify_sweep_with_policy(&arc, &self.data.policy), arc.contains_sweep_point(point, &self.data.policy));
                            }
                            return Ok(Classification::Decided(None));
                        }""")
p.write_text(s[:a]+body+s[b:])
p=archive/'hypercurve/src/bezier_offset.rs'
s=p.read_text();a=s.index('pub(crate) fn quadratic_conic_parameter_at_incident_point(');b=s.index('\n/// Imports a fixed set of retained point evidences',a);body=s[a:b]
body=body.replace('return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));','eprintln!("conic inverse unsupported at {}",line!()); return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));')
body=body.replace('return Ok(Classification::Uncertain(reason));','eprintln!("conic inverse uncertain {:?} at {}",reason,line!()); return Ok(Classification::Uncertain(reason));')
p.write_text(s[:a]+body+s[b:])
p=archive/'hypercurve/src/curve_region_boolean.rs'
s=p.read_text();needle='        let parameters = incidence;';assert s.count(needle)==1
s=s.replace(needle,'        eprintln!("circle tangent certificates={} scalar-contacts={}", certified_tangent_contacts.len(), incidence.iter().filter(|(p,_)|p.scalar().is_some()).count());\n'+needle)
p.write_text(s)
manifest={name:hashlib.sha256((archive/name).read_bytes()).hexdigest() for name in original}
manifest_name=prefix+'-sources.json'
(A/manifest_name).write_text(json.dumps(manifest,indent=2)+'\n')
build=A/'build-workspace-20260925'
for name in manifest:
    source,destination=archive/name,build/name
    if not destination.exists() or source.read_bytes()!=destination.read_bytes():
        shutil.copy2(source,destination)
        os.utime(destination,None)
    assert (source.stat().st_dev,source.stat().st_ino)!=(destination.stat().st_dev,destination.stat().st_ino)
env = dict(os.environ,HYPERCURVE_DEBUG_RATIONAL_BLOCKER='1',**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
toolchain = Path('/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin')
cargo = str(toolchain/'cargo')
report = dict(parents={repo:subprocess.check_output(['git','rev-parse','HEAD'],cwd=W/repo,text=True).strip() for repo in ['hypercurve','hypersolve','hyperreal']},
              source_manifest=manifest_name,source_directory=str(archive),build_source_directory=str(build),
              workspace_guard=guard_name,checks=[],cases=[],binaries={},all_processes_reaped=False,diagnostic_only=True)

def verify():
    for name,sha in manifest.items():
        assert hashlib.sha256((W/name).read_bytes()).hexdigest() == guard[name], name
        for root in [archive,build]:
            assert hashlib.sha256((root/name).read_bytes()).hexdigest() == sha,name
    for binary in report['binaries'].values():
        assert hashlib.sha256(Path(binary['path']).read_bytes()).hexdigest() == binary['sha256']

def save():
    (A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')

def run(repo,label,command,limit,group):
    report['active'] = label
    save()
    log = A/f'{prefix}-{label}.log'
    start = time.monotonic()
    with log.open('w') as out:
        try:
            code = subprocess.run(command,cwd=build/repo,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=limit).returncode
        except subprocess.TimeoutExpired:
            code = 'timeout'
    row = dict(repo=repo,label=label,command=command,returncode=code,elapsed_seconds=time.monotonic()-start,log=log.name)
    report[group].append(row)
    report.pop('active')
    save()
    print(label,code,round(row['elapsed_seconds'],3),log.read_text()[-2400:] if code else '',flush=True)
    return row

def stop():
    verify()
    report['all_sources_unchanged'] = True
    report['all_processes_reaped'] = True
    save()
    raise SystemExit(1)

def compile(repo):
    command = [cargo,'test','--lib','--release','--all-features','--no-run','--message-format=json','--locked','--offline']
    report['active'] = repo+'-build'
    save()
    with (A/f'{prefix}-{repo}-build.jsonl').open('w') as out, (A/f'{prefix}-{repo}-build.log').open('w') as err:
        code = subprocess.run(command,cwd=build/repo,env=env,stdout=out,stderr=err,timeout=1200).returncode
    report.pop('active')
    report[repo+'_build_returncode'] = code
    if code:
        print((A/f'{prefix}-{repo}-build.log').read_text()[-3000:],flush=True)
        stop()
    rows = [json.loads(line) for line in (A/f'{prefix}-{repo}-build.jsonl').read_text().splitlines()]
    artifact = next(row for row in rows if row.get('reason') == 'compiler-artifact' and row['target']['name'] == 'hypercurve' and row.get('executable'))
    assert not artifact['fresh'],repo
    binary = A/f'{prefix}-{repo}'
    shutil.copy2(artifact['executable'],binary)
    report['binaries'][repo] = dict(path=str(binary),sha256=hashlib.sha256(binary.read_bytes()).hexdigest())
    save()
    return binary

verify()
binary=compile('hypercurve')
jobs = ['curve_region_boolean::certified_successor_tests::selected_parallel_arrangement_splits_interior_cusps']
names = [line[:-6] for line in subprocess.check_output([str(binary),'--list'], text=True).splitlines() if line.endswith(': test')]
assert set(jobs) <= set(names), set(jobs)-set(names)
for index,name in enumerate(jobs):
    row=run('hypercurve',f'case-{index:02}',[str(binary),'--exact',name,'--test-threads=1','--nocapture','--color','never'],180,'cases')
    row['name']=name
    if not row['returncode']:
        assert re.search(r'test result: ok\. 1 passed; 0 failed; 0 ignored;', (A/row['log']).read_text()),name
    save()
verify()
report['all_sources_unchanged']=True
report['all_processes_reaped']=True
report['qualification_complete']=not any(row['returncode'] for row in report['cases']+report['checks'])
save()
print('Every owned process reaped; success=',report['qualification_complete'],flush=True)
sys.exit(0 if report['qualification_complete'] else 1)
