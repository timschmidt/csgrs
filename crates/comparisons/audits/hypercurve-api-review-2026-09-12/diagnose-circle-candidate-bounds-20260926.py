from pathlib import Path
import hashlib, json, os, re, shutil, subprocess, sys, time

A = Path(__file__).resolve().parent
W = A.parent
prefix = 'retained-circle-candidate-bounds-20260926-v194'
guard_name = 'local-chord-complete-replay-20260924-v192-sources.json'
guard = json.loads((A/guard_name).read_text())
original = guard
archive = A/'source-archives'/prefix
assert not archive.exists()
for name,sha in original.items():
    source,destination = W/name,archive/name
    assert hashlib.sha256(source.read_bytes()).hexdigest() == sha
    destination.parent.mkdir(parents=True,exist_ok=True)
    shutil.copy2(source,destination)
p=archive/'hypercurve/src/bezier_offset.rs'; source=p.read_text()
a=source.index('    fn recursive_circle_parallel_intersections('); b=source.index('    fn represented_parallel_intersections(',a); f=source[a:b]
f=f.replace('            if needs_incidence_replay {', '            if needs_incidence_replay {\n                fn bounds(label: &str, parameter: &BezierParameter2) {\n                    let degree = match parameter { BezierParameter2::Exact(_) => 0, BezierParameter2::Algebraic(p) => p.polynomial().degree() };\n                    let mut refinement = BezierParameterRefinement2::new(parameter, &CurveContext::STRICT);\n                    for steps in [0, 8, 64] {\n                        let parameter = refinement.refine_to(steps);\n                        if let Ok(Classification::Decided(interval)) = parameter.known_interval(&CurveContext::STRICT) {\n                            let ends = [interval.start(), interval.end()].map(|v| v.certified_dyadic_interval(-32).map(|x| [x[0].to_string(), x[1].to_string()]));\n                            eprintln!("candidate-bounds role={label} degree={degree} steps={steps} ends={ends:?}");\n                        }\n                    }\n                }\n                eprintln!("candidate-replay axes={} bounded={} normal={} interval_root={interval_incidence:?} interval_location={interval_location:?} cross={interval_cross:?}", system.base.sources.len(), policy.has_bounded_exact_predicate_budget(), normal_contact.is_some());\n                bounds("candidate", &candidate);\n                if let Some(frame) = self.data.frame.chord_normal() {\n                    if let CurvePoint2(CurvePointData2::AnalyticParallel(point)) = &frame.center {\n                        if let Some((source, parameter)) = point.native_parallel_evaluation() {\n                            if let Some(parameter) = parameter.as_bezier_parameter() { bounds("center",parameter); }\n                            for (role,v) in [("source-distance",source.distance()),("target-distance",other.distance()),("radius",self.radial_distance())] {\n                                let bounds = v.certified_dyadic_interval(-32).map(|x| [x[0].to_string(), x[1].to_string()]);\n                                eprintln!("candidate-scalar role={role} bounds={bounds:?}");\n                            }\n                        }\n                    }\n                }\n                let box_ = bezier_parameter_root_representation(&candidate);\n                let interval = RealInterval { lower:box_.interval.lower, upper:box_.interval.upper };\n                for steps in [8,64,128] {\n                    let coefficients = |items: &[BezierRecursiveQuadraticValue2]| items.iter().filter(|v| v.interval_with_coefficient_precision(steps,Some(-128)).is_none()).count();\n                    eprintln!("candidate-enclosure steps={steps} missing=({},{},{}) expression={}", coefficients(&system.circle.rational),coefficients(&system.circle.radical),coefficients(&system.circle.speed_squared),recursive_quadratic_parallel_expression_interval(&system.circle, system.unit_target_speed,&interval,steps,-128).is_some());\n                }\n')
source=source[:a]+f+source[b:];p.write_text(source)
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
env = dict(os.environ,**json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
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
    print(label,code,round(row['elapsed_seconds'],3),log.read_text()[-6000:] if code else '',flush=True)
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
os.utime(build/'hypercurve/src/lib.rs',None)
binary=compile('hypercurve')
jobs = ['curve_region_boolean::certified_successor_tests::selected_parallel_arrangement_splits_interior_cusps']
names = [line[:-6] for line in subprocess.check_output([str(binary),'--list'], text=True).splitlines() if line.endswith(': test')]
assert set(jobs) <= set(names), set(jobs)-set(names)

for index,name in enumerate(jobs):
    row=run('hypercurve',f'case-{index:02}',[str(binary),'--exact',name,'--test-threads=1','--nocapture','--color','never'],30,'cases')
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
