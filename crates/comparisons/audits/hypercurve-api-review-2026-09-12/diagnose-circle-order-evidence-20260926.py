from pathlib import Path
import hashlib, json, os, re, shutil, subprocess, sys, time

A = Path(__file__).resolve().parent
W = A.parent
prefix = 'circle-order-evidence-20260926-v201'
guard_name = 'circle-contact-evidence-20260926-v200-workspace.json'
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
a=source.index('fn recursive_quadratic_parameter_interval_sign('); b=source.index('fn recursive_quadratic_parallel_expression_interval(',a); f=source[a:b]
f=f.replace('    let strict = &CurveContext::STRICT;', '    eprintln!("order-interval-begin");\n    let strict = &CurveContext::STRICT;')
f=f.replace('        let target =\n            bezier_parameter_root_representation(target_refinement.refine_to(target_steps));', '        eprintln!("order-refine-begin steps={target_steps}");\n        let target = bezier_parameter_root_representation(target_refinement.refine_to(target_steps));\n        eprintln!("order-refine-end steps={target_steps} degree={} rational={} witness={} bounds_rational={}", target.polynomial_coefficients.len()-1, target.polynomial_coefficients.iter().filter(|c| c.exact_rational_ref().is_some()).count(), target.exact_point_witness().is_some(), target.interval.lower.exact_rational_ref().is_some() && target.interval.upper.exact_rational_ref().is_some());')
assert 'order-refine-begin' in f; source=source[:a]+f+source[b:]
a=source.index('    /// Orders one analytic-parallel contact against a represented semicircle parameter.'); b=source.index('    /// Refines one analytic-parallel contact',a); f=source[a:b]
f=f.replace('        if let BezierAlgebraicCuspSemicircleParallelCorrelation2::Retained(retained) =', '        eprintln!("contact-order location={:?} probe={}", contact.location, parameter.exact_rational_ref().map(|r| r.to_string()).unwrap_or_else(|| "nonrational".into()));\n        if let BezierAlgebraicCuspSemicircleParallelCorrelation2::Retained(retained) =')
source=source[:a]+f+source[b:]
a=source.index('    pub(crate) fn certified_incident_point_evidence_location('); b=source.index('    fn incident_location_from_orders(',a); f=source[a:b]
f=f.replace('            let scalar_order = policy.bounded_exact_predicate_pass(|| {','            eprintln!("point-order begin start={source_start} parameter={:?} endpoint={:?}", std::mem::discriminant(parameter), std::mem::discriminant(endpoint_parameter));\n            let scalar_order = policy.bounded_exact_predicate_pass(|| {')
f=f.replace('            if matches!(scalar_order, Classification::Decided(_)) {','            eprintln!("point-order scalar={scalar_order:?}");\n            if matches!(scalar_order, Classification::Decided(_)) {')
f=f.replace('            if parameter.shares_parametric_source_point(endpoint_parameter, policy)? {','            eprintln!("point-order source-identity-begin");\n            if parameter.shares_parametric_source_point(endpoint_parameter, policy)? {\n                eprintln!("point-order source-identity-proved");')
f=f.replace('            let endpoint = match self.endpoint_point_evidence(', '            eprintln!("point-order projective-begin");\n            let endpoint = match self.endpoint_point_evidence(')
f=f.replace('            parameter.cmp_by_refinement(endpoint_parameter, policy)\n        };','            eprintln!("point-order full-scalar-begin");\n            parameter.cmp_by_refinement(endpoint_parameter, policy)\n        };')
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
