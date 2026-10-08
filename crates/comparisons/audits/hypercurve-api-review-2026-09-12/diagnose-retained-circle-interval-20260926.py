from pathlib import Path
import hashlib, json, os, re, shutil, subprocess, sys, time

A = Path(__file__).resolve().parent
W = A.parent
prefix = 'retained-circle-isolated-diagnostic-20260926-v178'
guard_name = 'local-chord-complete-replay-20260924-v175-sources.json'
guard = json.loads((A/guard_name).read_text())
original = guard
archive = A/'source-archives'/prefix
assert not archive.exists()
for name,sha in original.items():
    source,destination = W/name,archive/name
    assert hashlib.sha256(source.read_bytes()).hexdigest() == sha
    destination.parent.mkdir(parents=True,exist_ok=True)
    shutil.copy2(A/'source-archives/retained-circle-parallel-isolated-20260926-v176'/name,destination)
p=archive/'hypercurve/src/bezier_offset.rs'
t=p.read_text();a=t.index('    fn recursive_circle_parallel_intersections(');b=t.index('    fn represented_parallel_intersections(',a);chunk=t[a:b]
chunk=chunk.replace('        let mut contacts = Vec::with_capacity(candidates.len());','        eprintln!("recursive-circle candidate-count={}", candidates.len());\n        let mut contacts = Vec::with_capacity(candidates.len());')
chunk=chunk.replace('        for candidate in candidates {','        for candidate in candidates {\n            eprintln!("recursive-circle is-t2={:?}", candidate.same_value(&BezierParameter2::Exact(Real::from(2_i8)), policy));')
chunk=chunk.replace('            if !incidence_certified && transverse == Some(false) {','            eprintln!("recursive-circle certified={} transverse={transverse:?}", incidence_certified);\n            if !incidence_certified && transverse == Some(false) {')
chunk=chunk.replace('            if interval_location == Some(None) {','            eprintln!("recursive-circle location={interval_location:?}");\n            if interval_location == Some(None) {')
chunk=chunk.replace('            let needs_incidence_replay =','            eprintln!("recursive-circle cross={interval_cross:?}");\n            let needs_incidence_replay =')
old='                match system.expression_sign_with_evaluation(\n                    &system.circle,\n                    evaluation\n                        .as_ref()\n                        .expect("an uncertified recursive incidence retains its evaluation"),\n                    policy,\n                )? {'
new=old.replace('match system.', 'let replay = system.').replace(')? {', ')?;\n                eprintln!("recursive-circle incidence={replay:?}");\n                match replay {')
assert old in chunk;chunk=chunk.replace(old,new);t=t[:a]+chunk+t[b:]
a=t.index('fn recursive_quadratic_parallel_expression_interval(');b=t.index('fn recursive_quadratic_parallel_expression_transverse_root(',a);chunk=t[a:b]
old='    Some(rational.add(&radical.multiply(&speed)?))'
new='    let value = rational.add(&radical.multiply(&speed)?);\n    if target.lower == Real::from(2_i8) && target.upper == Real::from(2_i8) {\n        for (label, value) in [("a", &rational), ("b", &radical), ("speed", &speed), ("sum", &value)] {\n            eprintln!("root2 interval {label} approx=({:?},{:?}) rational=({:?},{:?})", value.lower.to_f64_lossy(), value.upper.to_f64_lossy(), value.lower.exact_rational_ref().map(|v| v.to_string()), value.upper.exact_rational_ref().map(|v| v.to_string()));\n        }\n    }\n    Some(value)'
assert old in chunk;chunk=chunk.replace(old,new);t=t[:a]+chunk+t[b:];p.write_text(t)
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
jobs = ['bezier_offset::conversion_tests::dense_chord_normal_circle_extends_over_an_independent_analytic_speed']
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
