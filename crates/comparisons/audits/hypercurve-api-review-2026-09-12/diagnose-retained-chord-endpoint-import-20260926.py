from pathlib import Path
import hashlib, json, os, re, shutil, subprocess, sys, time

A = Path(__file__).resolve().parent
W = A.parent
prefix = 'retained-chord-endpoint-import-20260926-v131'
guard_name = 'local-chord-complete-replay-20260924-v130-sources.json'
guard = json.loads((A/guard_name).read_text())
original = guard
archive = A/'source-archives'/prefix
assert not archive.exists()
for name,sha in original.items():
    source,destination = W/name,archive/name
    assert hashlib.sha256(source.read_bytes()).hexdigest() == sha
    destination.parent.mkdir(parents=True,exist_ok=True)
    shutil.copy2(source,destination)
p = archive/'hypercurve/src/bezier_offset.rs'
s = p.read_text()
start = s.index('    fn recursive_projective_parallel_intersections_in_domain(')
end = s.index('    fn recursive_projective_parallel_intersections_from_projection(',start)
part = s[start:end]
needle = '        if let Some(candidates) =\n'
assert part.count(needle) == 1
probe = r'''        eprintln!("local chord/parallel sources={} depth={} rational-degree={} radical-degree={}", system.base.sources.len(), system.field.base_and_extension_path().1.len(), system.incidence.rational.len().saturating_sub(1), system.incidence.radical.len().saturating_sub(1));
        for point in [self.start(), self.end()] {
            if let CurvePoint2(CurvePointData2::AnalyticParallel(point)) = point {
                let parameter = point.data.parameter.curve_parameter();
                let same_sheet = point.data.parallel == *parallel
                    && point.data.frame_tangent.as_ref() == frame_tangent
                    && policy.accepts_retained_policy(point.data.policy)
                    && [&point.data.tangent_distance, &point.data.translation_x, &point.data.translation_y]
                        .into_iter().all(|v| v.zero_status() == ZeroKnowledge::Zero);
                let imported = system.field.retained_parameter_value(&parameter, policy)?.is_some();
                eprintln!("endpoint same-sheet={} native={} selected={} recursive={} scalar={} imported={}", same_sheet, parameter.as_bezier_parameter().is_some(), parameter.as_selected_fiber().is_some(), parameter.as_recursive_projective().is_some(), parameter.scalar().is_some(), imported);
            } else {
                eprintln!("endpoint non-analytic");
            }
        }
'''
part = part.replace(needle,probe+needle)
s = s[:start]+part+s[end:]
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
binary=compile('hypercurve')
name='bezier_region::tests::one_fragment_nonzero_parallel_loop_extends_chamfer_cuts_on_one_finite_envelope'
row=run('hypercurve','case-00',[str(binary),'--exact',name,'--test-threads=1','--nocapture','--color','never'],25,'cases')
row['name']=name
verify()
report['all_sources_unchanged']=True
report['all_processes_reaped']=True
save()
print('Diagnostic complete; every owned process reaped.',flush=True)
