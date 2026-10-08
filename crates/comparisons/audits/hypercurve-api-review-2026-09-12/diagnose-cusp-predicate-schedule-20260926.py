from pathlib import Path
import hashlib, json, os, re, shutil, subprocess, sys, time

A = Path(__file__).resolve().parent
W = A.parent
prefix = 'cusp-predicate-schedule-20260926-v160'
guard_name = 'local-chord-complete-replay-20260924-v159-sources.json'
guard = json.loads((A/guard_name).read_text())
original = guard
archive = A/'source-archives'/prefix
assert not archive.exists()
for name,sha in original.items():
    source,destination = W/name,archive/name
    assert hashlib.sha256(source.read_bytes()).hexdigest() == sha
    destination.parent.mkdir(parents=True,exist_ok=True)
    shutil.copy2(source,destination)
p=archive/'hypercurve/src/lib.rs'
p.write_text(p.read_text()+'\n#[cfg(test)] pub(crate) static CUSP_PROBE_ACTIVE: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);\n')
p=archive/'hypercurve/src/curve_region_boolean.rs'
s=p.read_text();needle='                let expanded = normalized\n';assert s.count(needle)==1
s=s.replace(needle,'                crate::CUSP_PROBE_ACTIVE.store(true, std::sync::atomic::Ordering::Relaxed);\n'+needle)
p.write_text(s)
p=archive/'hypercurve/src/bezier_parameter.rs'
s=p.read_text();needle='    if let Some(sign) = hypersolve::sign_at_selected_root('
assert s.count(needle)==1
s=s.replace(needle,'''    #[cfg(test)]
    if crate::CUSP_PROBE_ACTIVE.load(std::sync::atomic::Ordering::Relaxed)
        && coefficients.iter().any(|value| value.exact_rational_ref().is_none())
    {
        static COUNT: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
        if COUNT.fetch_add(1, std::sync::atomic::Ordering::Relaxed) < 80 {
            eprintln!("general-real root sign bounded={} defining={} predicate={}",
                policy.has_bounded_exact_predicate_budget(), algebraic.polynomial().degree(), filter.degree());
        }
    }
'''+needle)
p.write_text(s)
p=archive/'hypercurve/src/bezier_offset.rs'
s=p.read_text();a=s.index('    pub(crate) fn certified_incident_point_evidence_location(');b=s.index('    fn incident_location_from_orders(',a);body=s[a:b]
body=body.replace('        if let Classification::Decided(side) = endpoint_side','''        #[cfg(test)] if crate::CUSP_PROBE_ACTIVE.load(std::sync::atomic::Ordering::Relaxed) {
            eprintln!("incident location chord-side={endpoint_side:?}");
        }
        if let Classification::Decided(side) = endpoint_side''',1)
body=body.replace('            let scalar_order = policy.bounded_exact_predicate_pass(|| {','''            #[cfg(test)] if crate::CUSP_PROBE_ACTIVE.load(std::sync::atomic::Ordering::Relaxed) {
                eprintln!("incident scalar comparison source-start={source_start}");
            }
            let scalar_order = policy.bounded_exact_predicate_pass(|| {''',1)
body=body.replace('            if matches!(scalar_order, Classification::Decided(_)) {','''            #[cfg(test)] if crate::CUSP_PROBE_ACTIVE.load(std::sync::atomic::Ordering::Relaxed) {
                eprintln!("incident scalar result={scalar_order:?}");
            }
            if matches!(scalar_order, Classification::Decided(_)) {''',1)
body=body.replace('            parameter.cmp_by_refinement(endpoint_parameter, policy)\n','''            #[cfg(test)] if crate::CUSP_PROBE_ACTIVE.load(std::sync::atomic::Ordering::Relaxed) {
                eprintln!("incident complete scalar comparison source-start={source_start}");
            }
            parameter.cmp_by_refinement(endpoint_parameter, policy)
''')
p.write_text(s[:a]+body+s[b:])
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
jobs = ['curve_region_boolean::certified_successor_tests::selected_parallel_arrangement_splits_interior_cusps']
for index,name in enumerate(jobs):
    row=run('hypercurve',f'case-{index:02}',[str(binary),'--exact',name,'--test-threads=1','--nocapture','--color','never'],30,'cases')
    row['name']=name
    save()
verify()
report['all_sources_unchanged']=True
report['all_processes_reaped']=True
save()
print('Diagnostic complete; every owned process reaped.',flush=True)
