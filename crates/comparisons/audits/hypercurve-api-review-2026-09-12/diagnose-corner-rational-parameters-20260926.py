from pathlib import Path
import hashlib, json, os, re, shutil, subprocess, sys, time

A = Path(__file__).resolve().parent
W = A.parent
prefix = 'corner-support-audit-20260926-v146'
guard_name = 'local-chord-complete-replay-20260924-v143-sources.json'
guard = json.loads((A/guard_name).read_text())
original = guard
archive = A/'source-archives'/prefix
assert not archive.exists()
for name,sha in original.items():
    source,destination = W/name,archive/name
    assert hashlib.sha256(source.read_bytes()).hexdigest() == sha
    destination.parent.mkdir(parents=True,exist_ok=True)
    shutil.copy2(source,destination)
p = archive/'hypercurve/src/error.rs'
s = p.read_text().replace('pub(crate) const fn blocked(', '#[track_caller]\n    pub(crate) fn blocked(')
needle='        Self::Blocked(ExactCurveBlocker::new(operation, family, reason))'
probe="""        if matches!(family, CurveFamily2::Line) && matches!(reason, UncertaintyReason::Unsupported) {
            static COUNT: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
            if COUNT.fetch_add(1, std::sync::atomic::Ordering::Relaxed) < 12 {
                eprintln!(\"line unsupported operation={:?}\\n{}\", operation, std::panic::Location::caller());
            }
        }
"""
assert s.count(needle)==1
s=s.replace(needle,probe+needle)
p.write_text(s)
p=archive/'hypercurve/src/curve_region_boolean.rs'
s=p.read_text().replace('    fn blocked(&self, carrier_index:', '    #[track_caller]\n    fn blocked(&self, carrier_index:')
needle='            if let Some(blocker) = result.blockers.first() {'
assert s.count(needle)==1
s=s.replace(needle,needle+'\n                eprintln!("pair blocker indices=({}, {}) families=({:?}, {:?}) context={:?} kind={:?} adjacent={}", pair.first_carrier_index, pair.second_carrier_index, self.data.carriers[pair.first_carrier_index].family, self.data.carriers[pair.second_carrier_index].family, std::mem::discriminant(&pair.context), std::mem::discriminant(blocker), self.authored_carriers_are_adjacent(pair));\n')
p.write_text(s)
p=archive/'hypercurve/src/bezier_offset.rs'
s=p.read_text()
a=s.index('    fn recursive_projective_rational_system(')
b=s.index('    pub(crate) fn rational_intersections(',a)
part=s[a:b].replace('std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some()', 'true')
s=s[:a]+part+s[b:]
p.write_text(s)
p=archive/'hypercurve/src/bezier_offset.rs'
s=p.read_text()
a=s.index('impl BezierRecursiveProjectiveChordRationalSystem2 {')
b=s.index('    /// Compares a retained unit-domain contact', a)
part=s[a:b]
needle='        let crossing = if known_roots.is_empty() {'
part=part.replace(needle, '        eprintln!("rational roots owned={} degree={} unit={}", known_roots.len(), coefficients.len().saturating_sub(1), range == &CurveParameterRange2::unit());\n'+needle)
s=s[:a]+part+s[b:]
a=s.index('fn recursive_quadratic_polynomial_local_parameters(')
b=s.index('fn recursive_projective_polynomial_parameters(',a)
part=s[a:b].replace('std::env::var_os("HYPERCURVE_DEBUG_RATIONAL_BLOCKER").is_some()', 'true')
part=part.replace('return Ok(Classification::Uncertain(reason))', '{ eprintln!("polynomial uncertainty line={} reason={reason:?}", line!()); return Ok(Classification::Uncertain(reason)); }')
part=part.replace('return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));', 'eprintln!("polynomial projection unavailable line={}", line!()); return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));')
s=s[:a]+part+s[b:]
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
jobs = ['curve_support::tests::parallel_injectivity_requires_regularity_on_the_requested_range',
        'bezier_region::tests::one_fragment_materialized_loop_extends_algebraic_chamfer_cuts_once']
for index,name in enumerate(jobs):
    row=run('hypercurve',f'case-{index:02}',[str(binary),'--exact',name,'--test-threads=1','--nocapture','--color','never'],30,'cases')
    row['name']=name
    save()
verify()
report['all_sources_unchanged']=True
report['all_processes_reaped']=True
save()
print('Diagnostic complete; every owned process reaped.',flush=True)
