from pathlib import Path
import hashlib, json, os, re, shutil, subprocess, sys, time

A = Path(__file__).resolve().parent
W = A.parent
prefix = 'subdivision-progress-20260926-v136'
guard_name = 'local-chord-complete-replay-20260924-v135-sources.json'
guard = json.loads((A/guard_name).read_text())
original = guard
archive = A/'source-archives'/prefix
assert not archive.exists()
for name,sha in original.items():
    source,destination = W/name,archive/name
    assert hashlib.sha256(source.read_bytes()).hexdigest() == sha
    destination.parent.mkdir(parents=True,exist_ok=True)
    shutil.copy2(source,destination)
p = archive/'hypersolve/src/ordered_field_roots.rs'
s = p.read_text()
a = s.index('pub fn isolate_ordered_field_polynomial_roots<C, F>(')
b = s.index('\n#[cfg(test)]',a)
part = s[a:b]
needle = '    if lower.partial_cmp(upper) != Some(Ordering::Less) {'
probe = r'''    static CALL_ID: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
    let diagnostic_id = CALL_ID.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    let diagnostic = polynomial.len() >= 8;
    let diagnostic_start = std::time::Instant::now();
    let mut diagnostic_depth = None;
    if diagnostic {
        eprintln!("isolation begin id={} degree={}", diagnostic_id, polynomial.len().saturating_sub(1));
    }
'''
assert part.count(needle)==1
part=part.replace(needle,probe+needle)
needle='            let variations = bernstein_sign_variations(&node.controls, field)?;'
probe=r'''            if diagnostic && (node.depth == 0 || node.depth.is_power_of_two())
                && diagnostic_depth.is_none_or(|depth| node.depth > depth) {
                diagnostic_depth = Some(node.depth);
                eprintln!("isolation node id={} depth={} pending={} subdivisions={} elapsed-ms={}", diagnostic_id, node.depth, stack.len(), subdivision_steps, diagnostic_start.elapsed().as_millis());
            }
'''
assert part.count(needle)==1
part=part.replace(needle,probe+needle)
s=s[:a]+part+s[b:];p.write_text(s)
p=archive/'hypercurve/src/bezier_offset.rs'
s=p.read_text();a=s.index('fn recursive_quadratic_polynomial_local_parameters(');b=s.index('\n/// Isolates every root',a)
part=s[a:b]
needle='    match report.status {'
assert part.count(needle)==1
part=part.replace(needle,'    eprintln!("local isolation complete degree={} status={:?} subdivisions={} roots={}", coefficients.len().saturating_sub(1), report.status, report.subdivision_steps, report.intervals.len());\n'+needle)
s=s[:a]+part+s[b:];p.write_text(s)
p=archive/'hypercurve/src/bezier_region.rs'
s=p.read_text();a=s.index('    fn one_fragment_nonzero_parallel_loop_extends_chamfer_cuts_on_one_finite_envelope()');b=s.index('\n    #[test]',a)
part=s[a:b].replace('                let seam = p(0, 0);','                eprintln!("case begin policy={policy:?} reversed={reversed}");\n                let seam = p(0, 0);').replace('                let trimmed = region','                eprintln!("trim begin");\n                let trimmed = region').replace('                let extended_work = || {','                eprintln!("trim complete candidates={}; extension begin", trimmed.candidate_count());\n                let extended_work = || {').replace('                let extended = extended.unwrap();','                eprintln!("extension returned ok={}", extended.is_ok());\n                let extended = extended.unwrap();')
s=s[:a]+part+s[b:];p.write_text(s)
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
