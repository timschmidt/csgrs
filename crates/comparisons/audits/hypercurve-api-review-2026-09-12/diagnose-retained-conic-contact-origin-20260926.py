from pathlib import Path
import hashlib, json, os, re, shutil, subprocess, sys, time

A = Path(__file__).resolve().parent
W = A.parent
version = 'v129'
prefix = 'retained-conic-contacts-origin-20260926-v129'
prior = json.loads((A/'retained-conic-contacts-isolated-20260926-v127-terminal.json').read_text())
assert prior['all_processes_reaped']
guard = json.loads((A/prior['workspace_guard']).read_text())
original = json.loads((A/prior['source_manifest']).read_text())
archive = A/'source-archives'/prefix
assert not archive.exists()
for name,sha in original.items():
    source,destination = Path(prior['source_directory'])/name,archive/name
    assert hashlib.sha256(source.read_bytes()).hexdigest() == sha
    destination.parent.mkdir(parents=True,exist_ok=True)
    shutil.copy2(source,destination)
p = archive/'hypercurve/src/error.rs'
s = p.read_text().replace('    pub(crate) const fn blocked(', '    #[track_caller]\n    pub(crate) fn blocked(')
s = s.replace('        Self::Blocked(ExactCurveBlocker::new(operation, family, reason))', '        let caller = std::panic::Location::caller();\n        eprintln!("blocker origin {}:{} operation={operation:?} family={family:?} reason={reason:?}", caller.file(), caller.line());\n        Self::Blocked(ExactCurveBlocker::new(operation, family, reason))')
p.write_text(s)
p = archive/'hypercurve/tests/hypercurve_analytic_parallel_region.rs'
s=p.read_text(); start=s.index('fn retained_rational_arc_and_analytic_parallel_fillet_extends_exactly()'); end=s.index('\n#[test]',start)
part=s[start:end].replace('                let (source, vertex_index) =', '                eprintln!("case policy={policy:?} unit_end_weights={unit_end_weights} reversed={reversed}");\n                let (source, vertex_index) =')
s=s[:start]+part+s[end:]; p.write_text(s)
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
              checks=[],cases=[],binaries={},all_processes_reaped=False,diagnostic_only=True)

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
    command = [cargo,'test','--test','hypercurve_analytic_parallel_region','--release','--all-features','--no-run','--message-format=json','--locked','--offline']
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
    artifact = next(row for row in rows if row.get('reason') == 'compiler-artifact' and row['target']['name'] == 'hypercurve_analytic_parallel_region' and row.get('executable'))
    assert not artifact['fresh'],repo
    binary = A/f'{prefix}-{repo}'
    shutil.copy2(artifact['executable'],binary)
    report['binaries'][repo] = dict(path=str(binary),sha256=hashlib.sha256(binary.read_bytes()).hexdigest())
    save()
    return binary

verify()
binary=compile('hypercurve')
name='retained_rational_arc_and_analytic_parallel_fillet_extends_exactly'
row=run('hypercurve','case-00',[str(binary),'--exact',name,'--test-threads=1','--nocapture','--color','never'],60,'cases')
row['name']=name
verify()
report['all_sources_unchanged']=True
report['all_processes_reaped']=True
save()
print('Diagnostic complete; every owned process reaped.',flush=True)
