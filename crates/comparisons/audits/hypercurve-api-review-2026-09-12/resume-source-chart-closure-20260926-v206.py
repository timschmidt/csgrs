from pathlib import Path
import hashlib, json, os, re, shutil, subprocess, sys, time

A = Path(__file__).resolve().parent
W = A.parent
version = 'v206'
original_prefix = f'corner-source-chart-full-20260926-{version}'
prefix = original_prefix+'-resumed'
guard_name = f'{original_prefix}-sources.json'
guard = json.loads((A/guard_name).read_text())
manifest = guard
archive = A/'source-archives'/original_prefix
build = A/'build-workspace-20260925'
assert not (A/f'{prefix}-terminal.json').exists()
parents = {name: subprocess.check_output(['git','rev-parse','HEAD'],cwd=W/name,text=True).strip()
           for name in ['hypercurve','hypersolve','hyperreal']}
for repo in parents:
    assert not subprocess.check_output(['git','diff','--cached','--name-only'],cwd=W/repo)
for name,sha in manifest.items():
    source,destination = archive/name,build/name
    assert hashlib.sha256((W/name).read_bytes()).hexdigest()==sha,name
    assert hashlib.sha256(source.read_bytes()).hexdigest()==sha,name
    if not destination.exists() or source.read_bytes()!=destination.read_bytes():
        shutil.copy2(source,destination)
        os.utime(destination,None)
    assert (source.stat().st_dev,source.stat().st_ino)!=(destination.stat().st_dev,destination.stat().st_ino)
env = dict(os.environ, **json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
toolchain = Path('/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin')
cargo = str(toolchain/'cargo')
report=json.loads((A/f'{original_prefix}-terminal.json').read_text())
assert report['all_processes_reaped']
report['previous_qualification']=f'{original_prefix}-terminal.json'
report['all_processes_reaped']=False
report.pop('qualification_complete',None)
for index,row in enumerate(report['cases']):
 target,name=report['selection'][index]
 row.update(target=target,name=name)

def verify():
    for repo, head in parents.items():
        assert subprocess.check_output(['git','rev-parse','HEAD'], cwd=W/repo, text=True).strip() == head
    for name, sha in guard.items():
        assert hashlib.sha256((W/name).read_bytes()).hexdigest() == sha, name
        assert hashlib.sha256((archive/name).read_bytes()).hexdigest() == manifest[name], name
        assert hashlib.sha256((build/name).read_bytes()).hexdigest() == manifest[name], name

def save():
    report['all_sources_unchanged'] = True
    (A/f'{prefix}-terminal.json').write_text(json.dumps(report, indent=2)+'\n')

def run(repo, label, command, limit, group):
    report['active'] = label
    save()
    log = A/f'{prefix}-{label}.log'
    start = time.monotonic()
    with log.open('w') as out:
        try:
            code = subprocess.run(command, cwd=build/repo, env=env, stdout=out, stderr=subprocess.STDOUT, timeout=limit).returncode
        except subprocess.TimeoutExpired:
            code = 'timeout'
    row = dict(repo=repo, label=label, command=command, returncode=code,
               elapsed_seconds=time.monotonic()-start, log=log.name)
    report[group].append(row)
    report.pop('active')
    save()
    print(label, code, log.read_text()[-2200:] if code else '', flush=True)
    return row

def compile_tests(repo, targets):
    os.utime(build/repo/'src/lib.rs', None)
    command = [cargo,'test','--lib']
    for target in targets:
        if target != repo:
            os.utime(build/repo/'tests'/f'{target}.rs', None)
            command += ['--test',target]
    command += ['--release','--all-features','--no-run','--message-format=json','--locked','--offline']
    report['active'] = repo+'-build'
    save()
    with (A/f'{prefix}-{repo}-build.jsonl').open('w') as out, (A/f'{prefix}-{repo}-build.log').open('w') as err:
        code = subprocess.run(command, cwd=build/repo, env=env, stdout=out, stderr=err, timeout=1200).returncode
    report.pop('active')
    report[repo+'_build_returncode'] = code
    if code:
        report['all_processes_reaped'] = True
        save()
        print((A/f'{prefix}-{repo}-build.log').read_text()[-3000:], flush=True)
        raise SystemExit(code)
    rows = [json.loads(line) for line in (A/f'{prefix}-{repo}-build.jsonl').read_text().splitlines()]
    names = {}
    for target in targets:
        artifact = next(row for row in rows if row.get('reason') == 'compiler-artifact' and row['target']['name'] == target and row.get('executable'))
        assert not artifact['fresh'], target
        binary = A/f'{prefix}-{target}'
        shutil.copy2(artifact['executable'], binary)
        report['binaries'][target] = dict(path=str(binary), sha256=hashlib.sha256(binary.read_bytes()).hexdigest())
        names[target] = [line[:-6] for line in subprocess.check_output([str(binary),'--list'], text=True).splitlines() if line.endswith(': test')]
    save()
    return names

verify()
for index,(target,name) in enumerate(report['selection'][len(report['cases']):],start=len(report['cases'])):
 binary=report['binaries'][target]
 assert hashlib.sha256(Path(binary['path']).read_bytes()).hexdigest()==binary['sha256']
 limit=300 if 'one_fragment_nonzero_parallel_loop_extends_chamfer' in name else 180
 row=run('hypercurve',f'case-{index:03}',[binary['path'],'--exact',name,'--nocapture','--test-threads=1','--color','never'],limit,'cases')
 row.update(target=target,name=name)
 if not row['returncode']:
  assert re.search(r'test result: ok\. 1 passed; 0 failed; 0 ignored;',(A/row['log']).read_text()),name
 save()
verify()
for binary in report['binaries'].values():
 assert hashlib.sha256(Path(binary['path']).read_bytes()).hexdigest()==binary['sha256']
report['all_processes_reaped']=True
report['hypercurve_passed']=sum(not row['returncode'] for row in report['cases'])
report['qualification_complete']=all(not row['returncode'] for row in report['cases'])
save()
print('Complete:',report['hypercurve_passed'],'/',len(report['selection']),'passed.',flush=True)
sys.exit(0 if report['qualification_complete'] else 1)
