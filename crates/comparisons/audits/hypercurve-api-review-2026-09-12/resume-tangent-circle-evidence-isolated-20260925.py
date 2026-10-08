from pathlib import Path
import hashlib, json, os, re, shutil, subprocess, time

A = Path(__file__).resolve().parent
W = A.parent
prefix = 'tangent-circle-evidence-isolated-20260925-v114'
report_path = A / f'{prefix}-terminal.json'
report = json.loads(report_path.read_text())
guard = json.loads((A / report['workspace_guard']).read_text())
manifest = json.loads((A / report['source_manifest']).read_text())
assert report['all_processes_reaped'] and len(report['cases']) == 1
assert all(row['returncode'] == 0 for row in report['checks'] + report['cases'])
assert len(report['selection']) == 265
build = Path(report['build_source_directory'])
env = dict(os.environ, **json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
cargo = '/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'

def verify():
    for repo, head in report['parents'].items():
        assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=W/repo,text=True).strip() == head
    for name, sha in guard.items():
        assert hashlib.sha256((W/name).read_bytes()).hexdigest() == sha, name
        for root in [report['source_directory'], report['build_source_directory']]:
            assert hashlib.sha256((Path(root)/name).read_bytes()).hexdigest() == manifest[name], name
    for binary in report['binaries'].values():
        assert hashlib.sha256(Path(binary['path']).read_bytes()).hexdigest() == binary['sha256']

def save():
    report_path.write_text(json.dumps(report,indent=2)+'\n')

verify()
target = 'hypercurve_analytic_parallel_region'
assert target not in report['binaries']
os.utime(build/'hypercurve/tests'/f'{target}.rs',None)
command = [cargo,'test','--test',target,'--release','--all-features','--no-run','--message-format=json','--locked','--offline']
report['all_processes_reaped'] = False
report['active'] = 'resume-analytic-build'
save()
with (A/f'{prefix}-resume-build.jsonl').open('w') as out, (A/f'{prefix}-resume-build.log').open('w') as err:
    code = subprocess.run(command,cwd=build/'hypercurve',env=env,stdout=out,stderr=err,timeout=1200).returncode
report.pop('active')
report['resume_build_returncode'] = code
assert code == 0, (A/f'{prefix}-resume-build.log').read_text()[-3000:]
rows = [json.loads(line) for line in (A/f'{prefix}-resume-build.jsonl').read_text().splitlines()]
artifact = next(row for row in rows if row.get('reason') == 'compiler-artifact' and row['target']['name'] == target and row.get('executable'))
assert not artifact['fresh']
binary = A/f'{prefix}-{target}'
shutil.copy2(artifact['executable'],binary)
report['binaries'][target] = dict(path=str(binary),sha256=hashlib.sha256(binary.read_bytes()).hexdigest())
names = {target:[line[:-6] for line in subprocess.check_output([binary['path'],'--list'],text=True).splitlines() if line.endswith(': test')]
         for target,binary in report['binaries'].items()}
verify()
save()
done = {(row['target'],row['name']) for row in report['cases']}
for index,(target,name) in enumerate(report['selection']):
    if (target,name) in done:
        continue
    assert name in names[target], name
    binary = report['binaries'][target]
    assert hashlib.sha256(Path(binary['path']).read_bytes()).hexdigest() == binary['sha256']
    label = f'case-{index:03}'
    log = A/f'{prefix}-{label}.log'
    command = [binary['path'],'--exact',name,'--nocapture','--test-threads=1','--color','never']
    report['active'] = label
    save()
    start = time.monotonic()
    with log.open('w') as out:
        try:
            code = subprocess.run(command,cwd=build/'hypercurve',env=env,stdout=out,stderr=subprocess.STDOUT,timeout=75).returncode
        except subprocess.TimeoutExpired:
            code = 'timeout'
    row = dict(repo='hypercurve',label=label,command=command,returncode=code,
               elapsed_seconds=time.monotonic()-start,log=log.name,target=target,name=name)
    report['cases'].append(row)
    report.pop('active')
    save()
    print(label,code,name,log.read_text()[-2200:] if code else '',flush=True)
    if code:
        verify()
        report['all_processes_reaped'] = True
        save()
        raise SystemExit(1)
    assert re.search(r'test result: ok\. 1 passed; 0 failed; 0 ignored;',log.read_text()),name
verify()
report['hypercurve_passed'] = len(report['selection'])
report['all_sources_unchanged'] = True
report['all_processes_reaped'] = True
report['qualification_complete'] = True
save()
print('Complete:',len(report['selection']),'Hypercurve cases passed.',flush=True)
