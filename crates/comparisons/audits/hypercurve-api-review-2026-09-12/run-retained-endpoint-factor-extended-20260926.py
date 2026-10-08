from pathlib import Path
import hashlib, json, os, signal, subprocess, time

A = Path(__file__).resolve().parent
W = A.parent
prefix = 'retained-endpoint-factor-20260926-v133-extended-run'
assert not (A/f'{prefix}-terminal.json').exists()
qualification = json.loads((A/'retained-endpoint-factor-20260926-v133-terminal.json').read_text())
assert qualification['all_processes_reaped'] and qualification['all_sources_unchanged']
manifest = json.loads((A/qualification['source_manifest']).read_text())
binding = qualification['binaries']['hypercurve']
binary = Path(binding['path'])
roots = [W,Path(qualification['source_directory']),Path(qualification['build_source_directory'])]
jobs = [row['name'] for row in qualification['cases'] if row['returncode'] == 'timeout' and 'one_fragment_nonzero' in row['name']]
assert len(jobs) == 1

def verify():
    assert hashlib.sha256(binary.read_bytes()).hexdigest() == binding['sha256']
    for root in roots:
        for name,sha in manifest.items():
            assert hashlib.sha256((root/name).read_bytes()).hexdigest() == sha, name

verify()
prefix = 'retained-endpoint-factor-20260926-v133-extended-run'
assert not (A/f'{prefix}-terminal.json').exists()
name = jobs[0]
command = [str(binary),'--exact',name,'--test-threads=1','--nocapture','--color','never']
report = dict(binary=binding,source_manifest=qualification['source_manifest'],qualification='retained-endpoint-factor-20260926-v133-terminal.json',source_directory=qualification['source_directory'],build_source_directory=qualification['build_source_directory'],command=command,name=name,all_processes_reaped=False,limit_seconds=300)
log=A/f'{prefix}.log'
report['log']=log.name
(A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
start=time.monotonic()
with log.open('w') as out:
    try:
        code=subprocess.run(command,cwd=roots[1]/'hypercurve',stdout=out,stderr=subprocess.STDOUT,timeout=300).returncode
    except subprocess.TimeoutExpired:
        code='timeout'
report['returncode']=code
report['elapsed_seconds']=time.monotonic()-start
verify()
report['all_sources_unchanged']=True
report['all_processes_reaped']=True
report['passed']=code==0 and 'test result: ok. 1 passed; 0 failed; 0 ignored;' in log.read_text()
(A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:report[k] for k in ['returncode','elapsed_seconds','passed','all_processes_reaped']}),flush=True)
raise SystemExit(0 if report['passed'] else 1)
