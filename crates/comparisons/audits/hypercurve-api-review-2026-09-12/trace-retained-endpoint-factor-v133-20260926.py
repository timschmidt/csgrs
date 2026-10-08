from pathlib import Path
import hashlib, json, os, signal, subprocess, time

A = Path(__file__).resolve().parent
W = A.parent
prefix = 'retained-endpoint-factor-20260926-v133-stack'
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
report = dict(binary=binding,source_manifest=qualification['source_manifest'],diagnostic_only=True,cases=[],all_processes_reaped=False)
for index,name in enumerate(jobs):
    command = ['/usr/bin/gdb','-nx','-nh','--batch']
    for option in ['set pagination off','set confirm off','set debuginfod enabled off',
                   'set print frame-arguments none','set print entry-values no','run',
                   'thread apply all bt 12','thread apply all bt -45','kill','quit']:
        command += ['-ex',option]
    command += ['--args',str(binary),'--exact',name,'--test-threads=1','--nocapture']
    log = A/f'{prefix}-{index}.log'
    start = time.monotonic()
    interrupted = False
    with log.open('w') as out:
        process = subprocess.Popen(command,cwd=roots[1]/'hypercurve',stdout=out,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            code = process.wait(timeout=20)
        except subprocess.TimeoutExpired:
            interrupted = True
            os.killpg(process.pid,signal.SIGINT)
            try:
                code = process.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGKILL)
                process.wait()
                raise
    output = log.read_text()
    sampled = interrupted and 'received signal SIGINT' in output and '#0' in output
    row = dict(name=name,log=log.name,command=command,returncode=code,elapsed_seconds=time.monotonic()-start,
               interrupted=interrupted,sample_obtained=sampled,debugger_reaped=True)
    report['cases'].append(row)
    (A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
    print(name,'sampled=',sampled,'elapsed=',round(row['elapsed_seconds'],2),flush=True)
    assert sampled or (not interrupted and code == 0 and '1 passed;' in output)
verify()
report['all_sources_unchanged'] = True
report['all_processes_reaped'] = True
(A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
print('Bounded diagnostic reaped; source and executable hashes unchanged.',flush=True)
