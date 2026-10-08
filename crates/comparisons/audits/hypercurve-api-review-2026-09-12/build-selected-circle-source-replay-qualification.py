from pathlib import Path
import hashlib, json, os, subprocess, time
root = Path('/home/tim/Documents/GitHub/workspace')
audit = root / 'hypercurve-api-review-2026-09-12'
prefix = 'selected-circle-source-replay'
rows = []
for name in ['hypercurve', 'hyperbrep']:
    paths = subprocess.check_output(['git', 'ls-files', '-c', '-o', '--exclude-standard', '-z'], cwd=root/name).decode().split('\0')
    for relative in sorted(set(filter(None, paths))):
        path = root/name/relative
        if path.is_file():
            rows.append({'file':str(path.relative_to(root)), 'sha256':hashlib.sha256(path.read_bytes()).hexdigest()})
(audit/(prefix+'-source-before-tests.json')).write_text(json.dumps(rows,indent=2)+'\n')
env = dict(os.environ, CARGO_BUILD_JOBS='2')
for name in ['hypercurve','hyperbrep']:
    suffix = '' if name == 'hypercurve' else '-hyperbrep'
    for kind,args in [
        ('check',['check','--all-targets','--no-default-features','--offline']),
        ('test-build',['test','--release','--all-features','--offline','--lib','--tests','--no-run','--message-format=json']),
    ]:
        stem = prefix+suffix+'-'+kind
        start = time.monotonic()
        with (audit/(stem+'.log')).open('w') as error:
            if kind == 'test-build':
                with (audit/(stem+'.jsonl')).open('w') as output:
                    result = subprocess.run(['cargo',*args],cwd=root/name,env=env,stdout=output,stderr=error,timeout=600)
            else:
                result = subprocess.run(['cargo',*args],cwd=root/name,env=env,stdout=error,stderr=subprocess.STDOUT,timeout=300)
        (audit/(stem+'.exit')).write_text(str(result.returncode)+'\n')
        print(stem,result.returncode,round(time.monotonic()-start,2),flush=True)
        if result.returncode:
            print((audit/(stem+'.log')).read_text()[-6000:],flush=True)
            raise SystemExit(result.returncode)
print('builds complete; sources remain frozen',flush=True)
