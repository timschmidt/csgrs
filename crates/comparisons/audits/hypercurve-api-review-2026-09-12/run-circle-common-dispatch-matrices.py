from pathlib import Path
import hashlib, json, subprocess, time
root = Path('/home/tim/Documents/GitHub/workspace')
audit = root / 'hypercurve-api-review-2026-09-12'
lib = root / 'hypercurve/target/release/deps/libhypercurve-2c7ff6f3fbe07653.rlib'
sha = hashlib.sha256(lib.read_bytes()).hexdigest()
matrices = [
    ('circle-common-dispatch-baseline', 'circle-common-dispatch-probe-final'),
    ('circle-common-dispatch-matrix', 'circle-common-dispatch-matrix'),
    ('general-trim-matrix', 'circle-common-dispatch-general-trim-matrix'),
    ('constant-image-trim-matrix', 'circle-common-dispatch-constant-image-trim-matrix'),
]
for source, stem in matrices:
    command = ['rustc', '--edition=2024', '-O', str(audit/(source+'.rs')), '--extern', 'hypercurve='+str(lib), '-L', 'dependency='+str(lib.parent), '-o', str(audit/stem)]
    compile_result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=120)
    (audit/(stem+'-compile.log')).write_text(compile_result.stdout)
    if compile_result.returncode:
        print(compile_result.stdout)
        raise SystemExit(compile_result.returncode)
    start = time.monotonic()
    with (audit/(stem+'.log')).open('w') as output:
        result = subprocess.run([str(audit/stem)],stdout=output,stderr=subprocess.STDOUT,timeout=120)
    data = {'returncode':result.returncode,'elapsed_seconds':time.monotonic()-start,'normal_library_sha256':sha,'log':stem+'.log','source':source+'.rs','command':command}
    lines = (audit/(stem+'.log')).read_text().splitlines()
    for line in reversed(lines):
        try:
            counts=json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(counts,dict):
            data['counts']=counts
            break
    (audit/(stem+'.json')).write_text(json.dumps(data,indent=2)+'\n')
    print(stem, data, flush=True)
    if result.returncode:
        raise SystemExit(result.returncode)
assert hashlib.sha256(lib.read_bytes()).hexdigest() == sha
