from pathlib import Path
import hashlib, json, subprocess, time
root = Path('/home/tim/Documents/GitHub/workspace')
audit = root / 'hypercurve-api-review-2026-09-12'
lib = root / 'hypercurve/target/release/deps/libhypercurve-2c7ff6f3fbe07653.rlib'
sha = hashlib.sha256(lib.read_bytes()).hexdigest()
matrices = [
    ('selected-circle-tangent-identity-matrix', 'selected-circle-tangent-identity-tangent-matrix'),
    ('finite-circle-domain-next', 'selected-circle-tangent-identity-finite-circle-probe'),
    ('native-rational-publication-family-matrix', 'selected-circle-tangent-identity-family-matrix'),
    ('native-rational-publication-completed', 'selected-circle-tangent-identity-completed'),
    ('native-rational-publication-unit-replay', 'selected-circle-tangent-identity-unit-replay'),
    ('finite-domain-chord-matrix', 'selected-circle-tangent-identity-retained-chord-matrix'),
    ('finite-domain-chord-cubic-replay', 'selected-circle-tangent-identity-cubic-replay'),
    ('finite-rational-pairs-matrix', 'selected-circle-tangent-identity-finite-matrix'),
    ('finite-domain-baseline', 'selected-circle-tangent-identity-baseline-recheck'),
    ('region-overlap-correspondence-replay-matrix', 'selected-circle-tangent-identity-replay-matrix-run'),
    ('analytic-common-dispatch-completed', 'selected-circle-tangent-identity-analytic-probe'),
    ('singleton-endpoint-parameter-completed', 'selected-circle-tangent-identity-endpoint-probe'),
    ('native-point-components-completed', 'selected-circle-tangent-identity-completed-probe'),
    ('circle-common-dispatch-baseline', 'selected-circle-tangent-identity-probe-final'),
    ('circle-common-dispatch-matrix', 'selected-circle-tangent-identity-circle-matrix'),
    ('region-overlap-correspondence-carrier-matrix', 'selected-circle-tangent-identity-region-carrier-matrix'),
    ('general-trim-matrix', 'selected-circle-tangent-identity-general-trim-matrix'),
    ('constant-image-trim-matrix', 'selected-circle-tangent-identity-constant-image-trim-matrix'),
]
for source, stem in matrices:
    command = ['rustc', '--edition=2024', '-O', str(audit/(source+'.rs')), '--extern', 'hypercurve='+str(lib), '-L', 'dependency='+str(lib.parent), '-o', str(audit/stem)]
    compile_result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=120)
    (audit/(stem+'-compile.log')).write_text(compile_result.stdout)
    if compile_result.returncode:
        print(compile_result.stdout)
        raise SystemExit(compile_result.returncode)
    source_sha = hashlib.sha256((audit/(source+'.rs')).read_bytes()).hexdigest()
    executable_sha = hashlib.sha256((audit/stem).read_bytes()).hexdigest()
    start = time.monotonic()
    with (audit/(stem+'.log')).open('w') as output:
        result = subprocess.run([str(audit/stem)],stdout=output,stderr=subprocess.STDOUT,timeout=360 if source == 'region-overlap-correspondence-carrier-matrix' else 120)
    data = {'returncode':result.returncode,'elapsed_seconds':time.monotonic()-start,'normal_library_sha256':sha,'log':stem+'.log','source':source+'.rs','command':command,'source_sha256':source_sha,'executable_sha256':executable_sha}
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
