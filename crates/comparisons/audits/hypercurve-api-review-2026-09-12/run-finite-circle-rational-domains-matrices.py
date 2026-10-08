from pathlib import Path
import hashlib, json, subprocess, time
root = Path('/home/tim/Documents/GitHub/workspace')
audit = root / 'hypercurve-api-review-2026-09-12'
lib = root / 'hypercurve/target/release/deps/libhypercurve-2c7ff6f3fbe07653.rlib'
sha = hashlib.sha256(lib.read_bytes()).hexdigest()
matrices = [('selected-circle-source-replay-public', 'finite-circle-rational-domains-public-replay'), ('finite-circle-affine-parallel-public', 'finite-circle-rational-domains-affine-parallel-replay'), ('finite-circle-affine-completed', 'finite-circle-rational-domains-exterior-completed'), ('selected-circle-tangent-identity-matrix', 'finite-circle-rational-domains-tangent-matrix'), ('finite-circle-domain-next', 'finite-circle-rational-domains-finite-circle-probe'), ('native-rational-publication-family-matrix', 'finite-circle-rational-domains-family-matrix'), ('native-rational-publication-completed', 'finite-circle-rational-domains-completed'), ('native-rational-publication-unit-replay', 'finite-circle-rational-domains-unit-replay'), ('finite-domain-chord-matrix', 'finite-circle-rational-domains-retained-chord-matrix'), ('finite-domain-chord-cubic-replay', 'finite-circle-rational-domains-cubic-replay'), ('finite-rational-pairs-matrix', 'finite-circle-rational-domains-finite-matrix'), ('finite-domain-baseline', 'finite-circle-rational-domains-baseline-recheck'), ('region-overlap-correspondence-replay-matrix', 'finite-circle-rational-domains-replay-matrix-run'), ('analytic-common-dispatch-completed', 'finite-circle-rational-domains-analytic-probe'), ('singleton-endpoint-parameter-completed', 'finite-circle-rational-domains-endpoint-probe'), ('native-point-components-completed', 'finite-circle-rational-domains-completed-probe'), ('circle-common-dispatch-baseline', 'finite-circle-rational-domains-probe-final'), ('circle-common-dispatch-matrix', 'finite-circle-rational-domains-circle-matrix'), ('region-overlap-correspondence-carrier-matrix', 'finite-circle-rational-domains-region-carrier-matrix'), ('general-trim-matrix', 'finite-circle-rational-domains-general-trim-matrix'), ('constant-image-trim-matrix', 'finite-circle-rational-domains-constant-image-trim-matrix')]
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
