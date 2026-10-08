from pathlib import Path
import hashlib, json, subprocess, time
root = Path('/home/tim/Documents/GitHub/workspace')
audit = root / 'hypercurve-api-review-2026-09-12'
lib = root / 'hypercurve/target/release/deps/libhypercurve-2c7ff6f3fbe07653.rlib'
sha = hashlib.sha256(lib.read_bytes()).hexdigest()
matrices = [('selected-circle-source-replay-public', 'circle-closed-clipping-public-replay'), ('finite-circle-affine-parallel-public', 'circle-closed-clipping-affine-parallel-replay'), ('finite-circle-affine-completed', 'circle-closed-clipping-exterior-completed'), ('selected-circle-tangent-identity-matrix', 'circle-closed-clipping-tangent-matrix'), ('finite-circle-domain-next', 'circle-closed-clipping-finite-circle-probe'), ('native-rational-publication-family-matrix', 'circle-closed-clipping-family-matrix'), ('native-rational-publication-completed', 'circle-closed-clipping-completed'), ('native-rational-publication-unit-replay', 'circle-closed-clipping-unit-replay'), ('finite-domain-chord-matrix', 'circle-closed-clipping-retained-chord-matrix'), ('finite-domain-chord-cubic-replay', 'circle-closed-clipping-cubic-replay'), ('finite-rational-pairs-matrix', 'circle-closed-clipping-finite-matrix'), ('finite-domain-baseline', 'circle-closed-clipping-baseline-recheck'), ('region-overlap-correspondence-replay-matrix', 'circle-closed-clipping-replay-matrix-run'), ('analytic-common-dispatch-completed', 'circle-closed-clipping-analytic-probe'), ('singleton-endpoint-parameter-completed', 'circle-closed-clipping-endpoint-probe'), ('native-point-components-completed', 'circle-closed-clipping-completed-probe'), ('circle-common-dispatch-baseline', 'circle-closed-clipping-probe-final'), ('circle-common-dispatch-matrix', 'circle-closed-clipping-circle-matrix'), ('region-overlap-correspondence-carrier-matrix', 'circle-closed-clipping-region-carrier-matrix'), ('general-trim-matrix', 'circle-closed-clipping-general-trim-matrix'), ('constant-image-trim-matrix', 'circle-closed-clipping-constant-image-trim-matrix')]
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
