from pathlib import Path
import hashlib, json, subprocess, time
root = Path('/home/tim/Documents/GitHub/workspace')
audit = root / 'hypercurve-api-review-2026-09-12'
artifacts = [json.loads(l) for l in (audit/'finite-parallel-affine-test-build.jsonl').read_text().splitlines()]
lib = Path(next(f for x in artifacts if x.get('reason') == 'compiler-artifact' and x['target']['name'] == 'hypercurve' and not x['profile']['test'] for f in x['filenames'] if f.endswith('.rlib')))
sha = hashlib.sha256(lib.read_bytes()).hexdigest()
matrices = [('selected-circle-source-replay-public', 'finite-parallel-affine-public-replay'), ('finite-circle-affine-parallel-public', 'finite-parallel-affine-affine-parallel-replay'), ('finite-circle-affine-completed', 'finite-parallel-affine-exterior-completed'), ('selected-circle-tangent-identity-matrix', 'finite-parallel-affine-tangent-matrix'), ('finite-circle-domain-next', 'finite-parallel-affine-finite-circle-probe'), ('native-rational-publication-family-matrix', 'finite-parallel-affine-family-matrix'), ('native-rational-publication-completed', 'finite-parallel-affine-completed'), ('native-rational-publication-unit-replay', 'finite-parallel-affine-unit-replay'), ('finite-domain-chord-matrix', 'finite-parallel-affine-retained-chord-matrix'), ('finite-domain-chord-cubic-replay', 'finite-parallel-affine-cubic-replay'), ('finite-rational-pairs-matrix', 'finite-parallel-affine-finite-matrix'), ('finite-domain-baseline', 'finite-parallel-affine-baseline-recheck'), ('region-overlap-correspondence-replay-matrix', 'finite-parallel-affine-replay-matrix-run'), ('analytic-common-dispatch-completed', 'finite-parallel-affine-analytic-probe'), ('singleton-endpoint-parameter-completed', 'finite-parallel-affine-endpoint-probe'), ('native-point-components-completed', 'finite-parallel-affine-completed-probe'), ('circle-common-dispatch-baseline', 'finite-parallel-affine-probe-final'), ('circle-common-dispatch-matrix', 'finite-parallel-affine-circle-matrix'), ('region-overlap-correspondence-carrier-matrix', 'finite-parallel-affine-region-carrier-matrix'), ('general-trim-matrix', 'finite-parallel-affine-general-trim-matrix'), ('constant-image-trim-matrix', 'finite-parallel-affine-constant-image-trim-matrix')]
matrices += [('finite-circle-rational-domains-public', 'finite-parallel-affine-region-public'), ('selected-fillet-region-normalization-explicit', 'finite-parallel-affine-region-explicit'), ('finite-region-bounds-public', 'finite-parallel-affine-bounds-public'), ('finite-region-winding-public', 'finite-parallel-affine-winding-public')]
matrices.insert(0, ('finite-region-preparation-public','finite-parallel-affine-trim-public'))
matrices.insert(0, ('finite-region-self-domain-public','finite-parallel-affine-self-public'))
matrices.insert(0, ('finite-region-self-retrace-public','finite-parallel-affine-retrace-public'))
for source, stem in matrices:
    command = ['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustc', '--edition=2024', '-O', str(audit/(source+'.rs')), '--extern', 'hypercurve='+str(lib), '-L', 'dependency='+str(lib.parent), '-o', str(audit/stem)]
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
