from pathlib import Path
import hashlib, json, subprocess, time
root = Path('/home/tim/Documents/GitHub/workspace')
audit = root / 'hypercurve-api-review-2026-09-12'
artifacts = [json.loads(l) for l in (audit/'finite-region-preparation-test-build.jsonl').read_text().splitlines()]
lib = Path(next(f for x in artifacts if x.get('reason') == 'compiler-artifact' and x['target']['name'] == 'hypercurve' and not x['profile']['test'] for f in x['filenames'] if f.endswith('.rlib')))
sha = hashlib.sha256(lib.read_bytes()).hexdigest()
matrices = [('selected-circle-source-replay-public', 'finite-region-preparation-public-replay'), ('finite-circle-affine-parallel-public', 'finite-region-preparation-affine-parallel-replay'), ('finite-circle-affine-completed', 'finite-region-preparation-exterior-completed'), ('selected-circle-tangent-identity-matrix', 'finite-region-preparation-tangent-matrix'), ('finite-circle-domain-next', 'finite-region-preparation-finite-circle-probe'), ('native-rational-publication-family-matrix', 'finite-region-preparation-family-matrix'), ('native-rational-publication-completed', 'finite-region-preparation-completed'), ('native-rational-publication-unit-replay', 'finite-region-preparation-unit-replay'), ('finite-domain-chord-matrix', 'finite-region-preparation-retained-chord-matrix'), ('finite-domain-chord-cubic-replay', 'finite-region-preparation-cubic-replay'), ('finite-rational-pairs-matrix', 'finite-region-preparation-finite-matrix'), ('finite-domain-baseline', 'finite-region-preparation-baseline-recheck'), ('region-overlap-correspondence-replay-matrix', 'finite-region-preparation-replay-matrix-run'), ('analytic-common-dispatch-completed', 'finite-region-preparation-analytic-probe'), ('singleton-endpoint-parameter-completed', 'finite-region-preparation-endpoint-probe'), ('native-point-components-completed', 'finite-region-preparation-completed-probe'), ('circle-common-dispatch-baseline', 'finite-region-preparation-probe-final'), ('circle-common-dispatch-matrix', 'finite-region-preparation-circle-matrix'), ('region-overlap-correspondence-carrier-matrix', 'finite-region-preparation-region-carrier-matrix'), ('general-trim-matrix', 'finite-region-preparation-general-trim-matrix'), ('constant-image-trim-matrix', 'finite-region-preparation-constant-image-trim-matrix')]
matrices += [('finite-circle-rational-domains-public', 'finite-region-preparation-region-public'), ('selected-fillet-region-normalization-explicit', 'finite-region-preparation-region-explicit'), ('finite-region-bounds-public', 'finite-region-preparation-bounds-public'), ('finite-region-winding-public', 'finite-region-preparation-winding-public')]
matrices.insert(0, ('finite-region-preparation-public','finite-region-preparation-trim-public'))
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
