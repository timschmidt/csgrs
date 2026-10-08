from pathlib import Path
import hashlib, json, subprocess, sys, time
root = Path('/home/tim/Documents/GitHub/workspace')
audit = root / 'hypercurve-api-review-2026-09-12'
artifacts = [json.loads(l) for l in (audit/'selected-point-frame-replay-test-build.jsonl').read_text().splitlines()]
lib = Path(next(f for x in artifacts if x.get('reason') == 'compiler-artifact' and x['target']['name'] == 'hypercurve' and not x['profile']['test'] for f in x['filenames'] if f.endswith('.rlib')))
sha = hashlib.sha256(lib.read_bytes()).hexdigest()
matrices = [('selected-circle-source-replay-public', 'selected-point-frame-replay-public-replay'), ('finite-circle-affine-parallel-public', 'selected-point-frame-replay-affine-parallel-replay'), ('finite-circle-affine-completed', 'selected-point-frame-replay-exterior-completed'), ('selected-circle-tangent-identity-matrix', 'selected-point-frame-replay-tangent-matrix'), ('finite-circle-domain-next', 'selected-point-frame-replay-finite-circle-probe'), ('native-rational-publication-family-matrix', 'selected-point-frame-replay-family-matrix'), ('native-rational-publication-completed', 'selected-point-frame-replay-completed'), ('native-rational-publication-unit-replay', 'selected-point-frame-replay-unit-replay'), ('finite-domain-chord-matrix', 'selected-point-frame-replay-retained-chord-matrix'), ('finite-domain-chord-cubic-replay', 'selected-point-frame-replay-cubic-replay'), ('finite-rational-pairs-matrix', 'selected-point-frame-replay-finite-matrix'), ('finite-domain-baseline', 'selected-point-frame-replay-baseline-recheck'), ('region-overlap-correspondence-replay-matrix', 'selected-point-frame-replay-replay-matrix-run'), ('analytic-common-dispatch-completed', 'selected-point-frame-replay-analytic-probe'), ('singleton-endpoint-parameter-completed', 'selected-point-frame-replay-endpoint-probe'), ('native-point-components-completed', 'selected-point-frame-replay-completed-probe'), ('circle-common-dispatch-baseline', 'selected-point-frame-replay-probe-final'), ('circle-common-dispatch-matrix', 'selected-point-frame-replay-circle-matrix'), ('region-overlap-correspondence-carrier-matrix', 'selected-point-frame-replay-region-carrier-matrix'), ('general-trim-matrix', 'selected-point-frame-replay-general-trim-matrix'), ('constant-image-trim-matrix', 'selected-point-frame-replay-constant-image-trim-matrix')]
matrices += [('finite-circle-rational-domains-public', 'selected-point-frame-replay-region-public'), ('selected-fillet-region-normalization-explicit', 'selected-point-frame-replay-region-explicit'), ('finite-region-bounds-public', 'selected-point-frame-replay-bounds-public'), ('finite-region-winding-public', 'selected-point-frame-replay-winding-public')]
matrices.insert(0, ('finite-region-preparation-public','selected-point-frame-replay-trim-public'))
matrices.insert(0, ('finite-region-self-domain-public','selected-point-frame-replay-self-public'))
matrices.insert(0, ('finite-region-self-retrace-public','selected-point-frame-replay-retrace-public'))
matrices += [('finite-parallel-affine-public','selected-point-frame-replay-affine-public'), ('finite-parallel-constructor-public','selected-point-frame-replay-constructor-public'), ('finite-parallel-admission-region-normalized-public','selected-point-frame-replay-admission-region-entry'), ('finite-analytic-pair-domains-public','selected-point-frame-replay-previous-pairs'), ('finite-analytic-pair-domains-region-public','selected-point-frame-replay-region-entry')]
matrices += [('finite-analytic-native-poles-public', 'selected-point-frame-replay-native-poles-public')]
matrices += [('finite-circle-analytic-domains-public', 'selected-point-frame-replay-analytic-public')]
matrices += [('finite-circle-native-poles-charts-public', 'selected-point-frame-replay-rational-public')]
mode = sys.argv[1] if len(sys.argv)>1 else 'all'
assert mode in ['all','new','existing']
if mode == 'new': matrices = matrices[-1:]
if mode == 'existing': matrices = matrices[:-1]
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
        result = subprocess.run([str(audit/stem), *(['2'] if source == 'finite-circle-native-poles-charts-public' else [])],stdout=output,stderr=subprocess.STDOUT,timeout=360 if source == 'region-overlap-correspondence-carrier-matrix' else 120)
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
