from pathlib import Path
import hashlib, json, subprocess, sys, time
root = Path('/home/tim/Documents/GitHub/workspace')
audit = root / 'hypercurve-api-review-2026-09-12'
artifacts = [json.loads(l) for l in (audit/'finite-projective-parameter-test-build.jsonl').read_text().splitlines()]
lib = Path(next(f for x in artifacts if x.get('reason') == 'compiler-artifact' and x['target']['name'] == 'hypercurve' and not x['profile']['test'] for f in x['filenames'] if f.endswith('.rlib')))
sha = hashlib.sha256(lib.read_bytes()).hexdigest()
matrices = [('selected-circle-source-replay-public', 'finite-projective-parameter-public-replay'), ('finite-circle-affine-parallel-public', 'finite-projective-parameter-affine-parallel-replay'), ('finite-circle-affine-completed', 'finite-projective-parameter-exterior-completed'), ('selected-circle-tangent-identity-matrix', 'finite-projective-parameter-tangent-matrix'), ('finite-circle-domain-next', 'finite-projective-parameter-finite-circle-probe'), ('native-rational-publication-family-matrix', 'finite-projective-parameter-family-matrix'), ('native-rational-publication-completed', 'finite-projective-parameter-completed'), ('native-rational-publication-unit-replay', 'finite-projective-parameter-unit-replay'), ('finite-domain-chord-matrix', 'finite-projective-parameter-retained-chord-matrix'), ('finite-domain-chord-cubic-replay', 'finite-projective-parameter-cubic-replay'), ('finite-rational-pairs-matrix', 'finite-projective-parameter-finite-matrix'), ('finite-domain-baseline', 'finite-projective-parameter-baseline-recheck'), ('region-overlap-correspondence-replay-matrix', 'finite-projective-parameter-replay-matrix-run'), ('analytic-common-dispatch-completed', 'finite-projective-parameter-analytic-probe'), ('singleton-endpoint-parameter-completed', 'finite-projective-parameter-endpoint-probe'), ('native-point-components-completed', 'finite-projective-parameter-completed-probe'), ('circle-common-dispatch-baseline', 'finite-projective-parameter-probe-final'), ('circle-common-dispatch-matrix', 'finite-projective-parameter-circle-matrix'), ('region-overlap-correspondence-carrier-matrix', 'finite-projective-parameter-region-carrier-matrix'), ('general-trim-matrix', 'finite-projective-parameter-general-trim-matrix'), ('constant-image-trim-matrix', 'finite-projective-parameter-constant-image-trim-matrix')]
matrices += [('finite-circle-rational-domains-public', 'finite-projective-parameter-region-public'), ('selected-fillet-region-normalization-explicit', 'finite-projective-parameter-region-explicit'), ('finite-region-bounds-public', 'finite-projective-parameter-bounds-public'), ('finite-region-winding-public', 'finite-projective-parameter-winding-public')]
matrices.insert(0, ('finite-region-preparation-public','finite-projective-parameter-trim-public'))
matrices.insert(0, ('finite-region-self-domain-public','finite-projective-parameter-self-public'))
matrices.insert(0, ('finite-region-self-retrace-public','finite-projective-parameter-retrace-public'))
matrices += [('finite-parallel-affine-public','finite-projective-parameter-affine-public'), ('finite-parallel-constructor-public','finite-projective-parameter-constructor-public'), ('finite-parallel-admission-region-normalized-public','finite-projective-parameter-admission-region-entry'), ('finite-analytic-pair-domains-public','finite-projective-parameter-previous-pairs'), ('finite-analytic-pair-domains-region-public','finite-projective-parameter-region-entry')]
matrices += [('finite-analytic-native-poles-public', 'finite-projective-parameter-native-poles-public')]
matrices += [('finite-circle-analytic-domains-public', 'finite-projective-parameter-analytic-public')]
matrices += [('finite-circle-native-poles-charts-public', 'finite-projective-parameter-rational-public')]
matrices += [('finite-circle-native-poles-public', 'finite-projective-parameter-poles-public')]
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
    data = {'returncode':result.returncode,'elapsed_seconds':time.monotonic()-start,'normal_library_sha256':sha,'log':stem+'.log','source':source+'.rs','command':command,'run_command':[str(audit/stem), *(['2'] if source == 'finite-circle-native-poles-charts-public' else [])],'source_sha256':source_sha,'executable_sha256':executable_sha}
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
