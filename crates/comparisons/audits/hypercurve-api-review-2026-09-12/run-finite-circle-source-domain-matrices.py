from pathlib import Path
import hashlib, json, subprocess, sys, time
root = Path('/home/tim/Documents/GitHub/workspace')
audit = root / 'hypercurve-api-review-2026-09-12'
artifacts = [json.loads(l) for l in (audit/'finite-circle-source-domain-test-build.jsonl').read_text().splitlines()]
lib = Path(next(f for x in artifacts if x.get('reason') == 'compiler-artifact' and x['target']['name'] == 'hypercurve' and not x['profile']['test'] for f in x['filenames'] if f.endswith('.rlib')))
sha = hashlib.sha256(lib.read_bytes()).hexdigest()
matrices = [('selected-circle-source-replay-public', 'finite-circle-source-domain-public-replay'), ('finite-circle-affine-parallel-public', 'finite-circle-source-domain-affine-parallel-replay'), ('finite-circle-affine-completed', 'finite-circle-source-domain-exterior-completed'), ('selected-circle-tangent-identity-matrix', 'finite-circle-source-domain-tangent-matrix'), ('finite-circle-domain-next', 'finite-circle-source-domain-finite-circle-probe'), ('native-rational-publication-family-matrix', 'finite-circle-source-domain-family-matrix'), ('native-rational-publication-completed', 'finite-circle-source-domain-completed'), ('native-rational-publication-unit-replay', 'finite-circle-source-domain-unit-replay'), ('finite-domain-chord-matrix', 'finite-circle-source-domain-retained-chord-matrix'), ('finite-domain-chord-cubic-replay', 'finite-circle-source-domain-cubic-replay'), ('finite-rational-pairs-matrix', 'finite-circle-source-domain-finite-matrix'), ('finite-domain-baseline', 'finite-circle-source-domain-baseline-recheck'), ('region-overlap-correspondence-replay-matrix', 'finite-circle-source-domain-replay-matrix-run'), ('analytic-common-dispatch-completed', 'finite-circle-source-domain-analytic-probe'), ('singleton-endpoint-parameter-completed', 'finite-circle-source-domain-endpoint-probe'), ('native-point-components-completed', 'finite-circle-source-domain-completed-probe'), ('circle-common-dispatch-baseline', 'finite-circle-source-domain-probe-final'), ('circle-common-dispatch-matrix', 'finite-circle-source-domain-circle-matrix'), ('region-overlap-correspondence-carrier-matrix', 'finite-circle-source-domain-region-carrier-matrix'), ('general-trim-matrix', 'finite-circle-source-domain-general-trim-matrix'), ('constant-image-trim-matrix', 'finite-circle-source-domain-constant-image-trim-matrix')]
matrices += [('finite-circle-rational-domains-public', 'finite-circle-source-domain-region-public'), ('selected-fillet-region-normalization-explicit', 'finite-circle-source-domain-region-explicit'), ('finite-region-bounds-public', 'finite-circle-source-domain-bounds-public'), ('finite-region-winding-public', 'finite-circle-source-domain-winding-public')]
matrices.insert(0, ('finite-region-preparation-public','finite-circle-source-domain-trim-public'))
matrices.insert(0, ('finite-region-self-domain-public','finite-circle-source-domain-self-public'))
matrices.insert(0, ('finite-region-self-retrace-public','finite-circle-source-domain-retrace-public'))
matrices += [('finite-parallel-affine-public','finite-circle-source-domain-affine-public'), ('finite-parallel-constructor-public','finite-circle-source-domain-constructor-public'), ('finite-parallel-admission-region-normalized-public','finite-circle-source-domain-admission-region-entry'), ('finite-analytic-pair-domains-public','finite-circle-source-domain-previous-pairs'), ('finite-analytic-pair-domains-region-public','finite-circle-source-domain-region-entry')]
matrices += [('finite-analytic-native-poles-public', 'finite-circle-source-domain-native-poles-public')]
matrices += [('finite-circle-analytic-domains-public', 'finite-circle-source-domain-analytic-public')]
matrices += [('finite-circle-native-poles-charts-public', 'finite-circle-source-domain-rational-public')]
matrices += [('finite-circle-native-poles-public', 'finite-circle-source-domain-poles-public')]
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
