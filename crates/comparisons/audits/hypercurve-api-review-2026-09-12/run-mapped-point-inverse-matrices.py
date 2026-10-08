from pathlib import Path
import hashlib, json, subprocess, sys, time
root = Path('/home/tim/Documents/GitHub/workspace')
audit = root / 'hypercurve-api-review-2026-09-12'
artifacts = [json.loads(l) for l in (audit/'mapped-point-inverse-test-build.jsonl').read_text().splitlines()]
lib = Path(next(f for x in artifacts if x.get('reason') == 'compiler-artifact' and x['target']['name'] == 'hypercurve' and not x['profile']['test'] for f in x['filenames'] if f.endswith('.rlib')))
sha = hashlib.sha256(lib.read_bytes()).hexdigest()
matrices = [('selected-circle-source-replay-public', 'mapped-point-inverse-public-replay'), ('finite-circle-affine-parallel-public', 'mapped-point-inverse-affine-parallel-replay'), ('finite-circle-affine-completed', 'mapped-point-inverse-exterior-completed'), ('selected-circle-tangent-identity-matrix', 'mapped-point-inverse-tangent-matrix'), ('finite-circle-domain-next', 'mapped-point-inverse-finite-circle-probe'), ('native-rational-publication-family-matrix', 'mapped-point-inverse-family-matrix'), ('native-rational-publication-completed', 'mapped-point-inverse-completed'), ('native-rational-publication-unit-replay', 'mapped-point-inverse-unit-replay'), ('finite-domain-chord-matrix', 'mapped-point-inverse-retained-chord-matrix'), ('finite-domain-chord-cubic-replay', 'mapped-point-inverse-cubic-replay'), ('finite-rational-pairs-matrix', 'mapped-point-inverse-finite-matrix'), ('finite-domain-baseline', 'mapped-point-inverse-baseline-recheck'), ('region-overlap-correspondence-replay-matrix', 'mapped-point-inverse-replay-matrix-run'), ('analytic-common-dispatch-completed', 'mapped-point-inverse-analytic-probe'), ('singleton-endpoint-parameter-completed', 'mapped-point-inverse-endpoint-probe'), ('native-point-components-completed', 'mapped-point-inverse-completed-probe'), ('circle-common-dispatch-baseline', 'mapped-point-inverse-probe-final'), ('circle-common-dispatch-matrix', 'mapped-point-inverse-circle-matrix'), ('region-overlap-correspondence-carrier-matrix', 'mapped-point-inverse-region-carrier-matrix'), ('general-trim-matrix', 'mapped-point-inverse-general-trim-matrix'), ('constant-image-trim-matrix', 'mapped-point-inverse-constant-image-trim-matrix')]
matrices += [('finite-circle-rational-domains-public', 'mapped-point-inverse-region-public'), ('selected-fillet-region-normalization-explicit', 'mapped-point-inverse-region-explicit'), ('finite-region-bounds-public', 'mapped-point-inverse-bounds-public'), ('finite-region-winding-public', 'mapped-point-inverse-winding-public')]
matrices.insert(0, ('finite-region-preparation-public','mapped-point-inverse-trim-public'))
matrices.insert(0, ('finite-region-self-domain-public','mapped-point-inverse-self-public'))
matrices.insert(0, ('finite-region-self-retrace-public','mapped-point-inverse-retrace-public'))
matrices += [('finite-parallel-affine-public','mapped-point-inverse-affine-public'), ('finite-parallel-constructor-public','mapped-point-inverse-constructor-public'), ('finite-parallel-admission-region-normalized-public','mapped-point-inverse-admission-region-entry'), ('finite-analytic-pair-domains-public','mapped-point-inverse-previous-pairs'), ('finite-analytic-pair-domains-region-public','mapped-point-inverse-region-entry')]
matrices += [('finite-analytic-native-poles-public', 'mapped-point-inverse-native-poles-public')]
matrices += [('finite-circle-analytic-domains-public', 'mapped-point-inverse-analytic-public')]
matrices += [('finite-circle-native-poles-charts-public', 'mapped-point-inverse-rational-public')]
matrices += [('finite-circle-native-poles-public', 'mapped-point-inverse-poles-public')]
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
