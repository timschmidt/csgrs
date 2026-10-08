from pathlib import Path
import hashlib, json, os, shutil, subprocess, time
A = Path(__file__).resolve().parent
W = A.parent
prefix = 'chord-local-decline-20260924-v2'
setup = json.loads((A / f'{prefix}-sources.json').read_text())
root = Path(setup['root'])
base = json.loads((A / setup['base_manifest']).read_text())
def verify():
    assert subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=W/'hypercurve', text=True).strip() == setup['base_head']
    assert not subprocess.check_output(['git', 'status', '--porcelain=v1'], cwd=W/'hypercurve')
    for name, sha in base.items():
        assert hashlib.sha256((W/name).read_bytes()).hexdigest() == sha, name
        assert hashlib.sha256((root/name).read_bytes()).hexdigest() == setup['probe_sources'][name], name
verify()
env = dict(os.environ, **json.loads((A/'opposed-endpoint-contact-full1-build-settings.json').read_text()))
env.pop('HYPERCURVE_DEBUG_RATIONAL_BLOCKER', None)
command = ['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo', 'test', '--lib', '--release', '--all-features', '--no-run', '--message-format=json', '--locked', '--offline']
print('Building separately bound diagnostic executable', flush=True)
with (A/f'{prefix}-build.jsonl').open('w') as out, (A/f'{prefix}-build.log').open('w') as err:
    code = subprocess.run(command, cwd=root/'hypercurve', env=env, stdout=out, stderr=err, timeout=1200).returncode
verify()
assert code == 0, (A/f'{prefix}-build.log').read_text()[-5000:]
rows = [json.loads(line) for line in (A/f'{prefix}-build.jsonl').read_text().splitlines()]
artifact = next(row for row in rows if row.get('reason') == 'compiler-artifact' and row['target']['name'] == 'hypercurve' and row.get('executable'))
assert not artifact['fresh']
binary = A/f'{prefix}-libtest'; shutil.copy2(artifact['executable'], binary)
sha = hashlib.sha256(binary.read_bytes()).hexdigest()
case = 'bezier_region::tests::one_fragment_nonzero_parallel_loop_extends_chamfer_cuts_on_one_finite_envelope'
command = [str(binary), '--exact', case, '--test-threads=1', '--nocapture']
reports = []
for mode in ['bounded', 'complete']:
    if mode == 'complete': env['HYPERCURVE_DIAGNOSTIC_COMPLETE_LOCAL'] = '1'
    else: env.pop('HYPERCURVE_DIAGNOSTIC_COMPLETE_LOCAL', None)
    start = time.monotonic()
    with (A/f'{prefix}-{mode}.log').open('w') as out:
        try:
            code = subprocess.run(command, cwd=root/'hypercurve', env=env, stdout=out, stderr=subprocess.STDOUT, timeout=20).returncode
        except subprocess.TimeoutExpired:
            code = 'timeout'
    verify()
    assert hashlib.sha256(binary.read_bytes()).hexdigest() == sha
    reports.append(dict(mode=mode, returncode=code, elapsed_seconds=time.monotonic()-start, limit_seconds=20))
    print('Diagnostic case terminal:', mode, code, flush=True)
    print((A/f'{prefix}-{mode}.log').read_text()[-10000:], flush=True)
report = dict(base_head=setup['base_head'], source_manifest=f'{prefix}-sources.json', binary_sha256=sha, command=command, cases=reports, diagnostic_only=True, all_owned_processes_reaped=True, all_sources_unchanged=True)
(A/f'{prefix}-terminal.json').write_text(json.dumps(report, indent=2)+'\n')
print('Both diagnostic children and the build are reaped.', flush=True)
