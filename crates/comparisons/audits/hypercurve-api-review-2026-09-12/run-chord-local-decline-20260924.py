from pathlib import Path
import hashlib, json, os, shutil, subprocess, time
A = Path(__file__).resolve().parent
W = A.parent
prefix = 'chord-local-decline-20260924-v1'
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
start = time.monotonic()
with (A/f'{prefix}-case.log').open('w') as out:
    try:
        code = subprocess.run(command, cwd=root/'hypercurve', env=env, stdout=out, stderr=subprocess.STDOUT, timeout=20).returncode
    except subprocess.TimeoutExpired:
        code = 'timeout'
verify()
assert hashlib.sha256(binary.read_bytes()).hexdigest() == sha
report = dict(base_head=setup['base_head'], source_manifest=f'{prefix}-sources.json', binary_sha256=sha, command=command, returncode=code, elapsed_seconds=time.monotonic()-start, limit_seconds=20, diagnostic_only=True, all_owned_processes_reaped=True, all_sources_unchanged=True)
(A/f'{prefix}-terminal.json').write_text(json.dumps(report, indent=2)+'\n')
print('Diagnostic terminal:', code, flush=True)
print((A/f'{prefix}-case.log').read_text()[-12000:], flush=True)
