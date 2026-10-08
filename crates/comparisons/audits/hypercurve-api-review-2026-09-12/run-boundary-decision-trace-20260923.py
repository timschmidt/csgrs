from pathlib import Path
import hashlib, json, os, shutil, subprocess, sys, time

audit = Path(__file__).resolve().parent
root = Path('/tmp/hypercurve-boundary-decision-trace-2026-09-23')
repo = root / 'hypercurve'
prefix = 'boundary-decision-trace-20260923-' + sys.argv[1]
name = 'nonph_contact_isolation_probe_20260923'
bindings = {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in root.rglob('*') if p.is_file() and 'target' not in p.parts and 'dumps' not in p.parts}
dependencies = json.loads((audit / 'boundary-api-20260923-check5-sources.json').read_text())['isolated']
def verify():
    for name, sha in bindings.items():
        assert hashlib.sha256((root / name).read_bytes()).hexdigest() == sha, name
    for name, sha in dependencies.items():
        if not name.startswith(('hypercurve/', 'hypersolve/')):
            assert hashlib.sha256((root / name).read_bytes()).hexdigest() == sha, name
verify()
(audit / (prefix + '-sources.json')).write_text(json.dumps(bindings, indent=2) + '\n')
env = dict(os.environ, **json.loads((audit / 'opposed-endpoint-contact-full1-build-settings.json').read_text()))
cmd = ['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo', 'build', '--release', '--all-features', '--example', name, '--message-format=json', '--locked', '--offline']
start = time.monotonic()
with (audit / (prefix + '-build.jsonl')).open('w') as out, (audit / (prefix + '-build.log')).open('w') as err:
    code = subprocess.run(cmd, cwd=repo, env=env, stdout=out, stderr=err, timeout=900).returncode
verify()
assert code == 0
print('build completed', time.monotonic() - start, flush=True)
binary = audit / (prefix + '-probe')
for line in (audit / (prefix + '-build.jsonl')).read_text().splitlines():
    row = json.loads(line)
    if row.get('reason') == 'compiler-artifact' and row.get('executable') and row['target']['name'] == name:
        assert not row['fresh']
        shutil.copy2(row['executable'], binary)
start = time.monotonic()
with (audit / (prefix + '-run.log')).open('w') as out:
    try:
        code = subprocess.run([str(binary)], cwd=repo, env=env, stdout=out, stderr=subprocess.STDOUT, timeout=120).returncode
    except subprocess.TimeoutExpired:
        code = 'timeout'
verify()
result = dict(returncode=code, elapsed_seconds=time.monotonic()-start,
              binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(), sources_unchanged=True)
(audit / (prefix + '-result.json')).write_text(json.dumps(result, indent=2) + '\n')
print(result, flush=True)
print((audit / (prefix + '-run.log')).read_text()[-12000:], flush=True)
print('Terminal; build and probe reaped.', flush=True)

shutil.copytree(root / "dumps", audit / (prefix + "-dumps"))
