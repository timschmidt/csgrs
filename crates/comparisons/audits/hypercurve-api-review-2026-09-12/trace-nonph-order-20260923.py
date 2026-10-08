from pathlib import Path
import hashlib, json, os, shutil, subprocess, time

audit = Path(__file__).resolve().parent
root = Path('/tmp/hypercurve-parameter-order-2026-09-23')
baseline = Path('/tmp/hypercurve-boundary-api-2026-09-23')
root.mkdir(exist_ok=True)
for repo in baseline.iterdir():
    target = root / repo.name
    if repo.name != 'hypercurve' and repo.is_dir() and not target.exists():
        target.symlink_to(repo, target_is_directory=True)
repo = root / 'hypercurve'
assert not repo.exists()
shutil.copytree(baseline / 'hypercurve', repo)
offset = repo / 'src/bezier_offset.rs'
source = offset.read_text()
needle = '        if other_algebraic == &self.data.authority.data.retained_parameter {'
assert source.count(needle) == 1
trace = '''        eprintln!("ORDER before equality: selected=[{:?},{:?}] other=[{:?},{:?}] retained-degree={} other-degree={} same-retained={}",
            self.root().lower.to_f64_lossy(), self.root().upper.to_f64_lossy(),
            other_algebraic.interval().start().to_f64_lossy(), other_algebraic.interval().end().to_f64_lossy(),
            self.data.authority.data.retained_parameter.polynomial().degree(), other_algebraic.polynomial().degree(),
            other_algebraic == &self.data.authority.data.retained_parameter);
'''
offset.write_text(source.replace(needle, trace + needle))
name = 'nonph_parameter_order_probe_20260923'
(repo / 'examples' / (name + '.rs')).write_bytes((audit / 'boundary-api-20260923-nonph-probe.rs').read_bytes())
bindings = {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in root.rglob('*') if p.is_file() and 'target' not in p.parts}
# Dependency symlinks point only to the previously qualified committed snapshots.
dependencies = json.loads((audit / 'boundary-api-20260923-check5-sources.json').read_text())['isolated']
def verify():
    for name, sha in bindings.items():
        assert hashlib.sha256((root / name).read_bytes()).hexdigest() == sha, name
    for name, sha in dependencies.items():
        if not name.startswith('hypercurve/'):
            assert hashlib.sha256((baseline / name).read_bytes()).hexdigest() == sha, name
verify()
(audit / 'nonph-order-trace-20260923-sources.json').write_text(json.dumps(bindings, indent=2) + '\n')
env = dict(os.environ, **json.loads((audit / 'opposed-endpoint-contact-full1-build-settings.json').read_text()))
cmd = ['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo', 'build', '--release', '--all-features', '--example', name, '--message-format=json', '--locked', '--offline']
start = time.monotonic()
with (audit / 'nonph-order-trace-20260923-build.jsonl').open('w') as out, (audit / 'nonph-order-trace-20260923-build.log').open('w') as err:
    code = subprocess.run(cmd, cwd=repo, env=env, stdout=out, stderr=err, timeout=900).returncode
verify()
assert code == 0
print('build completed', time.monotonic() - start, flush=True)
binary = audit / 'nonph-order-trace-20260923-probe'
for line in (audit / 'nonph-order-trace-20260923-build.jsonl').read_text().splitlines():
    row = json.loads(line)
    if row.get('reason') == 'compiler-artifact' and row.get('executable') and row['target']['name'] == name:
        assert not row['fresh']
        shutil.copy2(row['executable'], binary)
start = time.monotonic()
with (audit / 'nonph-order-trace-20260923-run.log').open('w') as out:
    try:
        code = subprocess.run([str(binary)], cwd=repo, env=env, stdout=out, stderr=subprocess.STDOUT, timeout=75).returncode
    except subprocess.TimeoutExpired:
        code = 'timeout'
verify()
result = dict(returncode=code, elapsed_seconds=time.monotonic()-start,
              binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(), sources_unchanged=True)
(audit / 'nonph-order-trace-20260923-result.json').write_text(json.dumps(result, indent=2) + '\n')
print(result, flush=True)
print((audit / 'nonph-order-trace-20260923-run.log').read_text()[-12000:], flush=True)
print('Terminal; build and probe reaped.', flush=True)
