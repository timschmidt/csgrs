from pathlib import Path
import hashlib, json, os, shutil, subprocess, sys, time

audit = Path(__file__).resolve().parent
root = Path('/tmp/hypercurve-fillet-companion-public2-2026-09-23')
repo = root / 'hypercurve'
prefix = 'fillet-companion-chart-20260923-public2'
name = 'fillet_companion_public_20260923'
env = dict(os.environ, **json.loads((audit / 'opposed-endpoint-contact-full1-build-settings.json').read_text()))
bindings = {str(path.relative_to(root)): hashlib.sha256(path.read_bytes()).hexdigest() for directory in ['hypercurve', 'hypersolve'] for path in (root / directory).rglob('*') if path.is_file()}
dependencies = json.loads((audit / 'boundary-api-20260923-check5-sources.json').read_text())['isolated']
bindings.update({key: value for key, value in dependencies.items() if not key.startswith(('hypercurve/', 'hypersolve/'))})
def verify():
    for key, sha in bindings.items():
        assert hashlib.sha256((root / key).read_bytes()).hexdigest() == sha, key
verify()
(audit / f'{prefix}-sources.json').write_text(json.dumps(bindings, indent=2)+'\n')
command = ['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo', 'build', '--release', '--all-features', '--example', name, '--message-format=json', '--locked', '--offline']
with (audit / f'{prefix}-build.jsonl').open('w') as out, (audit / f'{prefix}-build.log').open('w') as err:
    code = subprocess.run(command, cwd=repo, env=env, stdout=out, stderr=err, timeout=1200).returncode
verify()
assert code == 0
binary = audit / f'{prefix}-probe'
for line in (audit / f'{prefix}-build.jsonl').read_text().splitlines():
    row = json.loads(line)
    if row.get('reason') == 'compiler-artifact' and row.get('executable') and row['target']['name'] == name:
        assert not row['fresh']
        shutil.copy2(row['executable'], binary)
print('Public probe built', hashlib.sha256(binary.read_bytes()).hexdigest(), flush=True)
rows = []
for index in range(4):
    log = audit / f'{prefix}-case-{index}.log'
    start = time.monotonic()
    with log.open('w') as out:
        try:
            code = subprocess.run([str(binary), str(index)], cwd=repo, env=env, stdout=out, stderr=subprocess.STDOUT, timeout=180).returncode
        except subprocess.TimeoutExpired:
            code = 'timeout'
    verify()
    row = dict(case=index, returncode=code, elapsed_seconds=time.monotonic()-start, log=log.name)
    rows.append(row)
    (audit / f'{prefix}-results.json').write_text(json.dumps(rows, indent=2)+'\n')
    print(row, log.read_text()[-1800:], flush=True)
report = dict(cases=rows, binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(), all_sources_unchanged=True, all_processes_reaped=True)
(audit / f'{prefix}-terminal.json').write_text(json.dumps(report, indent=2)+'\n')
print('Terminal; public probe cases reaped.', flush=True)
