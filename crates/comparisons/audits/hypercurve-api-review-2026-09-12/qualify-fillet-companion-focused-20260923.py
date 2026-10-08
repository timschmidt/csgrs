from pathlib import Path
import hashlib, json, os, shutil, subprocess, time

audit = Path(__file__).resolve().parent
prefix = 'fillet-companion-chart-20260923-focused1'
env = dict(os.environ, **json.loads((audit / 'opposed-endpoint-contact-full1-build-settings.json').read_text()))
cargo = '/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'
dependencies = json.loads((audit / 'boundary-api-20260923-check5-sources.json').read_text())['isolated']
names = (audit / 'fillet-companion-chart-20260923-new-tests.txt').read_text().splitlines()
results = []
for label, root in [('parent', Path('/tmp/hypercurve-fillet-companion-parent-2026-09-23')), ('candidate', Path('/tmp/hypercurve-fillet-companion-clean-2026-09-23'))]:
    repo = root / 'hypercurve'
    bindings = {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest() for name in ['hypercurve', 'hypersolve'] for p in (root / name).rglob('*') if p.is_file()}
    bindings.update({name: sha for name, sha in dependencies.items() if not name.startswith(('hypercurve/', 'hypersolve/'))})
    def verify():
        for name, sha in bindings.items():
            assert hashlib.sha256((root / name).read_bytes()).hexdigest() == sha, name
    verify()
    (audit / f'{prefix}-{label}-sources.json').write_text(json.dumps(bindings, indent=2) + '\n')
    command = [cargo, 'test', '--lib', '--release', '--all-features', '--no-run', '--message-format=json', '--locked', '--offline']
    start = time.monotonic()
    with (audit / f'{prefix}-{label}-build.jsonl').open('w') as out, (audit / f'{prefix}-{label}-build.log').open('w') as err:
        code = subprocess.run(command, cwd=repo, env=env, stdout=out, stderr=err, timeout=1200).returncode
    verify()
    assert code == 0, (audit / f'{prefix}-{label}-build.log').read_text()[-5000:]
    binary = audit / f'{prefix}-{label}-libtest'
    for line in (audit / f'{prefix}-{label}-build.jsonl').read_text().splitlines():
        row = json.loads(line)
        if row.get('reason') == 'compiler-artifact' and row.get('executable') and row['target']['name'] == 'hypercurve':
            assert not row['fresh']
            shutil.copy2(row['executable'], binary)
    listing = subprocess.check_output([str(binary), '--list'], text=True)
    assert all(name + ': test' in listing for name in names)
    print(label, 'fresh build', time.monotonic() - start, flush=True)
    rows = []
    for index, name in enumerate(names):
        log = audit / f'{prefix}-{label}-case-{index}.log'
        start = time.monotonic()
        with log.open('w') as out:
            try:
                code = subprocess.run([str(binary), '--exact', name, '--nocapture', '--test-threads=1'], cwd=repo, env=env, stdout=out, stderr=subprocess.STDOUT, timeout=75).returncode
            except subprocess.TimeoutExpired:
                code = 'timeout'
        row = dict(name=name, returncode=code, elapsed_seconds=time.monotonic()-start, log=log.name)
        rows.append(row)
        print(label, row, log.read_text()[-1800:], flush=True)
    verify()
    result = dict(label=label, cases=rows, binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(), sources_unchanged=True, all_processes_reaped=True)
    results.append(result)
    (audit / f'{prefix}-results.json').write_text(json.dumps(results, indent=2) + '\n')
print('Terminal; all focused builds and cases reaped.', flush=True)
