from pathlib import Path
import hashlib, json, os, subprocess, time

audit = Path(__file__).resolve().parent
root = Path('/tmp/hypercurve-mapped-cut-scalars-2026-09-23')
repo = root / 'hypercurve'
prefix = 'mapped-cut-scalars-20260923'
bindings = json.loads((audit / f'{prefix}-sources.json').read_text())
env = dict(os.environ, **json.loads((audit / 'opposed-endpoint-contact-full1-build-settings.json').read_text()))
cargo = '/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/cargo'

def verify():
    for name, sha in bindings.items():
        assert hashlib.sha256((root / name).read_bytes()).hexdigest() == sha, name

verify()
rows = []
for feature in ['--all-features', '--no-default-features']:
    command = [cargo, 'check', '--all-targets', feature, '--locked', '--offline']
    log = audit / f'{prefix}-check-{len(rows)}.log'
    start = time.monotonic()
    with log.open('w') as output:
        code = subprocess.run(command, cwd=repo, env=env, stdout=output, stderr=subprocess.STDOUT, timeout=1200).returncode
    verify()
    assert code == 0 and 'warning:' not in log.read_text(), log
    row = dict(command=command, returncode=code, elapsed_seconds=time.monotonic()-start, log=log.name)
    rows.append(row)
    print(row, flush=True)
report = dict(checks=rows, all_sources_unchanged=True, all_processes_reaped=True)
(audit / f'{prefix}-terminal.json').write_text(json.dumps(report, indent=2)+'\n')
print('Scalar API checks complete; all children reaped.', flush=True)
