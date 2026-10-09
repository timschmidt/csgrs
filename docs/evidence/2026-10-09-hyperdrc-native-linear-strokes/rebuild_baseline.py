"""Extract and rebuild a complete baseline dependency graph without shared targets.

Usage: python3 rebuild_baseline.py NEW_DIRECTORY [--extract-only]
After extraction, use --build-only to build the existing frozen source tree.
Build only after all candidate tests and performance measurements have exited.
"""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tarfile

revision = '4d1e32fe69c0eb5b71b9949f4b3129e961007d20'
out = Path(sys.argv[1]).resolve()
build_only = sys.argv[2:] == ['--build-only']
source = out / 'source'
record = out / 'source.json'
info = lambda p: dict(bytes=p.stat().st_size, sha256=hashlib.sha256(p.read_bytes()).hexdigest())
if not build_only:
    source.mkdir(parents=True, exist_ok=False)
    paths = subprocess.run(['git', 'ls-tree', '-r', '--name-only', revision],
                           capture_output=True, text=True, check=True).stdout.splitlines()
    selected = []
    for path in paths:
        p = Path(path)
        if p.parts[0] in ['src', '.cargo'] or (
            p.parts[0] in ['crates', 'examples', 'tests', 'benchmarks'] and p.suffix == '.rs'
        ) or (
            p.parts[0] == 'crates' and ('src' in p.parts or p.suffix == '.md'
                                     or p.name in ['Cargo.toml', 'Cargo.lock', 'build.rs'])
        ) or path in ['Cargo.toml', 'Cargo.lock', 'readme.md', 'LICENSE', '.rustfmt.toml']:
            selected.append(path)
    # Batch paths to stay below the host's command argument limit.
    for offset in range(0, len(selected), 500):
        with subprocess.Popen(['git', 'archive', revision, *selected[offset:offset + 500]],
                              stdout=subprocess.PIPE) as proc:
            with tarfile.open(fileobj=proc.stdout, mode='r|') as archive:
                archive.extractall(source, filter='data')
            assert proc.wait() == 0
    hashes = {p: info(source / p)['sha256'] for p in selected}
    record.write_text(json.dumps(dict(revision=revision, source_hashes=hashes), indent=2) + '\n')
    print('extracted', len(selected), 'files from', revision, flush=True)
else:
    hashes = json.loads(record.read_text())['source_hashes']
assert all(info(source / p)['sha256'] == h for p, h in hashes.items())
if sys.argv[2:] == ['--extract-only']:
    raise SystemExit(0)
target = out / 'target'
environment = {'CARGO_TARGET_DIR': str(target), 'CCACHE_DIR': str(out / 'ccache')}
command = ['cargo', 'build', '--locked', '--release', '-p', 'hyperdrc', '--lib', '--message-format=json']
with (out / 'build.jsonl').open('w') as stdout, (out / 'build.log').open('w') as stderr:
    code = subprocess.run(command, cwd=source, env={**os.environ, **environment},
                          stdout=stdout, stderr=stderr).returncode
(out / 'build-status.json').write_text(json.dumps(dict(command=command, exit_code=code,
    revision=revision, cwd=str(source), environment=environment), indent=2) + '\n')
assert code == 0
libraries = {}
for line in (out / 'build.jsonl').read_text().splitlines():
    artifact = json.loads(line)
    if artifact.get('reason') != 'compiler-artifact':
        continue
    for filename in artifact['filenames']:
        if filename.endswith('.rlib'):
            p = Path(filename)
            libraries[artifact['target']['name']] = dict(path=str(p), **info(p))
(out / 'manifest.json').write_text(json.dumps(libraries, indent=2) + '\n')
assert all(info(source / p)['sha256'] == h for p, h in hashes.items())
print('built', len(libraries), 'libraries', flush=True)
