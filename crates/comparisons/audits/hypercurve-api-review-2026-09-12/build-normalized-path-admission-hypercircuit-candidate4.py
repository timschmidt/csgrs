from pathlib import Path
import hashlib, io, json, os, shutil, subprocess, tarfile, time, tomllib

workspace = Path('/home/tim/Documents/GitHub/workspace')
root = Path('/tmp/hypercurve-region-admission-qualification')
audit = workspace / 'hypercurve-api-review-2026-09-12'
prefix = 'normalized-path-admission-hypercircuit-candidate4'
records = []
pending = [root / 'hypercircuit']
seen = set()
while pending:
    crate = pending.pop()
    if crate in seen:
        continue
    seen.add(crate)
    repo = crate.relative_to(root).parts[0]
    checkout = root / repo
    if not checkout.exists():
        source = workspace / repo
        head = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=source, text=True).strip()
        checkout.mkdir()
        archive = subprocess.check_output(['git', 'archive', head], cwd=source)
        with tarfile.open(fileobj=io.BytesIO(archive)) as tar:
            tar.extractall(checkout, filter='data')
        records.append(dict(repository=repo, commit=head, archive_sha256=hashlib.sha256(archive).hexdigest()))
    manifest = tomllib.loads((crate / 'Cargo.toml').read_text())
    def dependencies(value):
        if isinstance(value, dict):
            for key, child in value.items():
                if key == 'path' and isinstance(child, str) and (crate / child / 'Cargo.toml').is_file():
                    pending.append(Path(os.path.normpath(crate / child)))
                elif key == 'path' and isinstance(child, str) and child.startswith('../'):
                    pending.append(Path(os.path.normpath(crate / child)))
                elif isinstance(child, (dict, list)):
                    dependencies(child)
        elif isinstance(value, list):
            for child in value:
                dependencies(child)
    # Only dependency tables contain crate paths; targets also use a `path` key.
    for key in ['dependencies', 'dev-dependencies', 'build-dependencies', 'patch', 'target', 'workspace']:
        dependencies(manifest.get(key, {}))

shutil.copy2(workspace / 'hypercircuit/src/materialize.rs', root / 'hypercircuit/src/materialize.rs')
shutil.copy2(workspace / 'hypercircuit/src/layout.rs', root / 'hypercircuit/src/layout.rs')
shutil.copy2(workspace / 'csgrs/src/curve/native.rs', root / 'csgrs/src/curve/native.rs')
(audit / (prefix + '-csgrs-source.patch')).write_bytes(subprocess.check_output(['git', 'diff', '--', 'src/curve/native.rs'], cwd=workspace / 'csgrs'))
(audit / (prefix + '-archives.json')).write_text(json.dumps(records, indent=2) + '\n')
(audit / (prefix + '-source.patch')).write_bytes(subprocess.check_output(['git', 'diff', '--', 'src/materialize.rs', 'src/layout.rs'], cwd=workspace / 'hypercircuit'))
manifest = []
for checkout in sorted(root.iterdir()):
    if checkout.is_dir():
        for path in sorted(checkout.rglob('*')):
            if path.is_file() and 'target' not in path.parts:
                manifest.append(dict(file=str(path.relative_to(root)), sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
(audit / (prefix + '-sources.json')).write_text(json.dumps(manifest, indent=2) + '\n')
toolchain = Path('/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin')
settings = json.loads((audit / 'normalized-path-admission-full2-build-settings.json').read_text())
env = dict(os.environ, **settings)
builds = []
for kind, args in [
    ('check', ['check', '--all-targets', '--no-default-features', '--features', 'geometry,interchange', '--locked', '--offline']),
    ('test-build', ['test', '--release', '--no-default-features', '--features', 'geometry,interchange', '--locked', '--offline', '--lib', '--test', 'materialize', '--test', 'fabrication', '--test', 'curved_outline', '--test', 'preview', '--test', 'placement', '--no-run', '--message-format=json']),
]:
    command = [str(toolchain / 'cargo'), *args]
    stem = prefix + '-' + kind
    start = time.monotonic()
    with (audit / (stem + '.log')).open('w') as err, (audit / (stem + '.jsonl')).open('w') as out:
        result = subprocess.run(command, cwd=root / 'hypercircuit', env=env, stdout=out if kind == 'test-build' else err, stderr=err, timeout=1200)
    record = dict(kind=kind, command=command, returncode=result.returncode, elapsed_seconds=time.monotonic() - start, log=stem + '.log')
    builds.append(record)
    (audit / (stem + '.exit')).write_text(str(result.returncode) + '\n')
    (audit / (prefix + '-builds.json')).write_text(json.dumps(builds, indent=2) + '\n')
    print(stem, result.returncode, round(record['elapsed_seconds'], 2), flush=True)
    assert result.returncode == 0, (audit / (stem + '.log')).read_text()[-8000:]
for row in manifest:
    assert hashlib.sha256((root / row['file']).read_bytes()).hexdigest() == row['sha256'], row['file']
assert (workspace / 'hypercircuit/src/materialize.rs').read_bytes() == (root / 'hypercircuit/src/materialize.rs').read_bytes()
assert (workspace / 'hypercircuit/src/layout.rs').read_bytes() == (root / 'hypercircuit/src/layout.rs').read_bytes()
assert (workspace / 'csgrs/src/curve/native.rs').read_bytes() == (root / 'csgrs/src/curve/native.rs').read_bytes()
print('builds complete; isolated sources unchanged', flush=True)
