from pathlib import Path
import hashlib, json, subprocess, sys

A = Path(__file__).resolve().parent
W = A.parent
version = sys.argv[1]
root = Path(f'/tmp/hypercurve-rational-polynomial-{version}-20260924')
prefix = f'rational-polynomial-20260924-{version}'
manifest = json.loads((A / f'{prefix}-sources.json').read_text())
source = A / 'rational-polynomial-probe-demand-20260924.rs'
source_sha = hashlib.sha256(source.read_bytes()).hexdigest()
def verify():
    for name, sha in manifest.items():
        assert hashlib.sha256((W / name).read_bytes()).hexdigest() == sha, name
        assert hashlib.sha256((root / name).read_bytes()).hexdigest() == sha, name
    assert hashlib.sha256(source.read_bytes()).hexdigest() == source_sha
verify()
rows = [json.loads(line) for line in (A / f'{prefix}-probe-build.jsonl').read_text().splitlines()]
binary = A / f'{prefix}-demand-probe'
settings = json.loads((A / 'opposed-endpoint-contact-full1-build-settings.json').read_text())
command = ['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustc', '--edition', '2024', '-O', str(source), '-o', str(binary), '-L', 'dependency=' + settings['CARGO_TARGET_DIR'] + '/release/deps']
libraries = {}
for name in ['hyperreal', 'num']:
    row = next(row for row in rows if row.get('reason') == 'compiler-artifact' and row['target']['name'] == name and not row.get('executable'))
    path = next(path for path in row['filenames'] if path.endswith('.rlib'))
    libraries[name] = dict(path=path, sha256=hashlib.sha256(Path(path).read_bytes()).hexdigest())
    command += ['--extern', name + '=' + path]
subprocess.run(command, check=True, timeout=60)
results = []
for name in subprocess.check_output([str(binary), 'list'], text=True).splitlines():
    output = subprocess.check_output([str(binary), name], text=True, timeout=60)
    for line in output.splitlines():
        row = json.loads(line)
        results.append(row)
        print(row, flush=True)
verify()
(A / f'{prefix}-demand-terminal.json').write_text(json.dumps(dict(measurements=results, libraries=libraries, source_sha256=source_sha, binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(), all_processes_reaped=True, all_sources_unchanged=True), indent=2) + '\n')
