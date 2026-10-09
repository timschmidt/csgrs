import hashlib, json, os, subprocess, sys, tarfile, time
from pathlib import Path
os.environ.update(CARGO_HOME='/tmp/csgrs-properties-cargo', CARGO_NET_OFFLINE='true', CCACHE_DIR='/tmp/csgrs-properties-ccache')
out = Path(sys.argv[1]).resolve()
out.mkdir(parents=True, exist_ok=False)
paths = sorted([p for name in ['hyperreal', 'hyperlimit', 'hyperlattice', 'hypersolve', 'hypercurve', 'hyperdrc', 'hypermesh', 'hyperpath', 'hyperphysics', 'hypertri'] for p in Path('crates', name).rglob('*') if p.is_file() and 'target' not in p.parts and (p.suffix == '.rs' or p.name in ['Cargo.toml', 'Cargo.lock'])] + list(Path('src').rglob('*.rs')) + [Path(p) for p in ['Cargo.toml', 'Cargo.lock', '.cargo/config.toml', '.rustfmt.toml', 'crates/.rustfmt.toml', 'crates/hyperdrc/docs/CPArti FPGA dev board.zip', 'crates/hyperdrc/docs/HVP109A.zip']])
def hashes():
    return {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
initial = hashes()
with tarfile.open(out / 'sources.tar.gz', 'w:gz') as archive:
    for p in paths:
        archive.add(p, arcname=str(p))
record = dict(scope='Early point-equality correctness gate on the provisional native-source candidate; no full downstream matrix, full-board runtime or isolated production admission is implied.', environment={key: os.environ[key] for key in ['CARGO_HOME', 'CARGO_NET_OFFLINE', 'CCACHE_DIR']}, base_revision=subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(), source_hashes=initial, unchanged=True)
(out / 'sources.json').write_text(json.dumps(record, indent=2) + '\n')
(out / 'reproduce.py').write_text(Path(__file__).read_text())
commands = [('format', ['cargo', 'fmt', '--all', '--', '--check']), ('point-tests', ['cargo', 'test', '--locked', '-p', 'hypercurve', '--release', '--lib', 'point::', '--', '--test-threads=2']), ('public-curve-point', ['cargo', 'test', '--locked', '-p', 'hypercurve', '--release', '--test', 'hypercurve_curve_point', '--', '--test-threads=2']), ('curve-clippy', ['cargo', 'clippy', '--locked', '-p', 'hypercurve', '--release', '--all-targets', '--', '-D', 'warnings']), ('build-drc-library', ['cargo', 'build', '--locked', '-p', 'hyperdrc', '--release', '--lib', '--message-format=json'])]
status = []
def run(name, command):
    assert hashes() == initial
    started = time.monotonic()
    with (out / (name + '.log')).open('w') as log:
        code = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT).returncode
    unchanged = hashes() == initial
    status.append(dict(name=name, command=command, exit_code=code, elapsed_seconds=time.monotonic()-started))
    (out / 'status.json').write_text(json.dumps(status, indent=2) + '\n')
    record['unchanged'] = unchanged
    (out / 'sources.json').write_text(json.dumps(record, indent=2) + '\n')
    assert unchanged
    print(name, code, flush=True)
    if code:
        raise SystemExit(code)
for name, command in commands:
    run(name, command)
print('point-equality early cost gate ready; downstream admission not run', flush=True)
