"""Pin matched immutable libraries after complete batch qualification."""
import gzip
import hashlib
import json
from pathlib import Path
import subprocess

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
TARGET = Path('/dev/shm/csgrs-batch-winding-contracts/resources-v1')
BUILDS = {
    'baseline': ROOT / 'target/properties/minkowski-contracts/benchmark-repair-v1/release/deps',
    'candidate': ROOT / 'target/properties/batch-winding-contracts/qualification-v1/release/deps',
}


def sha(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def unit(build, name, fingerprint=None):
    matches = []
    for file in (build.parent / '.fingerprint').glob(name + '-*/lib-' + name + '.json'):
        if fingerprint is None:
            if file.parent.name != 'csgrs-5357807c720d8967':
                continue
        elif int.from_bytes(bytes.fromhex(file.with_suffix('').read_text().strip()), 'little') != fingerprint:
            continue
        matches.append((file.parent.name, json.loads(file.read_text())))
    assert len(matches) == 1, (name, matches)
    return matches[0]


assert not TARGET.exists()
qualification = json.loads((E / 'qualification-v1/result.json').read_text())
assert len(qualification['commands']) == 50 and not qualification['passed']
assert [row['name'] for row in qualification['commands'] if row['exit_code']] == ['hypertri-fuzz-clippy']
repair = json.loads((E / 'fuzz-lint-repair-v1/result.json').read_text())
assert repair['passed'] and len(repair['commands']) == 5
assert qualification['commands'][-1]['name'] == 'matched-benchmark-library'
with gzip.open(E / 'qualification-v1/source-hashes.json.gz', 'rt') as stream:
    qualified = json.load(stream)
with gzip.open(E / 'fuzz-lint-repair-v1/source-hashes.json.gz', 'rt') as stream:
    current = json.load(stream)
adjustment = json.loads((E / 'fuzz-lint-adjustment-v1.json').read_text())['sources']
assert {path for path in set(qualified) | set(current) if qualified.get(path) != current.get(path)} == set(adjustment)
assert all(row == {'before_sha256': qualified[path], 'after_sha256': current[path]}
           for path, row in adjustment.items())
assert all(sha(ROOT / path) == digest for path, digest in current.items())
baseline_protocol = json.loads((E / 'baseline-probes-v1/protocol.json').read_text())
assert baseline_protocol['rustc'] == subprocess.check_output(['rustc', '-vV'], text=True)
pins, libraries, preserved = {}, {}, {}
for variant, build in BUILDS.items():
    _, root = unit(build, 'csgrs')
    pins[variant] = {'csgrs': root}
    libraries[variant] = {'csgrs': 'libcsgrs.rlib'}
    for name in ['hypermesh', 'hypertri', 'hyperreal', 'hyperlimit']:
        expected = [dep[3] for dep in root['deps'] if dep[1] == name]
        if not expected:
            expected = [dep[3] for dep in pins[variant]['hypermesh']['deps'] if dep[1] == name]
        assert len(expected) == 1, (variant, name, expected)
        native_name, native = unit(build, name, expected[0])
        pins[variant][name] = native
        libraries[variant][name] = 'lib' + native_name + '.rlib'
    for path in build.iterdir():
        if path.is_file() and path.suffix in {'.rlib', '.rmeta', '.so'}:
            preserved[str(path)] = sha(path)
for name in pins['baseline']:
    for key in ['features', 'declared_features', 'profile', 'rustflags', 'rustc', 'target']:
        assert pins['baseline'][name][key] == pins['candidate'][name][key], (name, key)
initial_pins = json.loads((E / 'baseline-probes-v1-preserve.json').read_text())
assert all(preserved.get(path, digest) == digest and sha(Path(path)) == digest
           for path, digest in initial_pins.items())

driver = str((E / 'resource_probe_v1.rs').relative_to(ROOT))
with gzip.open(E / 'baseline-probes-v1/source-hashes.json.gz', 'rt') as stream:
    baseline_sources = json.load(stream)
assert sha(ROOT / driver) == baseline_sources[driver] == qualified[driver]
commands = [['mkdir', ['mkdir', '-p', str(TARGET)]]]
for variant, build in BUILDS.items():
    for kind in ['timing', 'allocation']:
        name = f'{variant}-{kind}'
        command = ['rustc', '--edition=2024', '-Copt-level=2', '-D', 'warnings',
                   '--check-cfg', 'cfg(allocation)', driver,
                   '--extern', f'csgrs={build}/libcsgrs.rlib',
                   '--extern', f'hypermesh={build}/{libraries[variant]["hypermesh"]}',
                   '-L', f'dependency={build}', '-o', str(TARGET / name)]
        if kind == 'allocation':
            command += ['--cfg', 'allocation']
        commands.append(['build-' + name, command])
commands.append(['measure', ['python3', str(E / 'measure_resources_v1.py'), str(TARGET)]])

previous = json.loads((E.parent / '2026-10-16-minkowski-contracts/checkpoint-v1.json').read_text())
pending = previous['pending_native_sha256']
assert len(pending) == 35 and all(sha(ROOT / path) == digest for path, digest in pending.items())
names = subprocess.check_output(['git', 'diff', '--name-only', '-z', 'HEAD',
                                '--', 'src', 'tests', 'crates', 'readme.md'], cwd=ROOT).decode().split('\0')
names += subprocess.check_output(['git', 'ls-files', '-z', '--others', '--exclude-standard',
                                 '--', 'src', 'tests', 'crates', 'readme.md'], cwd=ROOT).decode().split('\0')
sizes = {}
for name in sorted(set(names) - set(pending) - {''}):
    old = subprocess.run(['git', 'show', 'HEAD:' + name], cwd=ROOT, capture_output=True)
    new = (ROOT / name).read_bytes()
    if old.returncode == 0 and old.stdout == new:
        continue
    sizes[name] = {'baseline_exists': old.returncode == 0,
                  'baseline_bytes': len(old.stdout), 'candidate_bytes': len(new),
                  'baseline_lines': len(old.stdout.splitlines()), 'candidate_lines': len(new.splitlines())}
assert len(sizes) == 8, sizes
host = {
    'builds': {name: str(path) for name, path in BUILDS.items()},
    'fingerprints': pins, 'libraries': libraries, 'source_sizes': sizes,
    'baseline_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
    'cargo_lock_sha256': sha(ROOT / 'Cargo.lock'), 'driver_sha256': sha(ROOT / driver),
    'rustc': subprocess.check_output(['rustc', '-vV'], text=True),
    'hardware': subprocess.check_output(['lscpu'], text=True),
    'system': subprocess.check_output(['uname', '-a'], text=True),
    'library_bytes': {variant: {name: (build / filename).stat().st_size
                               for name, filename in libraries[variant].items()}
                      for variant, build in BUILDS.items()},
    'scope': 'Unchanged pre-edit probe; three completed controls and two candidate-only batch completions. Original 24 children plus 64 replay/comparison children. No optimization or universal cost claim. See cost-contract-v1.md and resource-replay-contract-v1.md.',
}
for suffix, value in [('.json', commands), ('-target.json', str(TARGET)),
                      ('-seed.json', None), ('-preserve.json', preserved)]:
    path = E / ('resources-v1' + suffix)
    assert not path.exists()
    path.write_text(json.dumps(value, indent=2) + '\n')
assert not (E / 'cost-host-v1.json').exists()
(E / 'cost-host-v1.json').write_text(json.dumps(host, indent=2) + '\n')
print(json.dumps({'commands': len(commands), 'pinned_libraries': len(preserved),
                  'source_files': len(sizes)}))
