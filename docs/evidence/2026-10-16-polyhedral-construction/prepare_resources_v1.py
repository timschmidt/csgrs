"""Pin matched libraries and retain every completed construction cost trial."""
import hashlib
import json
from pathlib import Path
import subprocess

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
TARGET = Path('/dev/shm/csgrs-polyhedral-construction/resources-v1')
BUILDS = {
    'baseline': ROOT / 'target/properties/distribution-contracts/cost-build-v2/release/deps',
    'candidate': ROOT / 'target/properties/polyhedral-construction/cost-build-v1/release/deps',
}
assert not TARGET.exists()

def sha(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()

def unit(build, name, fingerprint=None):
    units = []
    for file in (build.parent / '.fingerprint').glob(name + '-*/lib-' + name + '.json'):
        if fingerprint is not None:
            encoded = file.with_suffix('').read_text().strip()
            if int.from_bytes(bytes.fromhex(encoded), 'little') != fingerprint:
                continue
        elif name == 'csgrs' and file.parent.name != 'csgrs-5357807c720d8967':
            continue
        units.append((file.parent.name, json.loads(file.read_text())))
    assert len(units) == 1, (name, units)
    return units[0]

pins = {}
libraries = {}
for variant, build in BUILDS.items():
    root_name, root = unit(build, 'csgrs')
    pins[variant] = {'csgrs': root}
    libraries[variant] = {'csgrs': 'libcsgrs.rlib'}
    for name in ['hypermesh', 'hypertri']:
        expected = [dep[3] for dep in root['deps'] if dep[1] == name]
        assert len(expected) == 1
        native_name, native = unit(build, name, expected[0])
        pins[variant][name] = native
        libraries[variant][name] = 'lib' + native_name + '.rlib'
for name in ['csgrs', 'hypermesh', 'hypertri']:
    for key in ['features', 'declared_features', 'profile', 'rustflags', 'rustc', 'target']:
        assert pins['baseline'][name][key] == pins['candidate'][name][key], (name, key)

preserved = {}
commands = [['mkdir', ['mkdir', '-p', str(TARGET)]]]
for variant, build in BUILDS.items():
    for path in build.iterdir():
        if path.is_file() and path.suffix in {'.rlib', '.rmeta', '.so'}:
            preserved[str(path)] = sha(path)
    for kind in ['timing', 'allocation']:
        command = ['rustc', '--edition=2024', '-Copt-level=2', '-D', 'warnings',
                   '--check-cfg', 'cfg(allocation)', str(E / 'resource_probe_v1.rs'),
                   '--extern', f'csgrs={build}/libcsgrs.rlib',
                   '--extern', f'hypermesh={build}/{libraries[variant]["hypermesh"]}',
                   '-L', f'dependency={build}', '-o', str(TARGET / f'{variant}-{kind}')]
        if kind == 'allocation': command += ['--cfg', 'allocation']
        commands.append([f'build-{variant}-{kind}', command])
commands += [['measure', ['python3', str(E / 'measure_resources_v1.py'), str(TARGET)]]]
candidate = BUILDS['candidate']
commands += [
    ['replay-build', ['rustc', '--edition=2024', '-Copt-level=2', '-D', 'warnings',
                     str(E / 'baseline_probe_v1.rs'), '--extern', f'csgrs={candidate}/libcsgrs.rlib',
                     '--extern', f'hypermesh={candidate}/{libraries["candidate"]["hypermesh"]}',
                     '-L', f'dependency={candidate}', '-o', str(TARGET / 'replay')]],
    ['replay', [str(TARGET / 'replay'), str(TARGET / 'fixtures')]],
]
for suffix, value in [('.json', commands), ('-target.json', str(TARGET)),
                      ('-seed.json', None), ('-preserve.json', preserved)]:
    path = E / ('resources-v1' + suffix)
    assert not path.exists()
    path.write_text(json.dumps(value, indent=2) + '\n')

changed = ['src/solid.rs', 'crates/hypertri/src/earcut.rs', 'crates/hypertri/src/lib.rs',
           'crates/hypertri/tests/simple_ring.rs', 'tests/polyhedral_construction.rs',
           'crates/hypertri/README.md', 'readme.md', 'tests/ffi/polyhedral_construction.c',
           'tests/ffi/polyhedral_construction.cpp', 'tests/ffi/polyhedral_construction.py',
           'bindings/go/polyhedral_construction_test.go']
sizes = {}
for name in changed:
    result = subprocess.run(['git', 'show', 'HEAD:' + name], cwd=ROOT, capture_output=True)
    old = result.stdout if result.returncode == 0 else b''
    new = (ROOT / name).read_bytes()
    sizes[name] = {'baseline_exists': result.returncode == 0,
                  'baseline_bytes': len(old), 'candidate_bytes': len(new),
                  'baseline_lines': len(old.splitlines()), 'candidate_lines': len(new.splitlines())}
host = {
    'builds': {name: str(path) for name, path in BUILDS.items()},
    'fingerprints': pins,
    'source_sizes': sizes,
    'baseline_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
    'cargo_lock_sha256': sha(ROOT / 'Cargo.lock'),
    'rustc': subprocess.check_output(['rustc', '-vV'], text=True),
    'hardware': subprocess.check_output(['lscpu'], text=True),
    'system': subprocess.check_output(['uname', '-a'], text=True),
    'library_bytes': {variant: {name: (build / filename).stat().st_size
                               for name, filename in libraries[variant].items()}
                      for variant, build in BUILDS.items()},
    'scope': 'Same original completed construction and exact output rows. Correctness repair with additional validation; no optimization claim. See cost-contract-v1.md and cost-clarification-v1.md.',
}
(E / 'cost-host-v1.json').write_text(json.dumps(host, indent=2) + '\n')
print(json.dumps({'commands': len(commands), 'pinned_libraries': len(preserved)}))
