"""Pin matched old/final libraries without mutating either retained build."""
import hashlib
import json
from pathlib import Path
import subprocess

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
TARGET = Path('/dev/shm/csgrs-minkowski-contracts/resources-v1')
BUILDS = {
    'baseline': ROOT / 'target/properties/polyhedral-construction/cost-build-v1/release/deps',
    'candidate': ROOT / 'target/properties/minkowski-contracts/benchmark-repair-v1/release/deps',
}

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
        elif file.parent.name != 'csgrs-5357807c720d8967':
            continue
        units.append((file.parent.name, json.loads(file.read_text())))
    assert len(units) == 1, (name, units)
    return units[0]

assert not TARGET.exists()
qualification = json.loads((E / 'qualification-v1/result.json').read_text())
assert len(qualification['commands']) == 48
assert [row['name'] for row in qualification['commands'] if row['exit_code']] == ['benchmark-consumers']
assert json.loads((E / 'benchmark-repair-v1/result.json').read_text())['passed']
pins, libraries, preserved = {}, {}, {}
for variant, build in BUILDS.items():
    _, root = unit(build, 'csgrs')
    pins[variant] = {'csgrs': root}
    libraries[variant] = {'csgrs': 'libcsgrs.rlib'}
    for name in ['hypermesh', 'hyperreal', 'hyperlimit']:
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

commands = [['mkdir', ['mkdir', '-p', str(TARGET)]]]
drivers = {'cube': 'resource_probe_v1.rs', 'scalar': 'scalar_resource_probe_v1.rs',
           'expanded': 'expanded_resource_probe_v1.rs'}
for variant, build in BUILDS.items():
    for group, driver in drivers.items():
        if group == 'expanded' and variant == 'baseline': continue
        for kind in ['timing', 'allocation']:
            name = f'{variant}-{group}-{kind}'
            command = ['rustc', '--edition=2024', '-Copt-level=2', '-D', 'warnings',
                       '--check-cfg', 'cfg(allocation)', str(E / driver),
                       '--extern', f'csgrs={build}/libcsgrs.rlib',
                       '--extern', f'hypermesh={build}/{libraries[variant]["hypermesh"]}',
                       '-L', f'dependency={build}', '-o', str(TARGET / name)]
            if kind == 'allocation': command += ['--cfg', 'allocation']
            commands.append(['build-' + name, command])
commands.append(['measure', ['python3', str(E / 'measure_resources_v1.py'), str(TARGET)]])
for suffix, value in [('.json', commands), ('-target.json', str(TARGET)),
                      ('-seed.json', None), ('-preserve.json', preserved)]:
    path = E / ('resources-v1' + suffix)
    assert not path.exists()
    path.write_text(json.dumps(value, indent=2) + '\n')

previous = json.loads((E.parent / '2026-10-16-polyhedral-construction/checkpoint-v1.json').read_text())
pending = previous['pending_native_sha256']
names = subprocess.check_output(['git', 'diff', '--name-only', '-z', 'HEAD',
                                '--', 'src', 'tests', 'crates', 'readme.md'], cwd=ROOT).decode().split('\0')
names += subprocess.check_output(['git', 'ls-files', '-z', '--others', '--exclude-standard',
                                 '--', 'src', 'tests', 'crates', 'readme.md'], cwd=ROOT).decode().split('\0')
sizes = {}
for name in sorted(set(names) - set(pending) - {''}):
    old = subprocess.run(['git', 'show', 'HEAD:' + name], cwd=ROOT, capture_output=True)
    new = (ROOT / name).read_bytes()
    if old.returncode == 0 and old.stdout == new: continue
    sizes[name] = {'baseline_exists': old.returncode == 0,
                  'baseline_bytes': len(old.stdout), 'candidate_bytes': len(new),
                  'baseline_lines': len(old.stdout.splitlines()), 'candidate_lines': len(new.splitlines())}
host = {
    'builds': {name: str(path) for name, path in BUILDS.items()}, 'fingerprints': pins,
    'libraries': libraries, 'source_sizes': sizes,
    'baseline_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
    'cargo_lock_sha256': sha(ROOT / 'Cargo.lock'),
    'rustc': subprocess.check_output(['rustc', '-vV'], text=True),
    'hardware': subprocess.check_output(['lscpu'], text=True),
    'system': subprocess.check_output(['uname', '-a'], text=True),
    'library_bytes': {variant: {name: (build / filename).stat().st_size
                               for name, filename in libraries[variant].items()}
                      for variant, build in BUILDS.items()},
    'scope': 'Fixed pre-edit cube driver; retrospective scalar controls/proof expansion; new-only general sums. No optimization claim. See cost-contract-v1.md.',
}
(E / 'cost-host-v1.json').write_text(json.dumps(host, indent=2) + '\n')
print(json.dumps({'commands': len(commands), 'pinned_libraries': len(preserved)}))
