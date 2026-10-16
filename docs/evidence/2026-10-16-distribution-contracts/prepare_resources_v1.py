"""Freeze resource commands and dependency pins before measurements."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
TARGET = Path('/dev/shm/csgrs-distribution-contracts/resources-v1')
BUILDS = {
    'baseline': ROOT / 'target/properties/text-contracts/qualification-v3/release/deps',
    'candidate': ROOT / sys.argv[1] / 'release/deps',
}
assert not TARGET.exists()
def fingerprint(build, unit, name):
    return json.loads((build.parent / '.fingerprint' / unit / ('lib-' + name + '.json')).read_text())
root_pins = {variant: fingerprint(build, 'csgrs-5357807c720d8967', 'csgrs') for variant, build in BUILDS.items()}
mesh_pins = {variant: fingerprint(build, 'hypermesh-98c5db38df7e7da0', 'hypermesh') for variant, build in BUILDS.items()}
for key in ['features', 'declared_features', 'profile', 'rustflags', 'rustc', 'target']:
    assert root_pins['baseline'][key] == root_pins['candidate'][key], key
    assert mesh_pins['baseline'][key] == mesh_pins['candidate'][key], key
assert mesh_pins['baseline']['deps'] == mesh_pins['candidate']['deps']
for variant, build in BUILDS.items():
    encoded = (build.parent / '.fingerprint/hypermesh-98c5db38df7e7da0/lib-hypermesh').read_text().strip()
    expected = int.from_bytes(bytes.fromhex(encoded), 'little')
    assert [d[3] for d in root_pins[variant]['deps'] if d[1] == 'hypermesh'] == [expected]

preserved = {}
commands = [['mkdir', ['mkdir', '-p', str(TARGET)]]]
for variant, build in BUILDS.items():
    for path in sorted(build.iterdir()):
        if path.is_file() and path.suffix in {'.rlib', '.rmeta', '.so'}:
            with path.open('rb') as stream:
                preserved[str(path)] = hashlib.file_digest(stream, 'sha256').hexdigest()
    for kind in ['timing', 'allocation']:
        command = ['rustc', '--edition=2024', '-Copt-level=2', '-D', 'warnings',
                   str(E / 'resource_probe_v1.rs'), '--extern', f'csgrs={build}/libcsgrs.rlib',
                   '--extern', f'hypermesh={build}/libhypermesh-98c5db38df7e7da0.rlib',
                   '-L', f'dependency={build}', '-o', str(TARGET / f'{variant}-{kind}')]
        if kind == 'allocation':
            command += ['--cfg', 'allocation']
        commands.append([f'build-{variant}-{kind}', command])
commands += [['measure', ['python3', str(E / 'measure_resources_v1.py'), str(TARGET)]],
             ['independent-box-semantics', ['python3', str(E / 'check_box_corpus_v1.py')]]]
for suffix, value in [('.json', commands), ('-target.json', str(TARGET)),
                      ('-seed.json', None), ('-preserve.json', preserved)]:
    path = E / ('resources-v1' + suffix)
    assert not path.exists()
    path.write_text(json.dumps(value, indent=2) + '\n')
source_sizes = {}
for name in ['src/solid.rs', 'crates/hypermesh/src/mesh.rs', 'crates/hypermesh/src/mesh/concatenation.rs',
             'tests/distribution_contracts.rs', 'tests/native_queries.rs', 'tests/slice_projection_contract.rs',
             'tests/solid_cache.rs', 'benchmarks/rust/kernel_comparison.rs', 'benchmarks/rust/feature_pipeline.rs',
             'readme.md', 'crates/hypermesh/README.md']:
    previous = subprocess.run(['git', 'show', 'HEAD:' + name], cwd=ROOT, capture_output=True)
    assert previous.returncode == 0 or not subprocess.check_output(['git', 'ls-files', '--', name], cwd=ROOT)
    old = previous.stdout if previous.returncode == 0 else b''
    new = (ROOT / name).read_bytes()
    source_sizes[name] = {'baseline_exists': previous.returncode == 0,
                         'baseline_bytes': len(old), 'candidate_bytes': len(new),
                         'baseline_lines': len(old.splitlines()), 'candidate_lines': len(new.splitlines())}
host = {
    'builds': {key: str(value) for key, value in BUILDS.items()},
    'root_fingerprints': root_pins, 'native_fingerprints': mesh_pins, 'source_sizes': source_sizes,
    'rustc': subprocess.check_output(['rustc', '-vV'], text=True),
    'hardware': subprocess.check_output(['lscpu'], text=True),
    'system': subprocess.check_output(['uname', '-a'], text=True),
    'library_bytes': {variant: {name: (build / name).stat().st_size for name in
        ['libcsgrs.rlib', 'libhypermesh-98c5db38df7e7da0.rlib']} for variant, build in BUILDS.items()},
    'source_diff_numstat': subprocess.check_output(['git', 'diff', '--numstat', 'HEAD', '--',
        'src/solid.rs', 'crates/hypermesh/src/mesh.rs', 'crates/hypermesh/src/mesh/concatenation.rs', 'tests/distribution_contracts.rs'], cwd=ROOT, text=True),
    'scope': 'See cost-contract-v1.md. Exact output preservation required; this correctness extension makes no speedup claim.',
}
(E / 'cost-host-v1.json').write_text(json.dumps(host, indent=2) + '\n')
