"""Freeze resource commands and dependency pins before measurements."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
TARGET = Path('/dev/shm/csgrs-text-contracts/resources-v1')
BUILDS = {
    'baseline': ROOT / 'target/properties/gear-airfoil-failures/qualification-v1/release/deps',
    'candidate': ROOT / sys.argv[1] / 'release/deps',
}
assert not TARGET.exists()
preserved = {}
commands = [['mkdir', ['mkdir', '-p', str(TARGET)]]]
for variant, build in BUILDS.items():
    for path in sorted(build.iterdir()):
        if path.is_file() and path.suffix in {'.rlib', '.rmeta', '.so'}:
            with path.open('rb') as stream:
                preserved[str(path)] = hashlib.file_digest(stream, 'sha256').hexdigest()
    for kind in ['timing', 'allocation']:
        command = ['rustc', '--edition=2024', '-Copt-level=2', '-D', 'warnings',
                   str(E / 'resource_probe_v3.rs'), '--extern', f'csgrs={build}/libcsgrs.rlib',
                   '--extern', f'hypercurve={build}/libhypercurve-65c0df1f5cff6824.rlib',
                   '-L', f'dependency={build}', '-o', str(TARGET / f'{variant}-{kind}')]
        if kind == 'allocation':
            command += ['--cfg', 'allocation']
        commands.append([f'build-{variant}-{kind}', command])
commands += [['measure', ['python3', str(E / 'measure_resources_v1.py'), str(TARGET)]]]
for suffix, value in [('.json', commands), ('-target.json', str(TARGET)),
                      ('-seed.json', None), ('-preserve.json', preserved)]:
    path = E / ('resources-v1' + suffix)
    assert not path.exists()
    path.write_text(json.dumps(value, indent=2) + '\n')
host = {
    'builds': {key: str(value) for key, value in BUILDS.items()},
    'rustc': subprocess.check_output(['rustc', '-vV'], text=True),
    'hardware': subprocess.check_output(['lscpu'], text=True),
    'system': subprocess.check_output(['uname', '-a'], text=True),
    'library_bytes': {variant: {name: (build / name).stat().st_size for name in
        ['libcsgrs.rlib', 'libhypercurve-65c0df1f5cff6824.rlib']} for variant, build in BUILDS.items()},
    'source_diff_numstat': subprocess.check_output(['git', 'diff', '--numstat', 'HEAD', '--',
        'src/curve/truetype.rs', 'src/curve/native.rs', 'src/errors.rs', 'src/solid.rs',
        'src/parts/dimension.rs', 'crates/hypercurve/src/hershey.rs'], cwd=ROOT, text=True),
    'scope': 'See cost-contract-v1.md. TrueType output migrates; no performance-only claim.',
}
(E / 'cost-host-v1.json').write_text(json.dumps(host, indent=2) + '\n')
