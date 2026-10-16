"""Prepare a fresh downstream qualification without modifying prior targets."""
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
PREVIOUS = E.parent / '2026-10-16-minkowski-contracts'
PHASE = 'qualification-v1'
TARGET = ROOT / 'target/properties/batch-winding-contracts' / PHASE
assert json.loads((E / 'candidate-focused-v2/result.json').read_text())['passed']
assert not TARGET.exists() and not (E / PHASE).exists()

setup = E / 'setup_overlay_v3.py'
assert not setup.exists()
setup.write_text((E / 'setup_overlay_v1.py').read_text().replace(
    "E.parents[2] / 'target/properties/minkowski-contracts/benchmark-repair-v1'",
    "E.parents[2] / 'target/properties/batch-winding-contracts/candidate-focused-v2'"))
benchmark = E / 'run_benchmarks_v1.py'
assert not benchmark.exists()
benchmark.write_bytes((PREVIOUS / 'run_benchmarks_v1.py').read_bytes())

old_target = str(ROOT / 'target/properties/minkowski-contracts/qualification-v1')
commands = json.loads((PREVIOUS / 'qualification-v1.json').read_text().replace(old_target, str(TARGET)))
# These unchanged layers do not call the topology-only triangulation path.
# Retain their prior pinned qualification without presenting it as a new run.
omitted = {'scalar-all-features', 'scalar-default', 'foundations',
           'scalar-native-clippy', 'scalar-docs', 'curve-native'}
commands = [row for row in commands if row[0] not in omitted]
for row in commands:
    if row[0] == 'overlay':
        row[1] = ['python3', str(setup.relative_to(ROOT)), PHASE]
    elif row[0] == 'benchmark-consumers':
        row[1] = ['python3', str(benchmark.relative_to(ROOT)), str(TARGET)]
    elif row[0] == 'native-minimal':
        row[1] = ['cargo', 'test', '--locked', '--release', '-p', 'hypertri',
                  '--no-default-features', '--lib', '--tests', '--', '--test-threads=2']
matched = [row for row in commands if row[0] == 'matched-benchmark-library']
assert len(matched) == 1
commands = [row for row in commands if row[0] != 'matched-benchmark-library']
commands.extend([
    ['triangulation-cdt-only', ['cargo', 'test', '--locked', '--release', '-p', 'hypertri',
                               '--no-default-features', '--features', 'cdt', '--lib', '--tests',
                               '--', '--test-threads=2']],
    ['triangulation-cdt-clippy', ['cargo', 'clippy', '--locked', '--release', '-p', 'hypertri',
                                 '--no-default-features', '--features', 'cdt', '--all-targets',
                                 '--', '-D', 'warnings']],
    ['triangulation-minimal-clippy', ['cargo', 'clippy', '--locked', '--release', '-p', 'hypertri',
                                     '--no-default-features', '--all-targets', '--', '-D', 'warnings']],
    ['wasm-clippy', ['cargo', 'clippy', '--locked', '-p', 'csgrs', '--profile', 'wasm',
                     '--target', 'wasm32-unknown-unknown', '--no-default-features', '--features',
                     'wasm', '--lib', '--', '-D', 'warnings']],
])
for crate in ['hypertri', 'hypermesh']:
    manifest = f'crates/{crate}/fuzz/Cargo.toml'
    commands.extend([
        [crate + '-fuzz-check', ['cargo', 'check', '--locked', '--manifest-path', manifest, '--bins']],
        [crate + '-fuzz-clippy', ['cargo', 'clippy', '--locked', '--manifest-path', manifest,
                                 '--bins', '--', '-D', 'warnings']],
    ])
commands.extend(matched)
assert len(commands) == 50 and len({row[0] for row in commands}) == len(commands)
preserved = json.loads((E / 'candidate-focused-v2-preserve.json').read_text())
for row in json.loads((E / 'candidate-focused-v2/result.json').read_text())['commands']:
    preserved.update(row['binaries_sha256'])
upper = Path('/dev/shm/csgrs-batch-winding-contracts/candidate-focused-v2/upper/release/deps')
for path in upper.iterdir():
    if path.is_file() and path.suffix in {'.rlib', '.rmeta', '.so'}:
        with path.open('rb') as stream:
            preserved[str(path)] = hashlib.file_digest(stream, 'sha256').hexdigest()
for suffix, value in [('.json', commands), ('-target.json', str(TARGET)),
                      ('-seed.json', None), ('-preserve.json', preserved)]:
    path = E / (PHASE + suffix)
    assert not path.exists()
    path.write_text(json.dumps(value, indent=2) + '\n')
print(json.dumps({'commands': len(commands), 'preserved_libraries_and_executables': len(preserved),
                  'scope': 'Affected local/downstream qualification; not the complete CI powerset, coverage campaign, or external release.'}))
