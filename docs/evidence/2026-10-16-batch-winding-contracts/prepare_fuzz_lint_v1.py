"""Prepare a fresh final phase for the two unchanged-assertion harness repairs."""
import gzip
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
PHASE = 'fuzz-lint-repair-v1'
qualification = json.loads((E / 'qualification-v1/result.json').read_text())
assert len(qualification['commands']) == 50 and not qualification['passed']
assert [row['name'] for row in qualification['commands'] if row['exit_code']] == ['hypertri-fuzz-clippy']
with gzip.open(E / 'qualification-v1/source-hashes.json.gz', 'rt') as stream:
    before = json.load(stream)
names = ['crates/hypertri/fuzz/fuzz_targets/hyperreal_representations.rs',
         'crates/hypertri/fuzz/fuzz_targets/topology_invariants.rs']
changes = {name: {'before_sha256': before[name],
                  'after_sha256': hashlib.sha256((ROOT / name).read_bytes()).hexdigest()}
           for name in names}
assert all(row['before_sha256'] != row['after_sha256'] for row in changes.values())
for name, digest in before.items():
    if name not in changes:
        assert hashlib.sha256((ROOT / name).read_bytes()).hexdigest() == digest, name
path = E / 'fuzz-lint-adjustment-v1.json'
assert not path.exists()
path.write_text(json.dumps({'sources': changes, 'scope': 'Only fixed-size triangle iteration; all assertions retained.'}, indent=2) + '\n')

source = (E / 'setup_overlay_v3.py').read_text()
old = "LOWER = E.parents[2] / 'target/properties/batch-winding-contracts/candidate-focused-v2'"
assert source.count(old) == 1
path = E / 'setup_overlay_v4.py'
assert not path.exists()
path.write_text(source.replace(old, "LOWER = E.parents[2] / 'target/properties/batch-winding-contracts/qualification-v1'"))
target = ROOT / 'target/properties/batch-winding-contracts' / PHASE
commands = [
    ['overlay', ['python3', str(path), PHASE]],
    ['root-format', ['cargo', 'fmt', '--all', '--', '--check']],
    ['fuzz-format', ['cargo', 'fmt', '--manifest-path', 'crates/hypertri/fuzz/Cargo.toml', '--', '--check']],
    ['fuzz-check', ['cargo', 'check', '--locked', '--manifest-path', 'crates/hypertri/fuzz/Cargo.toml', '--bins']],
    ['fuzz-clippy', ['cargo', 'clippy', '--locked', '--manifest-path', 'crates/hypertri/fuzz/Cargo.toml', '--bins', '--', '-D', 'warnings']],
]
preserved = json.loads((E / 'qualification-v1-preserve.json').read_text())
for row in qualification['commands']:
    preserved.update(row['binaries_sha256'])
for path in (ROOT / 'target/properties/batch-winding-contracts/qualification-v1/release/deps').iterdir():
    if path.is_file() and path.suffix in {'.rlib', '.rmeta', '.so'}:
        with path.open('rb') as stream:
            preserved[str(path)] = hashlib.file_digest(stream, 'sha256').hexdigest()
for suffix, value in [('.json', commands), ('-target.json', str(target)),
                      ('-seed.json', None), ('-preserve.json', preserved)]:
    path = E / (PHASE + suffix)
    assert not path.exists()
    path.write_text(json.dumps(value, indent=2) + '\n')
print(json.dumps({'commands': len(commands), 'changed_harnesses': len(changes), 'preserved': len(preserved)}))
