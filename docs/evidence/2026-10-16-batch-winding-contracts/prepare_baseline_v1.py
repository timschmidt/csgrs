"""Reproduce the complete retained diagnostic with current immutable libraries."""
import hashlib
import json
from pathlib import Path
import subprocess

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
OLD = E.parent / '2026-10-16-minkowski-contracts'
TARGET = Path('/dev/shm/csgrs-batch-winding-contracts/baseline-v1')
assert not TARGET.exists()
assert subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip() == 'f74a7848805398eb62dddb167b3997662e6f90d5'
host = json.loads((OLD / 'cost-host-v1.json').read_text())
build = Path(host['builds']['candidate'])
libraries = host['libraries']['candidate']
previous_pins = json.loads((OLD / 'resources-v1-preserve.json').read_text())
preserved = {}
for path in build.iterdir():
    if path.is_file() and path.suffix in {'.rlib', '.rmeta', '.so'}:
        with path.open('rb') as stream:
            preserved[str(path)] = hashlib.file_digest(stream, 'sha256').hexdigest()
        assert preserved[str(path)] == previous_pins[str(path)], path
commands = [
    ['mkdir', ['mkdir', '-p', str(TARGET)]],
    ['build', ['rustc', '--edition=2024', '-Copt-level=2', '-D', 'warnings',
               str(OLD / 'union_probe_v1.rs'),
               '--extern', f'csgrs={build}/{libraries["csgrs"]}',
               '--extern', f'hypermesh={build}/{libraries["hypermesh"]}',
               '-L', f'dependency={build}', '-o', str(TARGET / 'probe')]],
    ['probe', [str(TARGET / 'probe')]],
]
for suffix, value in [('.json', commands), ('-target.json', str(TARGET)),
                      ('-seed.json', None), ('-preserve.json', preserved)]:
    path = E / ('baseline-v1' + suffix)
    assert not path.exists()
    path.write_text(json.dumps(value, indent=2) + '\n')
print(json.dumps({'commands': len(commands), 'pinned_libraries': len(preserved),
                  'current_libraries': libraries}), flush=True)
