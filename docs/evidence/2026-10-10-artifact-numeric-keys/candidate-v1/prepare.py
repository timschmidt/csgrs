"""Freeze this scoped repair over an immutable admitted or native source tree."""
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
phase, parent_path = sys.argv[1], Path(sys.argv[2]).resolve()
parent = json.loads(parent_path.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
changed = ['crates/hyperdrc/src/checks/artifacts.rs', 'crates/hyperdrc/tests/artifact_numeric_keys.rs']
maps = {}
for kind, base in [('source', Path(parent['source_directory'])), ('main', ROOT)]:
    path = Path(parent[kind + '_hashes_file'])
    assert sha(path) == parent[kind + '_hashes_file_sha256']
    maps[kind] = json.loads(gzip.decompress(path.read_bytes()))
    assert all(sha(base / name) == expected for name, expected in maps[kind].items() if kind == 'source' or name not in changed)
out = EVIDENCE / phase
out.mkdir()
work = ROOT / ('target/artifact-numeric-keys-' + phase)
work.mkdir()
source, target = work / 'source', work / 'target'
subprocess.run(['cp', '-a', '--reflink=auto', parent['source_directory'], source], check=True)
subprocess.run(['cp', '-a', '--reflink=auto', parent['target_directory'], target], check=True)
for name in changed:
    shutil.copyfile(ROOT / name, source / name)
    frozen = out / 'inputs' / name
    frozen.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(ROOT / name, frozen)
    for values in maps.values():
        values[name] = sha(ROOT / name)
record = {'source_directory': str(source), 'target_directory': str(target), 'parent_source_record': str(parent_path), 'parent_source_record_sha256': sha(parent_path), 'changed_files': changed, 'scope': 'Exact numeric keys and explicit numeric unavailability for production sidecar artifacts.'}
for kind, base in [('source', source), ('main', ROOT)]:
    assert all(sha(base / name) == expected for name, expected in maps[kind].items())
    path = out / (kind + '-hashes.json.gz')
    path.write_bytes(gzip.compress((json.dumps(maps[kind], sort_keys=True, indent=2) + '\n').encode(), mtime=0))
    record[kind + '_files'] = len(maps[kind])
    record[kind + '_hashes_file'] = str(path)
    record[kind + '_hashes_file_sha256'] = sha(path)
shutil.copyfile(__file__, out / 'prepare.py')
(out / 'sources.json').write_text(json.dumps(record, indent=2) + '\n')
print('Frozen', phase, record['source_files'], 'source files;', record['main_files'], 'main guards.', flush=True)
