"""Freeze compatibility with the preserved native geometry prototype.

Do not run while any build, validation, or measurement is active.
"""
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
production = Path(sys.argv[1]).resolve()
record = json.loads((production / 'sources.json').read_text())
parent_path = EVIDENCE.parent / '2026-10-10-hyperdrc-rule-export/native-compatibility/sources.json'
parent = json.loads(parent_path.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
source_map_path = Path(parent['source_hashes_file'])
assert sha(source_map_path) == parent['source_hashes_file_sha256']
values = json.loads(gzip.decompress(source_map_path.read_bytes()))
main_map_path = Path(record['main_hashes_file'])
assert sha(main_map_path) == record['main_hashes_file_sha256']
main = json.loads(gzip.decompress(main_map_path.read_bytes()))
assert all(sha(Path(parent['source_directory']) / name) == value for name, value in values.items())
assert all(sha(ROOT / name) == value for name, value in main.items())
OUT = EVIDENCE / 'native-compatibility'
OUT.mkdir()
work = ROOT / 'target/diagnostic-formatting-native'
work.mkdir()
source, target = work / 'source', work / 'target'
subprocess.run(['cp', '-a', '--reflink=auto', parent['source_directory'], str(source)], check=True)
subprocess.run(['cp', '-a', '--reflink=auto', record['target_directory'], str(target)], check=True)
for name in record['changed_files']:
    dest = source / name
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(ROOT / name, dest)
    values[name] = sha(dest)
assert all(sha(source / name) == value for name, value in values.items())
assert all(sha(source / name) == value for name, value in main.items())
result = {'source_directory': str(source), 'target_directory': str(target), 'production_source_record': str(production / 'sources.json'), 'production_source_record_sha256': sha(production / 'sources.json'), 'native_parent_source_record': str(parent_path), 'native_parent_source_record_sha256': sha(parent_path), 'source_files': len(values), 'main_files': len(main), 'changed_files': record['changed_files'], 'scope': 'Compatibility of diagnostic formatting with the preserved native geometry prototype. The prototype is not admitted and its earlier full-board performance gate remains open.'}
for kind, mapping in [('source', values), ('main', main)]:
    path = OUT / (kind + '-hashes.json.gz')
    path.write_bytes(gzip.compress((json.dumps(mapping, sort_keys=True, indent=2) + '\n').encode(), mtime=0))
    result[kind + '_hashes_file'] = str(path)
    result[kind + '_hashes_file_sha256'] = sha(path)
for name in record['changed_files']:
    path = OUT / 'inputs' / name
    path.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(source / name, path)
shutil.copyfile(__file__, OUT / 'prepare.py')
(OUT / 'sources.json').write_text(json.dumps(result, indent=2) + '\n')
print('Native compatibility source frozen:', len(values), 'source files;', len(main), 'main guards.', flush=True)
