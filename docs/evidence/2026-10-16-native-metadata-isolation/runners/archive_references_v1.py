"""Retain referenced executable bytes through verified physical files or fresh copies."""
import concurrent.futures
import gzip
import hashlib
import json
from pathlib import Path
import threading

E = Path(__file__).resolve().parents[1]
ROOT = E.parents[2]
WORK = Path('/dev/shm/csgrs-native-metadata-isolation/reference-copies-v1')
OUT = E / 'closure-diagnostic-v1/references-v1.json.gz'
lock = threading.Lock()
assert not OUT.exists()
WORK.mkdir(exist_ok=False)
references = {}


def retain(path, digest):
    if path in references:
        assert references[path] == digest
    references[path] = digest


for phase in ['baseline-v1', 'candidate-v1']:
    for command in json.loads((E / phase / 'result.json').read_text())['commands']:
        for path, digest in command['binaries_sha256'].items():
            retain(path, digest)
for row in json.loads((E / 'cost-v1/binaries.json').read_text()).values():
    retain(row['path'], row['sha256'])
for row in json.loads((E / 'report-replay-v1/protocol.json').read_text())['binaries'].values():
    retain(row['path'], row['sha256'])
overlays = []
for phase in ['baseline-v1', 'candidate-v1', 'cost-baseline-v1', 'cost-candidate-v1']:
    d = json.loads((E / phase / 'overlay.json').read_text())
    overlays.append((Path(d['target']), Path(d['upper'])))


def sha(data):
    return hashlib.sha256(data).hexdigest()


def copy_bytes(data, name):
    digest = sha(data)
    path = WORK / digest / name
    with lock:
        path.parent.mkdir(exist_ok=True)
        if not path.exists():
            with path.open('xb') as stream:
                stream.write(data)
        assert sha(path.read_bytes()) == digest
    return str(path)


def inspect(item):
    name, expected = item
    path = ROOT / name
    row = {'path': name, 'expected_sha256': expected, 'content_verified': False}
    for target, upper in overlays:
        if path.is_relative_to(target):
            physical = upper / path.relative_to(target)
            if physical.is_file():
                data = physical.read_bytes()
                row['physical_candidate'] = str(physical)
                row['physical_candidate_sha256'] = sha(data)
                if sha(data) == expected:
                    row.update({'content_verified': True, 'verified_content_path': str(physical), 'bytes': len(data)})
            break
    try:
        data = path.read_bytes()
        row['view_sha256'] = sha(data)
        row['view_matches_expected'] = sha(data) == expected
        if not row['view_matches_expected']:
            row['unexpected_view_copy'] = copy_bytes(data, path.name)
        elif not row['content_verified']:
            row.update({'content_verified': True, 'verified_content_path': copy_bytes(data, path.name), 'bytes': len(data)})
    except OSError as error:
        row['view_error'] = repr(error)
        row['view_matches_expected'] = False
    return row


with concurrent.futures.ThreadPoolExecutor(max_workers=8) as pool:
    rows = list(pool.map(inspect, references.items()))
record = {'scope': __doc__, 'references': rows,
          'count': len(rows), 'all_referenced_bytes_retained': all(row['content_verified'] for row in rows),
          'view_mismatches_or_errors': sum(not row['view_matches_expected'] for row in rows),
          'compiler_input_identity_qualified': False, 'runtime_admitted': False}
with gzip.open(OUT, 'wt') as stream:
    json.dump(record, stream, sort_keys=True)
print(json.dumps({key: record[key] for key in ['count', 'all_referenced_bytes_retained', 'view_mismatches_or_errors', 'compiler_input_identity_qualified']}), flush=True)
if not record['all_referenced_bytes_retained']:
    raise SystemExit(101)
