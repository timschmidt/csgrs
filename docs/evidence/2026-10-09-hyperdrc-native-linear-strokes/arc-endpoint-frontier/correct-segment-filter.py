"""Correct the initial zero-test filter without changing any frozen Rust."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import time

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
OUT = EVIDENCE / 'arc-endpoint-frontier'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
status = read(OUT / 'status.json')
assert len(status) in [16, 17] and all(row['exit_code'] == 0 for row in status)
records = [read(OUT / (variant + '-sources.json')) for variant in ['baseline', 'candidate']]
records += [read(EVIDENCE / 'arc-endpoint-certificate' / (variant + '-sources.json')) for variant in ['baseline', 'candidate']]
maps = []
for record in records:
    path = Path(record['source_hashes_file'])
    assert digest(path) == record['source_hashes_file_sha256']
    maps.append((Path(record['source_directory']), json.loads(gzip.decompress(path.read_bytes()))))
native = read(EVIDENCE / 'native-exactness-gate/sources.json')['source_hashes']


def verify():
    for source, hashes in maps:
        assert all(digest(source / name) == value for name, value in hashes.items())
    assert all(digest(ROOT / name) == value for name, value in native.items())


shutil.copyfile(__file__, OUT / 'correct-segment-filter.py')
for variant, record in zip(['baseline', 'candidate'], records[:2]):
    assert '0 passed; 0 failed' in (OUT / (variant + '-segment-regressions.log')).read_text()
    verify()
    command = ['cargo', 'test', '--locked', '--release', '-p', 'hypercurve', '--features', 'triangulation', '--lib', 'segment::', '--', '--skip', 'segment::arc_endpoint_frontier_probe::', '--test-threads=2']
    env = os.environ | {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache', 'CARGO_NET_OFFLINE': 'true', 'CARGO_TARGET_DIR': record['target_directory']}
    name = 'segment-regressions-corrected'
    path = OUT / (variant + '-' + name + '.log')
    completed = [row for row in status if row['variant'] == variant and row['name'] == name]
    if completed:
        assert len(completed) == 1 and digest(path) == completed[0]['log_sha256']
        assert '2 passed; 0 failed' in path.read_text()
        print('reuse verified', variant, name, 'two passed', flush=True)
        continue
    start = time.monotonic()
    print('start', variant, name, flush=True)
    with path.open('w') as log:
        result = subprocess.run(command, cwd=record['source_directory'], env=env, stdout=log, stderr=subprocess.STDOUT, check=False)
    status.append({'variant': variant, 'name': name, 'command': command, 'exit_code': result.returncode, 'wall_seconds': time.monotonic() - start, 'log_sha256': digest(path)})
    (OUT / 'status.json').write_text(json.dumps(status, indent=2) + '\n')
    verify()
    assert result.returncode == 0 and '2 passed; 0 failed' in path.read_text()
    print('finish', variant, name, 'two passed', flush=True)
(OUT / 'corrected-filter-source-verification.json').write_text(json.dumps({'all_probe_production_and_main_source_hashes_unchanged': True, 'initial_zero_test_logs_preserved': True, 'corrected_segment_tests_each_variant': 2}, indent=2) + '\n')
