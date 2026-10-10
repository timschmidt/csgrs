"""Focused native validation followed by one unchanged full-board gate per variant."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import time

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
OUT = EVIDENCE / 'arc-endpoint-native'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
records = {variant: read(OUT / (variant + '-sources.json')) for variant in ['baseline', 'candidate']}
maps = {}
for variant, record in records.items():
    path = Path(record['source_hashes_file'])
    assert digest(path) == record['source_hashes_file_sha256']
    maps[variant] = json.loads(gzip.decompress(path.read_bytes()))
native = read(EVIDENCE / 'native-exactness-gate/sources.json')['source_hashes']
assert not (OUT / 'status.json').exists()
shutil.copyfile(__file__, OUT / 'reproduce.py')
status, binaries = [], {}


def verify():
    for variant, record in records.items():
        source = Path(record['source_directory'])
        assert all(digest(source / name) == value for name, value in maps[variant].items())
    assert all(digest(ROOT / name) == value for name, value in native.items())


def run(variant, name, command, expected_tests=None, gate=False):
    verify()
    print('start', variant, name, flush=True)
    record = records[variant]
    env = os.environ | {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CARGO_TARGET_DIR': record['target_directory'], 'CCACHE_DIR': '/tmp/csgrs-properties-ccache', 'CARGO_NET_OFFLINE': 'true'}
    path = OUT / (variant + '-' + name + '.log')
    start = time.monotonic()
    with path.open('w') as log:
        result = subprocess.run(command, cwd=record['source_directory'], env=env, stdout=log, stderr=subprocess.STDOUT, check=False)
    entry = {'variant': variant, 'name': name, 'command': command, 'exit_code': result.returncode, 'wall_seconds': time.monotonic() - start}
    status.append(entry)
    (OUT / 'status.json').write_text(json.dumps(status, indent=2) + '\n')
    verify()
    print('finish', variant, name, result.returncode, flush=True)
    if gate:
        assert result.returncode in [0, 101]
        raw = path.read_text()
        assert 'skipping complex project fixture test' not in raw
        assert raw.count('hyperdrc: finished emit report') == 2
        if result.returncode:
            assert re.search(r'complex project KiCad smoke suite took [0-9.]+s of worker time', raw)
    else:
        if result.returncode:
            raise SystemExit(result.returncode)
        if expected_tests is not None:
            summaries = re.findall(r'test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;', path.read_text())
            assert summaries, path
            counts = [sum(int(row[column]) for row in summaries) for column in range(3)]
            assert counts == [expected_tests, 0, 0], (path, counts, expected_tests)
            entry['test_counts'] = dict(zip(['passed', 'failed', 'ignored'], counts))
            (OUT / 'status.json').write_text(json.dumps(status, indent=2) + '\n')


for variant in records:
    run(variant, 'format', ['cargo', 'fmt', '--all', '--', '--check'])
    run(variant, 'curve-clippy', ['cargo', 'clippy', '--locked', '--release', '-p', 'hypercurve', '--features', 'triangulation', '--all-targets', '--', '-D', 'warnings'])
    common = ['cargo', 'test', '--locked', '--release', '-p', 'hypercurve', '--features', 'triangulation']
    run(variant, 'point-regressions', common + ['--lib', 'point::', '--', '--test-threads=2'], expected_tests=14)
    run(variant, 'segment-regressions', common + ['--lib', 'segment::', '--', '--test-threads=2'], expected_tests=7)
    run(variant, 'curve-integrations', common + ['--test', 'hypercurve_arc_bezier', '--test', 'hypercurve_circle_predicates', '--test', 'native_linear_stroke', '--', '--test-threads=2'], expected_tests=25)
    run(variant, 'drc-native-regressions', ['cargo', 'test', '--locked', '--release', '-p', 'hyperdrc', '--test', 'native_linear_strokes', '--', '--test-threads=2'], expected_tests=26)
    run(variant, 'drc-clippy', ['cargo', 'clippy', '--locked', '--release', '-p', 'hyperdrc', '--all-targets', '--', '-D', 'warnings'])
    run(variant, 'build-board-smoke', ['cargo', 'test', '--locked', '--release', '-p', 'hyperdrc', '--lib', '--no-run', '--message-format=json'])
    executables = []
    for line in (OUT / (variant + '-build-board-smoke.log')).read_text().splitlines():
        if line.startswith('{'):
            event = json.loads(line)
            if event.get('reason') == 'compiler-artifact' and event.get('executable') and event['target']['name'] == 'hyperdrc':
                executables.append(event['executable'])
    assert len(executables) == 1
    binary = Path(executables[0])
    binaries[variant] = {'path': str(binary), 'sha256': digest(binary), 'bytes': binary.stat().st_size}
    (OUT / 'binaries.json').write_text(json.dumps(binaries, indent=2) + '\n')

for variant in records:
    binary = binaries[variant]
    assert digest(Path(binary['path'])) == binary['sha256']
    run(variant, 'full-board-smoke', ['/usr/bin/time', '-v', '-o', str(OUT / (variant + '-full-board-smoke-time.log')), binary['path'], 'zz_complex_project_performance_tests::kicad_board_completes_smoke_check_suite', '--exact', '--test-threads=1', '--nocapture'], gate=True)

verify()
assert all(digest(Path(binary['path'])) == binary['sha256'] for binary in binaries.values())
(OUT / 'source-verification.json').write_text(json.dumps({'all_source_and_binary_hashes_unchanged': True, 'native_inputs_each': len(maps['candidate']), 'main_files': len(native), 'both_variants_executed_original_board_gate': True, 'native_admission': False}, indent=2) + '\n')
print('Native comparison complete; original gate failures, if any, are preserved.', flush=True)
