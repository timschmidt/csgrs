"""Compare effective final-coordinate aborts with the measured production libraries."""
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
OUT = EVIDENCE / 'arc-endpoint-final-abort'
WORK = ROOT / 'target/arc-endpoint-final-abort'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
assert read(EVIDENCE / 'diagnostics-79.json')['board_gate']['passed']
production = {variant: read(EVIDENCE / 'arc-endpoint-certificate' / (variant + '-sources.json')) for variant in ['baseline', 'candidate']}
maps = {}
for variant, record in production.items():
    path = Path(record['source_hashes_file'])
    assert digest(path) == record['source_hashes_file_sha256']
    maps[variant] = json.loads(gzip.decompress(path.read_bytes()))
native = read(EVIDENCE / 'native-exactness-gate/sources.json')['source_hashes']
measured = read(EVIDENCE / 'arc-endpoint-cost/libraries.json')
OUT.mkdir()
WORK.mkdir()
shutil.copyfile(__file__, OUT / 'reproduce.py')
shutil.copyfile(EVIDENCE / 'arc_endpoint_abort_probe.rs', OUT / 'probe.rs')
probe_hash = digest(OUT / 'probe.rs')
libraries = {}
for variant in production:
    original = measured[variant]['hypercurve']
    retained = Path(original['retained_path'])
    assert digest(retained) == original['sha256']
    library = WORK / ('libhypercurve_' + variant + '.rlib')
    shutil.copyfile(retained, library)
    assert digest(library) == original['sha256']
    libraries[variant] = {'path': str(library), 'sha256': digest(library), 'retained_measured_library': str(retained)}
commands, trials = [], []


def verify():
    for variant, record in production.items():
        source = Path(record['source_directory'])
        assert all(digest(source / name) == value for name, value in maps[variant].items())
        assert digest(Path(libraries[variant]['path'])) == libraries[variant]['sha256']
    assert all(digest(ROOT / name) == value for name, value in native.items())
    assert digest(OUT / 'probe.rs') == probe_hash


def run(name, command):
    verify()
    print('start', name, flush=True)
    start = time.monotonic()
    with (OUT / (name + '.jsonl')).open('w') as stdout, (OUT / (name + '.log')).open('w') as stderr:
        result = subprocess.run(['timeout', '--signal=TERM', '--kill-after=10s', '180', *command], cwd=ROOT, stdout=stdout, stderr=stderr, check=False)
    commands.append({'name': name, 'command': command, 'exit_code': result.returncode, 'wall_seconds': time.monotonic() - start})
    (OUT / 'commands.json').write_text(json.dumps(commands, indent=2) + '\n')
    verify()
    print('finish', name, result.returncode, flush=True)
    assert result.returncode == 0, name
    return (OUT / (name + '.jsonl')).read_text()


run('format', ['rustfmt', '--edition', '2024', '--check', str(OUT / 'probe.rs')])
for variant, record in production.items():
    run('build-' + variant, ['rustc', '--edition=2024', '-O', '-D', 'warnings', str(OUT / 'probe.rs'), '-L', 'dependency=' + str(Path(record['target_directory']) / 'release/deps'), '--extern', 'hypercurve=' + libraries[variant]['path'], '-o', str(WORK / variant)])
for index in range(1, 4):
    for variant in (['baseline', 'candidate'] if index % 2 else ['candidate', 'baseline']):
        raw = run(f'{variant}-{index}', [str(WORK / variant)])
        rows = [json.loads(line) for line in raw.splitlines()]
        assert len(rows) == 210
        assert all(row['non_rational_query_coordinates'] > 0 for row in rows)
        assert all(row['available_before'] == row['available_after'] == (2 - row['non_rational_query_coordinates'] if row['abort_active'] else 2) for row in rows)
        trials.append({'variant': variant, 'index': index, 'rows': rows})
assert all(trial['rows'] == trials[0]['rows'] for trial in trials)
canonical = (json.dumps(trials[0]['rows'], sort_keys=True, separators=(',', ':')) + '\n').encode()
(OUT / 'canonical-receipts.json.gz').write_bytes(gzip.compress(canonical, mtime=0))
verify()
record = {
    'decision': 'effective final-coordinate abort differential gate passed; complete historical witness/report replay and native recheck still required',
    'final_query_abort_verified': True,
    'scope': 'Six public arc/query families, seven policy contexts, five active/repeated/cleared-prepared/reenabled-after-warm/cleared-final stages, in three fresh alternating processes per variant. The flag is attached to every non-rational final query coordinate after all arithmetic, and interval unavailability is asserted before and after each active query sequence.',
    'families': ['unresolved endpoint identity', 'radical interior circle point', 'radical off-circle point', 'pi-scaled and translated circle point', 'opaque translated circle point', 'copied pi/e endpoint coordinates'],
    'contexts': ['strict', 'approximate 512', 'strict edge preview', 'approximate edge preview', 'zero-work-budget strict', 'zero-work-budget approximate', 'strict evaluation nested in provisional scope'],
    'rows_each_fresh_process': 210, 'fresh_processes_each_variant': 3,
    'active_abort_rows_each_process': sum(row['abort_active'] for row in trials[0]['rows']),
    'canonical_receipts_sha256': hashlib.sha256(canonical).hexdigest(),
    'receipt_scope': 'First membership, point equality to both endpoints, repeated membership, aggregate certainty, and budget outcome/usage where applicable. Certified non-preview decisions are checked against exact circle/endpoint identities.',
    'libraries': libraries, 'production_sources': production,
    'all_sources_and_libraries_unchanged': True, 'main_runtime_unchanged': True,
    'earlier_coverage_correction': 'The initial probe and resource control attached a flag to an intermediate scalar and were not credited as effective final-coordinate abort coverage. Their frozen sources, labels, uncertain outcomes and samples are preserved. This separate gate supplies direct unavailable-interval assertions.',
    'limitations': 'An active abort need not prevent an exact structural proof. This gate verifies aborted final coordinates and preserved observed receipts, not propagation through every scalar rewrite or a universal requirement for uncertainty. Preview receipts are lossy. Zero shared-work budget is a scoped control, not an exhaustive budget sweep or a hard time bound. No simultaneous flag mutation race, full native admission or matched application speedup is claimed.',
}
record['artifacts'] = {str(path.relative_to(EVIDENCE)): {'bytes': path.stat().st_size, 'sha256': digest(path)} for path in sorted(OUT.iterdir()) if path.is_file()}
for name in ['arc_endpoint_abort_probe.rs', 'reproduce_arc_endpoint_abort.py']:
    path = EVIDENCE / name
    record['artifacts'][name] = {'bytes': path.stat().st_size, 'sha256': digest(path)}
(EVIDENCE / 'diagnostics-80.json').write_text(json.dumps(record, indent=2) + '\n')
print('Effective final-coordinate abort receipts match:', len(trials), 'processes x', len(trials[0]['rows']), 'rows.', flush=True)
