"""Serial old/new artifact costs and unchanged original full-board reports."""
from collections import defaultdict
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import statistics
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
CANDIDATE = Path(sys.argv[1]).resolve()
OUT = CANDIDATE / 'reports-cost'
OUT.mkdir()
(OUT / '.gitignore').write_text('/reports-1/\n/reports-2/\n')
WORK = ROOT / 'target/artifact-numeric-keys-reports-cost'
WORK.mkdir()
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
info = lambda p: {'bytes': p.stat().st_size, 'sha256': sha(p)}
records = {'candidate': read(CANDIDATE / 'sources.json'), 'baseline': read(EVIDENCE.parent / '2026-10-10-diagnostic-formatting/candidate-v5/sources.json')}
guards = []
libraries = {}
for variant, record in records.items():
    source, target = Path(record['source_directory']), Path(record['target_directory'])
    for kind, base in [('source', source), ('main', ROOT)]:
        if variant == 'baseline' and kind == 'main':
            continue
        path = Path(record[kind + '_hashes_file'])
        assert sha(path) == record[kind + '_hashes_file_sha256']
        guards.append((base, json.loads(gzip.decompress(path.read_bytes()))))
    library = target / 'release/libhyperdrc.rlib'
    matched = [p for p in (target / 'release/deps').glob('libhyperdrc-*.rlib') if sha(p) == sha(library)]
    assert len(matched) == 1
    fingerprints = target / 'release/.fingerprint'
    metadata = read(fingerprints / matched[0].stem.removeprefix('lib') / 'lib-hyperdrc.json')
    libraries[variant] = {'hyperdrc': library}
    for name in ['serde_json', 'clap']:
        identifier = next(row[3] for row in metadata['deps'] if row[1] == name)
        paths = [p for p in (target / 'release/deps').glob('lib' + name + '-*.rlib') if int.from_bytes(bytes.fromhex((fingerprints / p.stem.removeprefix('lib') / ('lib-' + name)).read_text()), 'little') == identifier]
        assert len(paths) == 1
        libraries[variant][name] = paths[0]
    validation = read(CANDIDATE / 'full/summary.json') if variant == 'candidate' else read(EVIDENCE.parent / '2026-10-10-diagnostic-formatting/candidate-v5/full/validation.json')
    assert info(library) == validation['libraries'][str(library)]
library_info = {str(p): info(p) for group in libraries.values() for p in group.values()}
for name in ['cost_probe.rs', 'report_probe.rs']:
    shutil.copyfile(EVIDENCE / name, OUT / name)
shutil.copyfile(__file__, OUT / 'reproduce.py')
frozen = {p: info(p) for p in OUT.iterdir() if p.is_file()}
input_record = read(EVIDENCE.parent / '2026-10-09-hyperdrc-native-pad-primitives/report-replay/manifest.json')
inputs = Path(input_record['trials'][0]['command'][-2])
inventory = lambda: {str(p.relative_to(inputs)): info(p) for p in sorted(inputs.rglob('*')) if p.is_file()}
assert inventory() == input_record['input_files']
commands = []


def verify():
    assert all(sha(base / name) == value for base, values in guards for name, value in values.items())
    assert all(info(Path(p)) == value for p, value in library_info.items())
    assert all(info(p) == value for p, value in frozen.items())


def run(name, command):
    print('start', name, flush=True)
    start = time.monotonic()
    with (OUT / (name + '.stdout')).open('w') as stdout, (OUT / (name + '.stderr')).open('w') as stderr:
        result = subprocess.run(['timeout', '--signal=TERM', '--kill-after=5s', '1200', *map(str, command)], cwd=ROOT, stdout=stdout, stderr=stderr, check=False)
    commands.append({'name': name, 'command': list(map(str, command)), 'exit_code': result.returncode, 'wall_seconds': time.monotonic() - start, 'timeout_seconds': 1200})
    (OUT / 'commands.json').write_text(json.dumps(commands, indent=2) + '\n')
    print('finish', name, result.returncode, flush=True)
    assert result.returncode == 0, name
    return (OUT / (name + '.stdout')).read_text()


verify()
for name in ['cost_probe.rs', 'report_probe.rs']:
    run(name + '-format', ['rustfmt', '--edition', '2024', '--check', OUT / name])
for variant, group in libraries.items():
    for kind in ['time', 'alloc'] + (['report'] if variant == 'candidate' else []):
        source = OUT / ('report_probe.rs' if kind == 'report' else 'cost_probe.rs')
        command = ['rustc', '--edition=2024', '-O', '-D', 'warnings', source, '-L', 'dependency=' + str(Path(records[variant]['target_directory']) / 'release/deps')]
        for dep in ['hyperdrc', 'serde_json'] + (['clap'] if kind == 'report' else []):
            command += ['--extern', dep + '=' + str(group[dep])]
        if kind == 'alloc':
            command += ['--cfg', 'feature="count-alloc"']
        binary = WORK / (variant + '-' + kind)
        run(variant + '-' + kind + '-build', command + ['-o', binary])
        frozen[binary] = info(binary)
verify()
print('All builds complete; measurements begin.', flush=True)
reports = {}
previous = EVIDENCE.parent / '2026-10-10-diagnostic-formatting/candidate-v5/reports-cost-v2'
for trial in [1, 2]:
    destination = OUT / ('reports-' + str(trial))
    run('reports-' + str(trial), ['taskset', '-c', '0', '/usr/bin/time', '-v', WORK / 'candidate-report', inputs, destination])
    for board in ['cparti-fpga', 'hvp109a']:
        path = destination / (board + '.json')
        raw = path.read_bytes()
        old = previous / ('reports-1-' + board + '.json.gz')
        assert raw == gzip.decompress(old.read_bytes()), board
        (OUT / f'reports-{trial}-{board}.json.gz').write_bytes(gzip.compress(raw, mtime=0))
        result = info(path) | {'violation_count': json.loads(raw)['violation_count'], 'byte_identical_to_admitted': True, 'admitted_path': str(old), 'admitted_sha256': sha(old)}
        assert board not in reports or reports[board] == result
        reports[board] = result
    verify()
assert inventory() == input_record['input_files']
rows = []
for trial in range(5):
    for variant in (['baseline', 'candidate'] if trial % 2 == 0 else ['candidate', 'baseline']):
        raw = run(f'{variant}-time-{trial}', ['taskset', '-c', '0', WORK / (variant + '-time'), '20'])
        values = [json.loads(line) for line in raw.splitlines()]
        assert len(values) == 6
        rows.extend(row | {'variant': variant, 'trial': trial, 'kind': 'time'} for row in values)
for variant in ['baseline', 'candidate']:
    raw = run(variant + '-alloc', ['taskset', '-c', '0', WORK / (variant + '-alloc'), '20'])
    values = [json.loads(line) for line in raw.splitlines()]
    assert len(values) == 6
    rows.extend(row | {'variant': variant, 'trial': 0, 'kind': 'allocation'} for row in values)
(OUT / 'cost-rows.json').write_text(json.dumps(rows, indent=2) + '\n')
groups = defaultdict(list)
witnesses = {}
for row in rows:
    key = row['family'], row['variant']
    assert key not in witnesses or witnesses[key] == row['witness']
    witnesses[key] = row['witness']
    if row['kind'] == 'time':
        groups[key].append(row['nanos'] / row['iterations'])
for variant in ['baseline', 'candidate']:
    for family in ['ordinary', 'coincident', 'near-placements', 'near-rotations', 'rational', 'extremes']:
        placement_count = 128 if family == 'coincident' else int(family == 'near-placements' and variant == 'baseline')
        rotation_count = 256 if family == 'extremes' or (family == 'near-rotations' and variant == 'candidate') else 0
        assert witnesses[(family, variant)][1:] == [placement_count, rotation_count], (family, variant)
comparisons = []
for family in sorted({row['family'] for row in rows}):
    before, after = groups[(family, 'baseline')], groups[(family, 'candidate')]
    comparisons.append({'family': family, 'baseline_median_ns': statistics.median(before), 'candidate_median_ns': statistics.median(after), 'ratio': statistics.median(after)/statistics.median(before), 'ranges_overlap': max(min(before), min(after)) <= min(max(before), max(after)), 'baseline_witness': witnesses[(family, 'baseline')], 'candidate_witness': witnesses[(family, 'candidate')]})
(OUT / 'cost-comparison.json').write_text(json.dumps(comparisons, indent=2) + '\n')
verify()
summary = {'source_record': str(CANDIDATE / 'sources.json'), 'source_record_sha256': sha(CANDIDATE / 'sources.json'), 'guards_unchanged': True, 'libraries': library_info, 'binaries': {str(p): v for p, v in frozen.items() if p.parent == WORK}, 'reports': reports, 'report_processes': 2, 'cost_trials_per_variant': 5, 'cost_allocation_trials_per_variant': 1, 'iterations': 20, 'rows_per_artifact': 256, 'input_files': input_record['input_files'], 'scope': 'Exactness repair with explicit unavailable numbers, cost/allocation/size evidence, and unchanged original full-resolution reports. Existing native geometry prototype is not admitted.'}
summary['artifacts'] = {str(p.relative_to(OUT)): info(p) for p in sorted(OUT.rglob('*')) if p.is_file() and p.parent.name not in ['reports-1', 'reports-2']}
(OUT / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
print('Report and cost evidence complete.', flush=True)
