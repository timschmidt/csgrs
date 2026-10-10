"""Repeated complete board reports plus measured diagnostic conversion costs."""
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
WORK = ROOT / 'target/diagnostic-formatting-reports-cost'
WORK.mkdir(exist_ok=True)
read = lambda p: json.loads(p.read_text())
digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
info = lambda p: {'bytes': p.stat().st_size, 'sha256': digest(p)}
record = read(CANDIDATE / 'sources.json')
SOURCE, TARGET = Path(record['source_directory']), Path(record['target_directory'])
maps = []
for kind, directory in [('source', SOURCE), ('main', ROOT)]:
    path = Path(record[kind + '_hashes_file'])
    assert digest(path) == record[kind + '_hashes_file_sha256']
    maps.append((directory, json.loads(gzip.decompress(path.read_bytes()))))
library = TARGET / 'release/libhyperdrc.rlib'
assert info(library) == read(CANDIDATE / 'full/validation.json')['libraries'][str(library)]
shutil.copyfile(__file__, OUT / 'reproduce.py')
shutil.copyfile(EVIDENCE / 'cost_probe.rs', OUT / 'cost_probe.rs')
previous = EVIDENCE.parent / '2026-10-10-hyperreal-formatting-frontier/downstream-checked-scientific'
shutil.copyfile(previous / 'probe_sources/full_report_replay.rs', OUT / 'report_probe.rs')
frozen = {p: info(p) for p in OUT.iterdir() if p.is_file()}
matched = [p for p in (TARGET / 'release/deps').glob('libhyperdrc-*.rlib') if digest(p) == digest(library)]
assert len(matched) == 1
fingerprints = TARGET / 'release/.fingerprint'
metadata = read(fingerprints / matched[0].stem.removeprefix('lib') / 'lib-hyperdrc.json')


def dependency(name):
    identifier = next(row[3] for row in metadata['deps'] if row[1] == name)
    paths = [p for p in (TARGET / 'release/deps').glob('lib' + name + '-*.rlib') if int.from_bytes(bytes.fromhex((fingerprints / p.stem.removeprefix('lib') / ('lib-' + name)).read_text()), 'little') == identifier]
    assert len(paths) == 1, name
    return paths[0]


libraries = {'hyperdrc': library} | {name: dependency(name) for name in ['hyperreal', 'clap', 'serde_json']}
library_info = {str(p): info(p) for p in libraries.values()}
input_record = read(EVIDENCE.parent / '2026-10-09-hyperdrc-native-pad-primitives/report-replay/manifest.json')
inputs = Path(input_record['trials'][0]['command'][-2])
inventory = lambda: {str(p.relative_to(inputs)): info(p) for p in sorted(inputs.rglob('*')) if p.is_file()}
assert inventory() == input_record['input_files']
commands = []


def verify():
    assert all(digest(base / name) == expected for base, values in maps for name, expected in values.items())
    assert all(info(Path(p)) == expected for p, expected in library_info.items())
    assert all(info(p) == expected for p, expected in frozen.items())


def run(name, command, seconds=1200):
    print('start', name, flush=True)
    start = time.monotonic()
    with (OUT / (name + '.stdout')).open('w') as stdout, (OUT / (name + '.stderr')).open('w') as stderr:
        result = subprocess.run(['timeout', '--signal=TERM', '--kill-after=5s', str(seconds), *map(str, command)], cwd=ROOT, stdout=stdout, stderr=stderr, check=False)
    commands.append({'name': name, 'command': list(map(str, command)), 'exit_code': result.returncode, 'wall_seconds': time.monotonic() - start, 'timeout_seconds': seconds})
    (OUT / 'commands.json').write_text(json.dumps(commands, indent=2) + '\n')
    print('finish', name, result.returncode, flush=True)
    assert result.returncode == 0, name
    return (OUT / (name + '.stdout')).read_text()


verify()
for name, source, deps, cfgs in [('report', 'report_probe.rs', ['hyperdrc', 'clap', 'serde_json'], []), ('legacy-time', 'cost_probe.rs', ['hyperreal'], []), ('checked-time', 'cost_probe.rs', ['hyperreal'], ['checked-api']), ('legacy-alloc', 'cost_probe.rs', ['hyperreal'], ['count-alloc']), ('checked-alloc', 'cost_probe.rs', ['hyperreal'], ['checked-api', 'count-alloc'])]:
    run(name + '-format', ['rustfmt', '--edition', '2024', '--check', OUT / source])
    command = ['rustc', '--edition=2024', '-O', '-D', 'warnings', OUT / source, '-L', 'dependency=' + str(TARGET / 'release/deps')]
    for dep in deps:
        command += ['--extern', dep + '=' + str(libraries[dep])]
    for cfg in cfgs:
        command += ['--cfg', 'feature="' + cfg + '"']
    run(name + '-build', command + ['-o', WORK / name])
    frozen[WORK / name] = info(WORK / name)
verify()
print('All report/cost binaries built; measurements begin with builds stopped.', flush=True)
reports = {}
for trial in [1, 2]:
    destination = OUT / ('reports-' + str(trial))
    run('reports-' + str(trial), ['taskset', '-c', '0', '/usr/bin/time', '-v', WORK / 'report', inputs, destination])
    for board in ['cparti-fpga', 'hvp109a']:
        path = destination / (board + '.json')
        raw = path.read_bytes()
        data = json.loads(raw)
        old_path = previous / ('reports-1-' + board + '.json.gz')
        old_raw = gzip.decompress(old_path.read_bytes())
        old = json.loads(old_raw)
        assert data['violation_count'] == len(data['violations']) == old['violation_count']
        changes = []
        for index, (before, after) in enumerate(zip(old['violations'], data['violations'])):
            if before != after:
                fields = sorted(key for key in set(before) | set(after) if before.get(key) != after.get(key))
                changes.append({'index': index, 'id': after['id'], 'fields': fields, 'before': {key: before.get(key) for key in fields}, 'after': {key: after.get(key) for key in fields}})
        (OUT / f'reports-{trial}-{board}-differences.json').write_text(json.dumps(changes, indent=2) + '\n')
        assert all(row['fields'] == ['message'] for row in changes), 'inspect any decision/geometry/identity change'
        assert {k: v for k, v in data.items() if k != 'violations'} == {k: v for k, v in old.items() if k != 'violations'}
        (OUT / f'reports-{trial}-{board}.json.gz').write_bytes(gzip.compress(raw, mtime=0))
        report = info(path) | {'violation_count': data['violation_count'], 'changed_messages': len(changes), 'byte_identical_to_admitted': raw == old_raw, 'admitted_path': str(old_path), 'admitted_sha256': digest(old_path)}
        assert board not in reports or reports[board] == report
        reports[board] = report
    verify()
assert inventory() == input_record['input_files']
rows = []
for trial in range(5):
    for variant in (['legacy', 'checked'] if trial % 2 == 0 else ['checked', 'legacy']):
        raw = run(f'{variant}-time-{trial}', ['taskset', '-c', '0', WORK / (variant + '-time'), '1000'])
        values = [json.loads(line) for line in raw.splitlines()]
        assert len(values) == (22 if variant == 'legacy' else 28)
        rows.extend(row | {'variant': variant, 'trial': trial, 'kind': 'time'} for row in values)
for variant in ['legacy', 'checked']:
    raw = run(variant + '-alloc', ['taskset', '-c', '0', WORK / (variant + '-alloc'), '1000'])
    values = [json.loads(line) for line in raw.splitlines()]
    assert len(values) == (22 if variant == 'legacy' else 28)
    rows.extend(row | {'variant': variant, 'trial': 0, 'kind': 'allocation'} for row in values)
(OUT / 'cost-rows.json').write_text(json.dumps(rows, indent=2) + '\n')
groups = defaultdict(list)
for row in rows:
    if row['kind'] == 'time':
        groups[(row['family'], row['stage'], row['variant'])].append(row['nanos'] / row['iterations'])
comparisons = []
for family, stage, variant in sorted(groups):
    if variant != 'legacy':
        continue
    old, new = groups[(family, stage, 'legacy')], groups[(family, stage, 'checked')]
    comparisons.append({'family': family, 'stage': stage, 'legacy_median_ns': statistics.median(old), 'checked_median_ns': statistics.median(new), 'ratio': statistics.median(new) / statistics.median(old), 'sample_ranges_overlap': max(min(old), min(new)) <= min(max(old), max(new))})
(OUT / 'cost-comparison.json').write_text(json.dumps(comparisons, indent=2) + '\n')
verify()
summary = {'source_record': str(CANDIDATE / 'sources.json'), 'source_record_sha256': digest(CANDIDATE / 'sources.json'), 'source_main_library_probe_guards_unchanged': True, 'reports': reports, 'report_processes': 2, 'input_files': input_record['input_files'], 'libraries': library_info, 'binaries': {str(p): value for p, value in frozen.items() if p.parent == WORK}, 'cost_timing_trials_per_variant': 5, 'cost_allocation_trials_per_variant': 1, 'cost_objects_per_family': 1000, 'scope': 'Repeated full reports and direct conversion cost/allocation/size evidence. First-query objects constructed before timing; constants/construction caches may be warm. Old/new APIs share the frozen library. Each result is asserted, including added explicit failure outcomes. This is a diagnostic correctness contract with measured costs, not a speedup claim or the full native-geometry admission gate.'}
summary['artifacts'] = {str(p.relative_to(OUT)): info(p) for p in sorted(OUT.rglob('*')) if p.is_file() and p.parent.name not in ['reports-1', 'reports-2']}
(OUT / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
print('Repeated reports and five paired timing trials complete.', flush=True)
