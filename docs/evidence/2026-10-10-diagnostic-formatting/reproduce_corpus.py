"""Independent decimal oracle, unchanged scientific receipts, and consumer replay.

Run after the focused production build, with no simultaneous build or measurement.
"""
from collections import Counter
from fractions import Fraction
import gzip
import hashlib
import json
import math
from pathlib import Path
import resource
import shutil
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
CANDIDATE = Path(sys.argv[1]).resolve()
OUT = CANDIDATE / 'corpus'
OUT.mkdir()
WORK = ROOT / 'target/diagnostic-formatting-corpus'
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
assert info(library) == read(CANDIDATE / 'focused/validation.json')['libraries'][str(library)]
shutil.copyfile(__file__, OUT / 'reproduce.py')
shutil.copyfile(EVIDENCE / 'decimal_probe.rs', OUT / 'decimal_probe.rs')
shutil.copyfile(EVIDENCE / 'baseline/probe.rs', OUT / 'consumer_probe.rs')
frontier = EVIDENCE.parent / '2026-10-10-hyperreal-formatting-frontier'
shutil.copyfile(frontier / 'checked-frontier/probe.rs', OUT / 'scientific_probe.rs')
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


libraries = {'hyperdrc': library} | {name: dependency(name) for name in ['hyperreal', 'hyperlattice', 'log']}
library_info = {str(p): info(p) for p in libraries.values()}
commands = []


def verify():
    assert all(digest(base / name) == expected for base, values in maps for name, expected in values.items())
    assert all(info(Path(p)) == expected for p, expected in library_info.items())
    assert all(info(p) == expected for p, expected in frozen.items())


def limit():
    resource.setrlimit(resource.RLIMIT_AS, (512 * 1024 * 1024, 512 * 1024 * 1024))


def run(name, command, stdin=None, seconds=120, capped=False):
    start = time.monotonic()
    with (OUT / (name + '.stdout')).open('w') as stdout, (OUT / (name + '.stderr')).open('w') as stderr:
        result = subprocess.run(['timeout', '--signal=TERM', '--kill-after=2s', str(seconds), *map(str, command)], cwd=ROOT, input=stdin, text=True, stdout=stdout, stderr=stderr, preexec_fn=limit if capped else None, check=False)
    commands.append({'name': name, 'command': list(map(str, command)), 'exit_code': result.returncode, 'wall_seconds': time.monotonic() - start, 'timeout_seconds': seconds, 'address_space_cap_mib': 512 if capped else None})
    (OUT / 'commands.json').write_text(json.dumps(commands, indent=2) + '\n')
    assert result.returncode == 0, name
    return (OUT / (name + '.stdout')).read_text()


verify()
for name, deps in [('decimal', ['hyperreal']), ('scientific', ['hyperreal']), ('consumer', ['hyperdrc', 'hyperlattice', 'log'])]:
    source = OUT / (name + '_probe.rs')
    run(name + '-format', ['rustfmt', '--edition', '2024', '--check', source])
    command = ['rustc', '--edition=2024', '-O', '-D', 'warnings', source, '-L', 'dependency=' + str(TARGET / 'release/deps')]
    for dep in deps:
        command += ['--extern', dep + '=' + str(libraries[dep])]
    run(name + '-build', command + ['-o', WORK / name])
    frozen[WORK / name] = info(WORK / name)
print('Three probes built; no further builds during corpus replay.', flush=True)
verify()


def rounded_integer(value):
    integer, remainder = divmod(value.numerator, value.denominator)
    twice = 2 * remainder
    return integer + (twice > value.denominator or (twice == value.denominator and integer % 2 == 1))


def power(exponent):
    return Fraction(10 ** exponent) if exponent >= 0 else Fraction(1, 10 ** -exponent)


def reference(value, places):
    magnitude = abs(value)
    scientific = magnitude < Fraction(1, 2 ** 19) or magnitude >= 2 ** 60
    exponent = 0
    if scientific and magnitude:
        # Exact integer comparisons select the base-ten exponent; no binary floats.
        exponent = len(str(magnitude.numerator)) - len(str(magnitude.denominator))
        if magnitude < power(exponent):
            exponent -= 1
        magnitude /= power(exponent)
    integer = rounded_integer(magnitude * 10 ** places)
    if scientific and integer >= 10 ** (places + 1):
        integer //= 10
        exponent += 1
    digits = str(integer).zfill(places + 1)
    text = digits if places == 0 else digits[:-places] + '.' + digits[-places:]
    if value < 0:
        text = '-' + text
    return text + ('e' + str(exponent) if scientific else '')


def atan(inverse):
    terms = 70
    lower = sum((Fraction((-1) ** i, (2 * i + 1) * inverse ** (2 * i + 1)) for i in range(terms)), Fraction())
    return lower, lower + Fraction(1, (2 * terms + 1) * inverse ** (2 * terms + 1))


a, b = atan(5), atan(239)
pi = 16 * a[0] - 4 * b[1], 16 * a[1] - 4 * b[0]
e = sum((Fraction(1, math.factorial(i)) for i in range(81)), Fraction())
ln = 2 * sum((Fraction(1, (2 * i + 1) * 3 ** (2 * i + 1)) for i in range(100)), Fraction())
scale = 10 ** 60
root2 = math.isqrt(2 * scale * scale)
bounds = {'pi': pi, 'e': (e, e + Fraction(1, 80 * math.factorial(80))), 'ln_two': (ln, ln + Fraction(2, 201 * 3 ** 201) / (1 - Fraction(1, 9))), 'sqrt_two': (Fraction(root2, scale), Fraction(root2 + 1, scale))}
values = {Fraction(0), Fraction(1, 8), Fraction(2, 3), Fraction(1200)}
for boundary in [Fraction(1, 2 ** 19), Fraction(2 ** 60)]:
    for delta in [Fraction(0), Fraction(1, 10 ** 40), Fraction(-1, 10 ** 40)]:
        values.add(boundary + delta)
for exponent in [-400, -40, -20, -7, -6, 18, 19, 40, 400]:
    for coefficient in [Fraction(1), Fraction(5, 4), Fraction(27, 20), Fraction(19999999, 2000000)]:
        values.add(coefficient * power(exponent))
for places in [0, 1, 3, 6, 9, 20, 32]:
    for integer in [0, 1, 2, 1234566, 1234567, 9999999]:
        midpoint = Fraction(2 * integer + 1, 2 * 10 ** places)
        values.update(midpoint + Fraction(delta, 10 ** (places + 8)) for delta in [-1, 0, 1])
values |= {-value for value in values}
rows = []


def add(family, token, places, floor, preparation, interval):
    outputs = [reference(value, places) for value in interval] if interval else None
    assert outputs is None or outputs[0] == outputs[1]
    rows.append({'id': len(rows), 'family': family, 'token': token, 'places': places, 'floor': floor, 'preparation': preparation, 'reference': outputs[0] if outputs else None})


for value in sorted(values):
    for places in [0, 1, 3, 6, 9, 20, 32]:
        for preparation in ['fresh', 'prepared']:
            add('rational', str(value), places, 0, preparation, (value, value))
for family, token in [(name, '0') for name in bounds] + [('scaled_pi', '1e-400'), ('scaled_pi', '1e400'), ('unresolved', '0'), ('unresolved', '1e-40'), ('unresolved', '-1e-40'), ('unresolved', '1.2345665'), ('unresolved', '1/524288'), ('aborted', '0')]:
    interval = bounds.get(family)
    if family == 'scaled_pi':
        interval = tuple(value * Fraction(token) for value in pi)
    elif family == 'unresolved':
        interval = (Fraction(token), Fraction(token))
    for places in [0, 6, 20, 32]:
        for floor in [0, -16, -64, -256, -512]:
            for preparation in ['fresh', 'prepared']:
                add(family, token, places, floor, preparation, interval)
(OUT / 'decimal-inputs.json').write_text(json.dumps(rows, indent=2) + '\n')
encode = lambda selected: ''.join('\t'.join(str(row[key]) for key in ['id', 'family', 'token', 'places', 'floor', 'preparation']) + '\n' for row in selected)
parse = lambda raw: {int(parts[0]): {'kind': parts[1], 'text': parts[2]} for line in raw.splitlines() if (parts := line.split('\t'))}
trials = []
for order in ['forward', 'reverse']:
    data = encode(rows if order == 'forward' else reversed(rows))
    (OUT / ('decimal-' + order + '.stdin')).write_text(data)
    results = parse(run('decimal-' + order, [WORK / 'decimal'], stdin=data, capped=True))
    assert len(results) == len(rows)
    for row in rows:
        result = results[row['id']]
        if result['kind'] == 'ok':
            assert result['text'] == row['reference'], (row, result)
        else:
            assert result['text'] in ['UnknownZero', 'Exhausted']
        if row['family'] == 'rational':
            assert result['kind'] == 'ok'
        if row['family'] == 'aborted':
            assert result == {'kind': 'error', 'text': 'Exhausted'}
        if row['family'] == 'unresolved' and row['token'] == '0':
            assert result == {'kind': 'error', 'text': 'UnknownZero'}
    trials.append(results)
assert trials[0] == trials[1]
cache_changes = []
for first, second in zip(rows[::2], rows[1::2]):
    assert first['preparation'] == 'fresh' and second['preparation'] == 'prepared'
    if trials[0][first['id']] != trials[0][second['id']]:
        cache_changes.append({'fresh': first, 'prepared': second, 'results': [trials[0][first['id']], trials[0][second['id']]]})
(OUT / 'decimal-cache-changes.json').write_text(json.dumps(cache_changes, indent=2) + '\n')
legacy_rows = [row for row in rows if row['family'] == 'rational']
legacy_input = encode(legacy_rows)
(OUT / 'decimal-legacy.stdin').write_text(legacy_input)
legacy = parse(run('decimal-legacy', [WORK / 'decimal', 'legacy'], stdin=legacy_input, capped=True))
assert len(legacy) == len(legacy_rows)
deltas = [{'input': row, 'legacy': legacy[row['id']]['text'], 'checked': trials[0][row['id']]['text']} for row in legacy_rows if legacy[row['id']] != trials[0][row['id']]]
(OUT / 'decimal-legacy-differences.json').write_text(json.dumps(deltas, indent=2) + '\n')
print('Adaptive oracle complete:', len(rows), 'rows/process;', len(deltas), 'legacy spelling/rounding changes.', flush=True)
verify()
scientific_reference = json.loads(gzip.decompress((frontier / 'checked-frontier/canonical-receipts.json.gz').read_bytes()))
key = lambda row: (row['family'], row['stage'], row['places'], row['floor'], row['case'])
for order in ['forward', 'reverse']:
    actual = sorted([json.loads(line) for line in run('scientific-' + order, [WORK / 'scientific', order], capped=True).splitlines()], key=key)
    assert actual == scientific_reference
print('Scientific API replay: 3816 identical receipts in each order.', flush=True)
verify()
consumers = []
for row in read(EVIDENCE / 'baseline/rows.json'):
    name = 'consumer-' + str(len(consumers))
    args = [WORK / 'consumer', row['operation'], row['family'], row['preparation'], row['log_level']]
    stdout = run(name, args, seconds=3, capped=True)
    stderr = (OUT / (name + '.stderr')).read_text()
    if row['operation'] not in ['complex', 'vector']:
        assert stdout == 'findings=0; ids=[]\n'
    if row['family'] in ['unresolved_zero', 'aborted_pi']:
        if row['log_level'] == 'trace' or row['operation'] in ['complex', 'vector']:
            assert ('[UnknownZero]' if row['family'] == 'unresolved_zero' else '[Exhausted]') in stdout + stderr
    consumers.append(row | {'baseline_exit_code': row['exit_code'], 'exit_code': 0, 'baseline_stdout': row['stdout'], 'baseline_stderr': row['stderr'], 'stdout': stdout, 'stderr': stderr, 'wall_seconds': commands[-1]['wall_seconds']})
(OUT / 'consumer-rows.json').write_text(json.dumps(consumers, indent=2) + '\n')
verify()
summary = {'source_record': str(CANDIDATE / 'sources.json'), 'source_record_sha256': digest(CANDIDATE / 'sources.json'), 'source_main_library_and_probe_guards_unchanged': True, 'libraries': library_info, 'adaptive_rows_each_process': len(rows), 'adaptive_processes': 2, 'adaptive_outcomes_each_process': dict(Counter(result['text'] if result['kind'] == 'error' else 'ok' for result in trials[0].values())), 'all_adaptive_ok_strings_match_exact_reference': True, 'adaptive_cache_receipt_changes': len(cache_changes), 'legacy_rational_rows': len(legacy_rows), 'legacy_rational_differences': len(deltas), 'scientific_receipts_each_process': len(scientific_reference), 'scientific_processes': 2, 'scientific_receipts_identical_to_admitted': True, 'consumer_cases': len(consumers), 'consumer_cases_completed': len(consumers), 'baseline_cutoffs_now_complete': sum(row['baseline_exit_code'] == 124 for row in consumers), 'scope': 'Exact Fraction rounding oracle; rational series brackets for pi/e/log and integer-square-root brackets. No universal work bound, performance improvement, or legacy byte-equivalence claim. Legacy differences retained for explicit audit.'}
summary['artifacts'] = {str(p.relative_to(OUT)): info(p) for p in sorted(OUT.rglob('*')) if p.is_file()}
(OUT / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
print('All corpora and 120 isolated consumers complete.', flush=True)
