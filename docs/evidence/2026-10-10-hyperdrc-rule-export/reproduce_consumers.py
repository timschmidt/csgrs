"""Audit checked exports with independent fractions and the installed KiCad CLI."""
from fractions import Fraction
import gzip
import hashlib
import json
import os
from pathlib import Path
import random
import re
import resource
import shutil
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
CANDIDATE = Path(sys.argv[1]).resolve()
OUT = CANDIDATE / 'consumers'
OUT.mkdir()
record = json.loads((CANDIDATE / 'sources.json').read_text())
validation = json.loads((CANDIDATE / 'validation.json').read_text())
SOURCE = Path(record['source_directory'])
TARGET = Path(record['target_directory'])
WORK = ROOT / ('target/hyperdrc-rule-export-consumers-' + CANDIDATE.name)
WORK.mkdir()
digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
info = lambda p: {'bytes': p.stat().st_size, 'sha256': digest(p)}
maps = [(directory, json.loads(gzip.decompress(Path(record[kind + '_hashes_file']).read_bytes()))) for kind, directory in [('source', SOURCE), ('main', ROOT)]]
library = TARGET / 'release/libhyperdrc.rlib'
library_info = validation['libraries'][str(library)]
assert info(library) == library_info
linked = WORK / 'libhyperdrc_checked.rlib'
shutil.copyfile(library, linked)
for original, name in [(Path(__file__), 'reproduce.py'), (EVIDENCE / 'checked_probe.rs', 'probe.rs'), (EVIDENCE / 'consumer-board.kicad_pcb', 'board.kicad_pcb')]:
    shutil.copyfile(original, OUT / name)
frozen = {name: digest(OUT / name) for name in ['reproduce.py', 'probe.rs', 'board.kicad_pcb']}
commands, exports, merges, consumers = [], [], [], []


def verify():
    assert all(digest(root / name) == value for root, values in maps for name, value in values.items())
    assert info(library) == library_info == info(linked)
    assert all(digest(OUT / name) == value for name, value in frozen.items())


def limit_probe():
    resource.setrlimit(resource.RLIMIT_AS, (512 * 1024 * 1024, 512 * 1024 * 1024))


def run(name, command, seconds=60, probe=False, env=None):
    verify()
    print('start', name, flush=True)
    start = time.monotonic()
    with (OUT / (name + '.stdout')).open('w') as stdout, (OUT / (name + '.stderr')).open('w') as stderr:
        result = subprocess.run(['timeout', '--signal=TERM', '--kill-after=2s', str(seconds), *map(str, command)], cwd=ROOT, env=env, stdout=stdout, stderr=stderr, preexec_fn=limit_probe if probe else None, check=False)
    commands.append({'name': name, 'command': list(map(str, command)), 'exit_code': result.returncode, 'wall_seconds': time.monotonic() - start, 'timeout_seconds': seconds, 'probe_address_space_cap_mib': 512 if probe else None})
    (OUT / 'commands.json').write_text(json.dumps(commands, indent=2) + '\n')
    verify()
    print('finish', name, result.returncode, flush=True)
    return result.returncode


binary = WORK / 'probe'
assert run('format', ['rustfmt', '--edition', '2024', '--check', OUT / 'probe.rs']) == 0
assert run('build', ['rustc', '--edition=2024', '-O', '-D', 'warnings', OUT / 'probe.rs', '-L', 'dependency=' + str(TARGET / 'release/deps'), '--extern', 'hyperdrc=' + str(linked), '-o', binary]) == 0
baseline = json.loads((EVIDENCE / 'baseline.json').read_text())
families = list(dict.fromkeys(row['family'] for row in baseline['exports'])) + ['unresolved_tie', '0.0000005', '0.1234565', '0.9999995']
expected_errors = {'1e-7': 'BelowResolution', '1e-40': 'BelowResolution', '1e-400': 'BelowResolution', '1e20': 'OutOfRange', '1e400': 'OutOfRange', '2147.483648': 'OutOfRange', 'unresolved_zero': 'Uncertified', 'unresolved_tiny': 'BelowResolution', 'unresolved_tie': 'Uncertified', 'aborted_pi': 'Aborted'}
for index, family in enumerate(families):
    previous = None
    for preparation in ['fresh', 'prepared']:
        name = f'export-{index:02}-{preparation}'
        destination = OUT / (name + '.kicad_dru')
        sentinel = b'preserve existing destination on rejection\n'
        destination.write_bytes(sentinel)
        code = run(name, [binary, family, preparation, 'render', destination], seconds=3, probe=True)
        if family in expected_errors:
            error = expected_errors[family]
            assert code == 2 and error in (OUT / (name + '.stderr')).read_text(), name
            assert destination.read_bytes() == sentinel, name
            text = None
        else:
            assert code == 0, name
            text = destination.read_text()
            assert previous is None or text == previous, name
            previous = text
        exports.append({'family': family, 'preparation': preparation, 'exit_code': code, 'typed_error': expected_errors.get(family), 'destination_unchanged_on_error': code == 2 and destination.read_bytes() == sentinel, 'output_sha256': digest(destination)})
(OUT / 'exports.json').write_text(json.dumps(exports, indent=2) + '\n')

# Independently derive nanometer rounding for exact fractions, including many
# ties and their neighbors. No floating-point arithmetic is used by the oracle.
rng = random.Random(0xD2C)
tokens = {'0', '-0.1', '1/3', '1e-400', '1e400', '2147.483647', '2147.483648'}
for _ in range(128):
    integer = rng.randrange(0, 2_147_483_647)
    for delta in [-1, 0, 1]:
        value = Fraction(2 * integer + 1, 2_000_000) + Fraction(delta, 10**24)
        tokens.add(str(value))
    tokens.add(str(Fraction(rng.randrange(-10**10, 10**10), rng.randrange(1, 10**9))))
tokens = sorted(tokens)
expected = {}
maximum = Fraction(2_147_483_647, 1_000_000)
for token in tokens:
    value = Fraction(token)
    if value <= 0:
        expected[token] = ('Ok', 'omitted')
    elif value > maximum:
        expected[token] = ('Err', 'OutOfRange')
    else:
        shifted = value * 1_000_000 + Fraction(1, 2)
        nm = shifted.numerator // shifted.denominator
        expected[token] = ('Ok', Fraction(nm, 1_000_000)) if nm else ('Err', 'BelowResolution')
matrix_rows = []
for mode, rows in [('fresh', tokens), ('prepared', list(reversed(tokens)))]:
    input_file = OUT / ('matrix-' + mode + '.txt')
    input_file.write_text('\n'.join(rows) + '\n')
    assert run('matrix-' + mode, [binary, 'matrix', mode, input_file], seconds=10, probe=True) == 0
    actual = {}
    for line in (OUT / ('matrix-' + mode + '.stdout')).read_text().splitlines():
        token, status, result = line.split('\t')
        expected_status, expected_value = expected[token]
        assert status == expected_status, line
        if status == 'Err':
            assert expected_value in result, line
        elif result == 'omitted':
            assert expected_value == 'omitted', line
        else:
            assert re.fullmatch(r'\d+(?:\.\d{1,6})?', result), line
            assert Fraction(result) == expected_value, line
            assert abs(Fraction(result) - Fraction(token)) <= Fraction(1, 2_000_000), line
        actual[token] = (status, result)
    assert len(actual) == len(tokens)
    if matrix_rows:
        assert actual == matrix_rows[0]
    matrix_rows.append(actual)
(OUT / 'matrix-oracle.json').write_text(json.dumps({'rows_per_process': len(tokens), 'processes': 2, 'identical_receipts_in_reverse_order_and_prepared_state': True, 'rounding': 'exact Fraction nearest nanometer, ties upward; maximum half-nanometer error; no scientific output'}, indent=2) + '\n')

merge_inputs = {row['case']: (EVIDENCE / row['original']).read_text() for row in baseline['merges']}
merge_inputs['legacy-generated-deck'] = (EVIDENCE / 'baseline/export-00-fresh.kicad_dru').read_text()
for name, original in merge_inputs.items():
    source = OUT / ('merge-input-' + name + '.kicad_dru')
    source.write_text(original)
    destination = OUT / ('merge-' + name + '.kicad_dru')
    sentinel = b'preserve existing merge destination\n'
    destination.write_bytes(sentinel)
    code = run('merge-' + name, [binary, 'default', 'fresh', source, destination], seconds=3, probe=True)
    if name == 'unterminated-owned-block':
        assert code == 2 and destination.read_bytes() == sentinel
        assert 'InvalidDeck' in (OUT / ('merge-' + name + '.stderr')).read_text()
    else:
        assert code == 0
        output = destination.read_text()
        assert output.count('(version 1)') == 1
        assert not any(line.lstrip().startswith(';') for line in output.splitlines())
        for marker in re.findall(r'KEEP-[A-Z-]+', original):
            assert marker in output
        repeat = OUT / ('merge-repeat-' + name + '.kicad_dru')
        assert run('merge-repeat-' + name, [binary, 'default', 'fresh', destination, repeat], seconds=3, probe=True) == 0
        assert repeat.read_bytes() == destination.read_bytes()
    merges.append({'case': name, 'exit_code': code, 'destination_sha256': digest(destination)})
(OUT / 'merges.json').write_text(json.dumps(merges, indent=2) + '\n')

assert run('kicad-version', ['kicad-cli', '--version']) == 0
env = os.environ | {'KICAD_CONFIG_HOME': str(WORK / 'config'), 'LC_ALL': 'C.UTF-8'}
default = (OUT / 'export-00-fresh.kicad_dru').read_text()
cases = [('default-below', default, '0.149999', True), ('default-at', default, '0.15', False), ('default-above', default, '0.150001', False)]
for family, widths in [('0.1234567', [('0.123456', True), ('0.123457', False)]), ('0.1234565', [('0.123456', True), ('0.123457', False)]), ('2147.483647', [('0.1', True)])]:
    deck = (OUT / f'export-{families.index(family):02}-fresh.kicad_dru').read_text()
    cases.extend((family + '-' + width, deck, width, violation) for width, violation in widths)
for name in merge_inputs:
    if name != 'unterminated-owned-block':
        cases.append(('merged-' + name, (OUT / ('merge-' + name + '.kicad_dru')).read_text(), '0.1', True))
for name, deck, width, violation in cases:
    directory = OUT / ('consumer-' + name)
    directory.mkdir()
    board = directory / 'probe.kicad_pcb'
    board.write_text((OUT / 'board.kicad_pcb').read_text().replace('(width 0.1)', '(width ' + width + ')'))
    (directory / 'probe.kicad_pro').write_text('{}\n')
    (directory / 'probe.kicad_dru').write_text(deck)
    report = directory / 'report.json'
    code = run('consumer-' + name, ['kicad-cli', 'pcb', 'drc', '--format', 'json', '--output', report, board], seconds=30, env=env)
    assert code == 0 and report.exists()
    data = json.loads(report.read_text())
    widths = [item for item in data['violations'] if item['type'] == 'track_width']
    assert bool(widths) == violation, (name, widths)
    assert all(item['severity'] == 'warning' for item in widths), name
    consumers.append({'case': name, 'width_mm': width, 'expected_track_width_violation': violation, 'track_width_violations': widths, 'exit_code': code})
    (OUT / 'consumers.json').write_text(json.dumps(consumers, indent=2) + '\n')
verify()
summary = {'source_record': str(CANDIDATE / 'sources.json'), 'source_record_sha256': digest(CANDIDATE / 'sources.json'), 'validation_sha256': digest(CANDIDATE / 'validation.json'), 'all_source_library_and_probe_hashes_unchanged': True, 'exports': exports, 'merges': merges, 'exact_matrix_rows_per_process': len(tokens), 'exact_matrix_processes': 2, 'consumers': consumers, 'kicad_version': (OUT / 'kicad-version.stdout').read_text().strip(), 'probe_binary': {'path': str(binary), **info(binary)}, 'library': {'path': str(linked), **library_info}, 'limitations': ['KiCad acceptance is checked through track-width enforcement, not CLI exit status alone.', 'Only track-width constraint semantics are covered by these consumer fixtures.', 'KiCad reports include run-specific timestamps and generated board-item UUIDs; full report byte determinism is not claimed.', 'External process cutoffs do not establish a total-work bound for underlying expressions.'], 'commands': commands}
summary['artifacts'] = {str(p.relative_to(OUT)): info(p) for p in sorted(OUT.rglob('*')) if p.is_file()}
(CANDIDATE / 'consumer-validation.json').write_text(json.dumps(summary, indent=2) + '\n')
print('Consumer validation complete:', len(exports), 'exports,', len(tokens), 'exact rows x 2,', len(consumers), 'KiCad consumers.', flush=True)
