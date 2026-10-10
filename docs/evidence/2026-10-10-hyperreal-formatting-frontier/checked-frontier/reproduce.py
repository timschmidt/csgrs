"""Expand checked-format receipts without changing the frozen runtime candidate."""
from collections import Counter
from decimal import Decimal, localcontext
from fractions import Fraction
import gzip
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import time

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
OUT = EVIDENCE / 'checked-frontier'
WORK = ROOT / 'target/hyperreal-checked-frontier'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
assert read(EVIDENCE / 'checked-scientific-focused.json')['decision'].startswith('focused checked scientific candidate passes;')
source = read(EVIDENCE / 'checked-scientific/sources.json')
hash_file = Path(source['source_hashes_file'])
assert digest(hash_file) == source['source_hashes_file_sha256']
hashes = json.loads(gzip.decompress(hash_file.read_bytes()))
native = read(Path(source['main_source_record']))['source_hashes']
OUT.mkdir()
WORK.mkdir()
shutil.copyfile(EVIDENCE / 'checked_frontier_probe.rs', OUT / 'probe.rs')
shutil.copyfile(__file__, OUT / 'reproduce.py')
probe_hash = digest(OUT / 'probe.rs')
env = os.environ | {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache', 'CARGO_NET_OFFLINE': 'true', 'CARGO_TARGET_DIR': source['target_directory']}
plan = {'scope': 'Checked API receipt corpus, not a timing benchmark.', 'matrix': {'families': 18, 'stages': ['fresh objects', 'prepared -128', 'prepared -512', 'clone after preparation -512'], 'decimal_places': [0, 5, 6, 20], 'precision_floors': [0, -16, -64, -128, -256, -512], 'cases': ['Lower', 'Upper'], 'rows': 3456}, 'concurrent': {'shared_values': 5, 'threads': 4, 'preparation_precisions': [-16, -128, -512, -256], 'places': [0, 6, 20], 'floors': [-64, -128, -256], 'cases': ['Lower', 'Upper'], 'rows': 360}, 'processes': ['forward family order', 'reverse family order'], 'source_record': str(EVIDENCE / 'checked-scientific/sources.json'), 'source_record_sha256': digest(EVIDENCE / 'checked-scientific/sources.json'), 'probe_sha256': probe_hash, 'baseline_comparison': 'Only the 48 original lower/upper scientific rows are directly comparable. Existing fixed and alternate traits, including six alternate-format cutoffs, remain outside this checked-API comparison.'}
(OUT / 'plan.json').write_text(json.dumps(plan, indent=2) + '\n')
commands = []


def verify():
    assert all(digest(Path(source['source_directory']) / name) == value for name, value in hashes.items())
    assert all(digest(ROOT / name) == value for name, value in native.items())
    assert digest(OUT / 'probe.rs') == probe_hash


def run(name, command, cwd=ROOT, timeout=180):
    verify()
    print('start', name, flush=True)
    start = time.monotonic()
    with (OUT / (name + '.stdout')).open('w') as stdout, (OUT / (name + '.stderr')).open('w') as stderr:
        result = subprocess.run(['timeout', '--signal=TERM', '--kill-after=5s', str(timeout), *command], cwd=cwd, env=env, stdout=stdout, stderr=stderr, check=False)
    commands.append({'name': name, 'command': command, 'exit_code': result.returncode, 'wall_seconds': time.monotonic() - start})
    (OUT / 'commands.json').write_text(json.dumps(commands, indent=2) + '\n')
    verify()
    print('finish', name, result.returncode, flush=True)
    assert result.returncode == 0, name
    return (OUT / (name + '.stdout')).read_text()


run('format', ['rustfmt', '--edition', '2024', '--check', str(OUT / 'probe.rs')])
build = run('library-build', ['cargo', 'build', '--locked', '--release', '-p', 'hyperreal', '--message-format=json'], cwd=Path(source['source_directory']))
artifacts = [json.loads(line) for line in build.splitlines() if line.startswith('{')]
libraries = [path for item in artifacts if item.get('reason') == 'compiler-artifact' and item['target']['name'] == 'hyperreal' for path in item['filenames'] if path.endswith('.rlib')]
assert len(libraries) == 1
library = Path(libraries[0])
library_hash = digest(library)
retained = WORK / 'libhyperreal_checked.rlib'
shutil.copyfile(library, retained)
(OUT / 'library.json').write_text(json.dumps({'path': str(library), 'retained_path': str(retained), 'sha256': library_hash, 'bytes': library.stat().st_size}, indent=2) + '\n')
run('probe-build', ['rustc', '--edition=2024', '-O', '-D', 'warnings', str(OUT / 'probe.rs'), '-L', 'dependency=' + str(Path(source['target_directory']) / 'release/deps'), '--extern', 'hyperreal=' + str(retained), '-o', str(WORK / 'probe')])
binary_hash = digest(WORK / 'probe')
(OUT / 'binary.json').write_text(json.dumps({'path': str(WORK / 'probe'), 'sha256': binary_hash, 'bytes': (WORK / 'probe').stat().st_size}, indent=2) + '\n')
trials = []
key = lambda row: (row['family'], row['stage'], row['places'], row['floor'], row['case'])
for order in ['forward', 'reverse']:
    raw = run(order, [str(WORK / 'probe'), order])
    rows = sorted([json.loads(line) for line in raw.splitlines()], key=key)
    assert len(rows) == 3816 and len({key(row) for row in rows}) == len(rows)
    trials.append(rows)
assert trials[0] == trials[1], 'retain both complete processes and inspect any receipt difference'
rows = trials[0]
canonical = (json.dumps(rows, sort_keys=True, separators=(',', ':')) + '\n').encode()
(OUT / 'canonical-receipts.json.gz').write_bytes(gzip.compress(canonical, mtime=0))


def atan_bounds(inverse, terms=40):
    total = sum((Fraction((-1) ** index, (2 * index + 1) * inverse ** (2 * index + 1)) for index in range(terms)), Fraction())
    return total, total + Fraction(1, (2 * terms + 1) * inverse ** (2 * terms + 1))


def sqrt_bounds(integer):
    scale = 10 ** 40
    lower = math.isqrt(integer * scale * scale)
    return Fraction(lower, scale), Fraction(lower + 1, scale)


a, b = atan_bounds(5), atan_bounds(239)
pi = 16 * a[0] - 4 * b[1], 16 * a[1] - 4 * b[0]
root2, root3 = sqrt_bounds(2), sqrt_bounds(3)
e_lower = sum((Fraction(1, math.factorial(index)) for index in range(61)), Fraction())
e_bounds = e_lower, e_lower + Fraction(1, 60 * math.factorial(60))
x = Fraction(1, 3)
ln_lower = 2 * sum((x ** (2 * index + 1) / (2 * index + 1) for index in range(60)), Fraction())
ln_bounds = ln_lower, ln_lower + 2 * x ** 121 / (121 * (1 - x * x))
exact = {'zero': Fraction(0), 'rational': Fraction(7, 8), 'tiny': Fraction(1, 10 ** 400), 'negative_tiny': Fraction(-1, 10 ** 400), 'huge': Fraction(10 ** 400), 'unresolved_zero': Fraction(0), 'unresolved_delta': Fraction(0), 'original_hypot': Fraction(0), 'positive_offset': Fraction(1, 10 ** 40), 'negative_offset': Fraction(-1, 10 ** 40), 'rounding_tie': Fraction(246913, 200000)}
bounds = {name: (value, value) for name, value in exact.items()} | {'pi': pi, 'sqrt_two': root2, 'pi_scaled_huge': tuple(value * 10 ** 400 for value in pi), 'pi_scaled_tiny': tuple(value / 10 ** 400 for value in pi), 'e': e_bounds, 'ln_two': ln_bounds, 'radical_sum': (root2[0] + root3[0], root2[1] + root3[1])}


def reference(value, places, case):
    with localcontext() as context:
        context.prec = 900
        number = Decimal(value.numerator) / Decimal(value.denominator)
        exponent = number.adjusted() if number else 0
        rounded = Decimal(format(number.scaleb(-exponent), f'.{places}f'))
        if abs(rounded) >= 10:
            rounded = rounded / 10
            exponent += 1
        return format(rounded, f'.{places}f') + ('e' if case == 'Lower' else 'E') + str(exponent)


oracle = {}
for row in rows:
    name = (row['family'], row['places'], row['case'])
    if name not in oracle:
        outputs = [reference(value, row['places'], row['case']) for value in bounds[row['family']]]
        assert outputs[0] == outputs[1], name
        oracle[name] = outputs[0]
    if row['result']['kind'] == 'ok':
        assert row['result']['text'] == oracle[name], row
    else:
        assert row['result']['problem'] in ['UnknownZero', 'Exhausted'], row
(OUT / 'reference-outputs.json').write_text(json.dumps([{'family': name[0], 'places': name[1], 'case': name[2], 'text': value} for name, value in sorted(oracle.items())], indent=2) + '\n')
lookup = {key(row): row['result'] for row in rows}
cache_differences = []
for row in rows:
    if row['stage'] != 'fresh':
        fresh = lookup.get((row['family'], 'fresh', row['places'], row['floor'], row['case']))
        assert fresh is not None
        if fresh != row['result']:
            cache_differences.append({'row': row, 'fresh_result': fresh})
baseline = read(EVIDENCE / 'baseline/trials.json')
comparison = []
for row in baseline:
    if row['style'] not in ['lower', 'upper']:
        continue
    stage = 'fresh' if row['preparation'] == 'fresh' else 'prepared_128'
    result = lookup[(row['family'], stage, 6, -512, row['style'].capitalize())]
    if row['exit_code'] == 0:
        assert result == {'kind': 'ok', 'text': row['output']}
    else:
        assert row['timed_out'] and result == {'kind': 'error', 'problem': 'UnknownZero'}
    comparison.append({'family': row['family'], 'stage': stage, 'style': row['style'], 'legacy_timed_out': row['timed_out'], 'checked_result': result})
assert len(comparison) == 48
(OUT / 'legacy-scientific-comparison.json').write_text(json.dumps(comparison, indent=2) + '\n')
(OUT / 'cache-differences.json').write_text(json.dumps(cache_differences, indent=2) + '\n')
verify()
assert digest(library) == digest(retained) == library_hash and digest(WORK / 'probe') == binary_hash
record = {'decision': 'expanded checked scientific corpus complete; inspect cache receipts and resources before full validation and runtime admission', 'rows_each_process': 3816, 'processes': 2, 'all_rows_repeat_in_reverse_family_order': True, 'outcomes_each_process': dict(Counter(row['result'].get('problem', 'ok') for row in rows)), 'all_returned_strings_match_independent_reference': True, 'reference_basis': 'Exact rational values and trigonometric zero identity; rational Machin-series brackets for pi, factorial-series brackets for e, atanh-series brackets for ln(2), integer-square-root brackets for sqrt(2)/sqrt(3), and exact rational scaling/summing of those brackets. Both reference endpoints round identically at each requested precision.', 'preparation_or_concurrency_receipt_differences_from_fresh': len(cache_differences), 'legacy_scientific_completed_outputs_preserved': sum(not row['legacy_timed_out'] for row in comparison), 'legacy_scientific_cutoffs_now_typed_unknown_zero': sum(row['legacy_timed_out'] for row in comparison), 'source_and_binary_hashes_unchanged': True, 'main_runtime_unchanged': True, 'limitations': 'This covers explicit checked scientific formatting only. The original fixed/alternate formatting traits remain unchanged; six alternate-format cutoffs are not repaired by this comparison. Finite receipts do not prove universal cache-frontier equivalence, arbitrary representation coverage or total-work bounds. Concurrency uses immutable shared values with cache preparation and no concurrent abort-flag mutation. This is not a matched timing/resource gate or runtime admission.'}
record['artifacts'] = {str(path.relative_to(EVIDENCE)): {'bytes': path.stat().st_size, 'sha256': digest(path)} for path in sorted(OUT.iterdir()) if path.is_file()}
record['scripts'] = {name: {'bytes': (EVIDENCE / name).stat().st_size, 'sha256': digest(EVIDENCE / name)} for name in ['checked_frontier_probe.rs', 'reproduce_checked_frontier.py']}
(EVIDENCE / 'checked-frontier.json').write_text(json.dumps(record, indent=2) + '\n')
print('Checked frontier:', record['outcomes_each_process'], '; cache/concurrency differences:', len(cache_differences), '; legacy outputs/cutoffs:', record['legacy_scientific_completed_outputs_preserved'], record['legacy_scientific_cutoffs_now_typed_unknown_zero'], flush=True)
