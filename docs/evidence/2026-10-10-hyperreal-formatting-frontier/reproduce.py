"""Inventory formatting completion using frozen admitted scalar code, without a patch."""
from decimal import Decimal, localcontext
from fractions import Fraction
import gzip
import hashlib
import json
import math
import os
from pathlib import Path
import resource
import shutil
import subprocess
import time

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
PREVIOUS = EVIDENCE.parent / '2026-10-09-hyperdrc-native-linear-strokes'
OUT = EVIDENCE / 'baseline'
WORK = ROOT / 'target/hyperreal-formatting-frontier'
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
assert read(PREVIOUS / 'diagnostics-84.json')['decision'].startswith('not retained:')
production_path = PREVIOUS / 'arc-endpoint-certificate/baseline-sources.json'
production = read(production_path)
map_path = Path(production['source_hashes_file'])
assert digest(map_path) == production['source_hashes_file_sha256']
source_hashes = json.loads(gzip.decompress(map_path.read_bytes()))
native = read(PREVIOUS / 'native-exactness-gate/sources.json')['source_hashes']
library = read(PREVIOUS / 'arc-endpoint-cost/libraries.json')['baseline']['hyperreal']
retained = Path(library['retained_path'])
assert digest(retained) == library['sha256']
OUT.mkdir()
WORK.mkdir()
linked_library = WORK / 'libhyperreal_baseline.rlib'
shutil.copyfile(retained, linked_library)
shutil.copyfile(EVIDENCE / 'probe.rs', OUT / 'probe.rs')
shutil.copyfile(__file__, OUT / 'reproduce.py')
probe_hash = digest(OUT / 'probe.rs')
families = ['zero', 'rational', 'pi', 'sqrt_two', 'tiny', 'negative_tiny', 'huge', 'unresolved_zero', 'unresolved_delta', 'original_hypot', 'positive_offset', 'negative_offset']
plan = {
    'scope': 'Baseline-only public formatting completion audit. No runtime candidate or benchmark.',
    'families': families, 'preparations': ['fresh', 'prepared'], 'formats': ['fixed', 'alternate', 'lower', 'upper'],
    'precision': 6, 'preparation_precision_bits': -128, 'fresh_processes_per_combination': 1,
    'external_wall_limit_seconds_each': 3, 'external_address_space_limit_bytes_each': 536870912,
    'limits_are_not_library_budgets': True,
    'original_fixture': 'docs/evidence/2026-10-09-hyperdrc-point-consumers/format_stage_probe.rs',
    'production_source_record': str(production_path), 'production_source_record_sha256': digest(production_path),
    'library': library, 'probe_sha256': probe_hash,
    'interpretation': 'A timeout records only failure to complete within the external limit. Prepared means one interval request at -128, not a proven zero/sign or maximum warm cache. Each process constructs fresh query objects; constructors and global constants can already have internal cached facts. Scientific precision is six digits after its leading digit; fixed precision is six fractional digits. No scientific query may be credited as equivalent to a fixed-decimal substitute.',
}
(OUT / 'plan.json').write_text(json.dumps(plan, indent=2) + '\n')


def verify():
    assert all(digest(Path(production['source_directory']) / name) == value for name, value in source_hashes.items())
    assert all(digest(ROOT / name) == value for name, value in native.items())
    assert digest(linked_library) == library['sha256']
    assert digest(OUT / 'probe.rs') == probe_hash


verify()
commands = []
for name, command in [
    ('format', ['rustfmt', '--edition', '2024', '--check', str(OUT / 'probe.rs')]),
    ('build', ['rustc', '--edition=2024', '-O', '-D', 'warnings', str(OUT / 'probe.rs'), '-L', 'dependency=' + str(Path(production['target_directory']) / 'release/deps'), '--extern', 'hyperreal=' + str(linked_library), '-o', str(WORK / 'probe')]),
]:
    with (OUT / (name + '.log')).open('w') as log:
        result = subprocess.run(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, check=False)
    commands.append({'name': name, 'command': command, 'exit_code': result.returncode})
    (OUT / 'commands.json').write_text(json.dumps(commands, indent=2) + '\n')
    assert result.returncode == 0, name
binary_hash = digest(WORK / 'probe')
(OUT / 'binary.json').write_text(json.dumps({'path': str(WORK / 'probe'), 'sha256': binary_hash, 'bytes': (WORK / 'probe').stat().st_size}, indent=2) + '\n')


def limit_address_space():
    resource.setrlimit(resource.RLIMIT_AS, (536870912, 536870912))
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))


trials = []
for family in families:
    for preparation in plan['preparations']:
        for style in plan['formats']:
            name = '-'.join([family, preparation, style])
            command = [str(WORK / 'probe'), family, preparation, style]
            start = time.monotonic()
            timed_out = False
            with (OUT / (name + '.stdout')).open('w') as stdout, (OUT / (name + '.stderr')).open('w') as stderr:
                try:
                    result = subprocess.run(command, cwd=ROOT, stdout=stdout, stderr=stderr, check=False, timeout=3, preexec_fn=limit_address_space)
                    code = result.returncode
                except subprocess.TimeoutExpired:
                    timed_out, code = True, None
            raw = (OUT / (name + '.stdout')).read_text()
            stages = (OUT / (name + '.stderr')).read_text()
            trials.append({'family': family, 'preparation': preparation, 'style': style, 'command': command, 'timed_out': timed_out, 'exit_code': code, 'wall_seconds': time.monotonic() - start, 'output': raw.removesuffix('\n'), 'stages': stages.splitlines()})
            (OUT / 'trials.json').write_text(json.dumps(trials, indent=2) + '\n')
    print('finished', family, flush=True)
verify()
assert digest(WORK / 'probe') == binary_hash

# Independent rational brackets supply reference digits. The Machin identity
# and alternating arctangent series bound pi; integer square roots bound sqrt(2).
def atan_bounds(inverse, terms=40):
    value = sum((Fraction((-1) ** index, (2 * index + 1) * inverse ** (2 * index + 1)) for index in range(terms)), Fraction())
    next_term = Fraction(1, (2 * terms + 1) * inverse ** (2 * terms + 1))
    return value, value + next_term


a, b = atan_bounds(5), atan_bounds(239)
pi_bounds = (16 * a[0] - 4 * b[1], 16 * a[1] - 4 * b[0])
scale = 10 ** 40
sqrt_floor = math.isqrt(2 * scale * scale)
sqrt_bounds = (Fraction(sqrt_floor, scale), Fraction(sqrt_floor + 1, scale))
exact = {'zero': Fraction(0), 'rational': Fraction(7, 8), 'tiny': Fraction(1, 10 ** 400), 'negative_tiny': Fraction(-1, 10 ** 400), 'huge': Fraction(10 ** 400), 'unresolved_zero': Fraction(0), 'unresolved_delta': Fraction(0), 'original_hypot': Fraction(0), 'positive_offset': Fraction(1, 10 ** 40), 'negative_offset': Fraction(-1, 10 ** 40)}


def reference(value, style):
    with localcontext() as context:
        context.prec = 900
        number = Decimal(value.numerator) / Decimal(value.denominator)
        if style == 'alternate':
            style = 'fixed' if Fraction(1, 2 ** 19) <= abs(value) < 2 ** 60 else 'lower'
        if style == 'fixed':
            return format(number, '.6f')
        exponent = number.adjusted() if number else 0
        return format(number.scaleb(-exponent), '.6f') + ('e' if style == 'lower' else 'E') + str(exponent)


for row in trials:
    bounds = pi_bounds if row['family'] == 'pi' else sqrt_bounds if row['family'] == 'sqrt_two' else (exact[row['family']],) * 2
    expected = [reference(value, row['style']) for value in bounds]
    assert expected[0] == expected[1]
    row['reference_output'] = expected[0]
    row['reference_matches_returned_output'] = row['output'] == expected[0] if row['exit_code'] == 0 else None
(OUT / 'trials.json').write_text(json.dumps(trials, indent=2) + '\n')
record = {
    'decision': 'baseline formatting frontier recorded; review outcomes and choose a completion contract before runtime edits',
    'scope': plan, 'trials': len(trials),
    'completed': sum(row['exit_code'] == 0 for row in trials),
    'timed_out': sum(row['timed_out'] for row in trials),
    'other_failures': [row for row in trials if not row['timed_out'] and row['exit_code'] != 0],
    'reference_output_differences': [row for row in trials if row['reference_matches_returned_output'] is False],
    'reference_basis': 'Exact rational values, sin(e)^2 + cos(e)^2 = 1 and hypot(0,0) = 0; rational Machin-series brackets for pi and integer-square-root brackets for sqrt(2). Both bracket endpoints round to the same recorded output. Negative fixed zero signs are retained as explicit presentation differences to review, not automatically declared numeric errors.',
    'main_and_production_source_hashes_unchanged': True,
    'limitations': 'One bounded process per row is an inventory, not a timing comparison or proof of nontermination. Only precision six and one preparation precision are covered. External wall/address-space termination is not a typed library result. Output matching is a presentation check, not an exact-zero certificate. No abort flag, budget-sweep, all-representation coverage, runtime change or full-property completion is claimed.',
}
record['artifacts'] = {str(path.relative_to(EVIDENCE)): {'bytes': path.stat().st_size, 'sha256': digest(path)} for path in sorted(OUT.iterdir()) if path.is_file()}
record['scripts'] = {name: {'bytes': (EVIDENCE / name).stat().st_size, 'sha256': digest(EVIDENCE / name)} for name in ['probe.rs', 'reproduce.py']}
(EVIDENCE / 'baseline.json').write_text(json.dumps(record, indent=2) + '\n')
print('Formatting frontier:', record['completed'], 'completed,', record['timed_out'], 'timed out,', len(record['other_failures']), 'other failures,', len(record['reference_output_differences']), 'presentation differences.', flush=True)
