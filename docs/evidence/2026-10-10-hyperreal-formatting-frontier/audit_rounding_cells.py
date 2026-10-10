"""Audit returned strings against exact rational significant-digit rounding cells."""
from fractions import Fraction
import gzip
import hashlib
import json
import math
from pathlib import Path
import re

EVIDENCE = Path(__file__).resolve().parent
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
source = EVIDENCE / 'checked-frontier/canonical-receipts.json.gz'
record = read(EVIDENCE / 'checked-frontier.json')
assert digest(source) == record['artifacts']['checked-frontier/canonical-receipts.json.gz']['sha256']
rows = json.loads(gzip.decompress(source.read_bytes()))


def atan_bounds(inverse, terms=40):
    lower = sum((Fraction((-1) ** index, (2 * index + 1) * inverse ** (2 * index + 1)) for index in range(terms)), Fraction())
    return lower, lower + Fraction(1, (2 * terms + 1) * inverse ** (2 * terms + 1))


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


def inside_cell(text, places, case, interval):
    marker = 'e' if case == 'Lower' else 'E'
    if text == '0' + ('.' + '0' * places if places else '') + marker + '0':
        return interval == (Fraction(0), Fraction(0))
    pattern = r'(-?)([1-9])' + (r'\.([0-9]{' + str(places) + '})' if places else '()') + marker + r'(0|[1-9][0-9]*|-[1-9][0-9]*)'
    match = re.fullmatch(pattern, text)
    if match is None:
        return False
    sign, leading, trailing, exponent = match.groups()
    lower, upper = interval
    if sign:
        lower, upper = -upper, -lower
    if lower <= 0 or upper < lower:
        return False
    mantissa = int(leading + trailing)
    unit = Fraction(10) ** (int(exponent) - places)
    center = mantissa * unit
    # At a carried decade, values below the result have one extra fractional
    # decimal place before rounding. Their half-cell is unit/20, not unit/2.
    # The carried integer ends in zero and wins an exact lower-bound tie,
    # including places=0 (where the displayed leading digit itself is odd).
    decade = mantissa == 10 ** places
    lower_boundary = center - unit / (20 if decade else 2)
    upper_boundary = center + unit / 2
    lower_closed = decade or mantissa % 2 == 0
    upper_closed = mantissa % 2 == 0
    return (lower > lower_boundary or (lower_closed and lower == lower_boundary)) and (upper < upper_boundary or (upper_closed and upper == upper_boundary))


# Fixed calibration points include both sides of ties, a carried decade, the
# asymmetric zero-place decade cell, signs, zero and a noncanonical spelling.
calibration = [
    ('1.2e0', 1, '1.25', True), ('1.3e0', 1, '1.25', False),
    ('1.4e0', 1, '1.35', True), ('1.3e0', 1, '1.35', False),
    ('1.0e1', 1, '9.95', True), ('9.9e0', 1, '9.95', False),
    ('1e0', 0, '0.95', True), ('1e0', 0, '1.5', False),
    ('2e0', 0, '1.5', True), ('-1e0', 0, '-0.95', True),
    ('0.00e0', 2, '0', True), ('0.00e0', 2, '1e-40', False),
    ('1.0e+0', 1, '1', False), ('1e0', 0, '0.949999', False),
]
for text, places, value, expected in calibration:
    value = Fraction(value)
    assert inside_cell(text, places, 'Lower', (value, value)) == expected
audited = 0
for row in rows:
    if row['result']['kind'] == 'ok':
        assert inside_cell(row['result']['text'], row['places'], row['case'], bounds[row['family']]), row
        audited += 1
summary = {
    'decision': 'every returned checked string contains its entire independent reference enclosure within its exact nearest-even rounding cell',
    'receipts': len(rows), 'returned_strings_audited': audited, 'calibration_cases': len(calibration),
    'method': 'Fraction-only comparison against exact rounding-cell boundaries, including asymmetric decade carries and tie inclusion. No Decimal, floating-point, production division/normalization algorithm or production numeric library is used by this audit.',
    'reference_basis': 'Exact rational fixtures and trigonometric identity; alternating Machin series for pi, factorial series for e, atanh series for ln(2), integer-square-root brackets, and exact rational scales/sums.',
    'source_receipts': str(source), 'source_receipts_sha256': digest(source),
    'script_sha256': digest(Path(__file__)),
    'limitations': 'This independently certifies the retained corpus outputs. It does not prove arbitrary input coverage, refinement completion or typed-error completeness, and it is not a performance measurement.',
}
(EVIDENCE / 'rounding-cell-audit.json').write_text(json.dumps(summary, indent=2) + '\n')
print('Exact rational rounding cells:', audited, 'returned strings pass;', len(calibration), 'calibrations pass.')
