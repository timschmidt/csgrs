"""Exact independent unit-circle membership oracle and deterministic Rust cases."""
from fractions import Fraction as F
import hashlib
import json
from pathlib import Path
import shutil

EVIDENCE = Path(__file__).resolve().parent
OUT = EVIDENCE / 'arc-endpoint-frontier'
OUT.mkdir()
start = (F(1), F(0))
ends = [(F(0), F(1)), (F(-1), F(0)), (F(0), F(-1)), start]
points = []
for t in [F(0), F(1, 3), F(1, 2), F(1), F(2), F(3), F(-1, 3), F(-1, 2), F(-1), F(-2), F(-3)]:
    points.append(((1 - t * t) / (1 + t * t), 2 * t / (1 + t * t)))
points.append((F(-1), F(0)))
assert len(set(points)) == 12 and all(x*x + y*y == 1 for x, y in points)
points += [(F(0), F(0)), (F(1, 2), F(0)), (F(2), F(0)), (1 - F(1, 10**40), F(0)), (1 + F(1, 10**40), F(0))]


def at_most_angle(point, end):
    # Exact circular ordering from the positive x axis; no trig or floats.
    half = lambda p: 0 if p[1] > 0 or (p[1] == 0 and p[0] >= 0) else 1
    a, b = half(point), half(end)
    return a < b if a != b else point[0] * end[1] - point[1] * end[0] >= 0


cases = []
for end_index, end in enumerate(ends):
    for clockwise in [False, True]:
        inputs = [(0, point) for point in points] + [(1, start), (2, end)]
        for query, (alias, point) in enumerate(inputs):
            on_circle = point[0]**2 + point[1]**2 == 1
            endpoint = point in [start, end]
            in_sweep = end == start or endpoint or (not at_most_angle(point, end) if clockwise else at_most_angle(point, end))
            cases.append({'id': len(cases), 'end': end_index, 'clockwise': clockwise, 'query': query, 'alias': alias, 'x': str(point[0]), 'y': str(point[1]), 'on_circle': on_circle, 'endpoint': endpoint, 'member': on_circle and in_sweep})
assert len(cases) == 152
assert all(row['member'] for row in cases if row['alias'])
format_bool = lambda value: str(value).lower()
rows = []
for row in cases:
    rows.append('    Case { id: %d, end: %d, clockwise: %s, alias: %d, x: "%s", y: "%s", member: %s, endpoint: %s, on_circle: %s },' % (row['id'], row['end'], format_bool(row['clockwise']), row['alias'], row['x'], row['y'], format_bool(row['member']), format_bool(row['endpoint']), format_bool(row['on_circle'])))
rust = OUT / 'arc_endpoint_cases.rs'
rust.write_text('const CASES: &[Case] = &[\n' + '\n'.join(rows) + '\n];\n')
record = {
    'scope': '152 exact-rational source-circle cases; similarity transport supplies expectations for seven coordinate representations and reversal. No floating-point oracle or baseline implementation is used.',
    'cases': cases,
    'similarity_argument': 'Common translation and a nonzero uniform scale preserve circle incidence and membership. Both rotations have norm one: (3/5)^2+(4/5)^2=1 and (sqrt(2)/2)^2+(sqrt(2)/2)^2=1. Negative uniform scaling also has positive determinant. Reversing endpoints and direction preserves the same arc set.',
    'representations': ['unit rational', 'rational translation/rotation and negative scale', 'huge rational translation and scale', 'tiny rational scale', 'radical translation/rotation and scale', 'pi/e translation and pi scale', 'sin(e)/cos(e) translation'],
    'membership_expectations_after_similarity_and_reversal': len(cases) * 7 * 2,
    'rust_cases_sha256': hashlib.sha256(rust.read_bytes()).hexdigest(),
}
(OUT / 'oracle.json').write_text(json.dumps(record, indent=2) + '\n')
shutil.copyfile(__file__, OUT / 'oracle.py')
print('Generated', len(cases), 'independent rational cases and', len(cases)*7*2, 'transported membership expectations.', flush=True)
