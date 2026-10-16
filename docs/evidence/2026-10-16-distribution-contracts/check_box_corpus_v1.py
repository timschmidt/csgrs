"""Independent exact face-coverage oracle for the retained rational box corpus.

Every component must cover all six unit-square faces with outward triangles,
balanced directed edges, and exact area one per face. Expected translations
come from the documented fixture, not a second call to the modeling kernel.
Duplicate concatenation denotes two positive winding copies, not set volume two.
"""
from collections import Counter, defaultdict
from fractions import Fraction
from itertools import product
import json
from pathlib import Path

E = Path(__file__).resolve().parent
OUT = E / 'resources-v1/measurements'


def rational(value):
    assert value['class'] == 'One' and value['computable'] is None
    q = value['rational']
    integer = lambda limbs: sum(v << (32 * i) for i, v in enumerate(limbs))
    return Fraction(q['sign'] * integer(q['numerator']), integer(q['denominator']))


def check(path, origins):
    data = json.loads(path.read_text())
    points = [tuple(map(rational, p)) for p in data['positions']]
    triangles = data['triangles']
    assert len(points) == 8 * len(origins) and len(triangles) == 12 * len(origins)
    for i, origin in enumerate(origins):
        expected = {tuple(o + d for o, d in zip(origin, offset))
                    for offset in product([0, 1], repeat=3)}
        assert set(points[8*i:8*i+8]) == expected
        edges = Counter()
        area_twice = defaultdict(Fraction)
        for triangle in triangles[12*i:12*i+12]:
            assert len(set(triangle)) == 3 and all(8*i <= j < 8*i+8 for j in triangle)
            a, b, c = [points[j] for j in triangle]
            u, v = [tuple(x-y for x, y in zip(p, a)) for p in [b, c]]
            cross = (u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0])
            faces = [(axis, side) for axis in range(3) for side in [0, 1]
                     if a[axis] == b[axis] == c[axis] == origin[axis] + side]
            assert len(faces) == 1
            axis, side = faces[0]
            assert all(cross[j] == 0 for j in range(3) if j != axis)
            outward_area = cross[axis] * (2*side-1)
            assert outward_area > 0
            area_twice[(axis, side)] += outward_area
            for start, end in zip(triangle, triangle[1:] + triangle[:1]):
                edges[(start, end)] += 1
        assert all(edges[(b, a)] == count == 1 for (a, b), count in edges.items())
        assert area_twice == {(axis, side): 2 for axis in range(3) for side in [0, 1]}


fixtures = {
    'linear-8': [(2*i, 0, 0) for i in range(8)],
    'grid-4x4': [(2*column, 2*row, 0) for row in range(4) for column in range(4)],
    'feature-merge-22': [(3*(i % 6), 3*(i // 6), 0) for i in range(22)],
    'merge-duplicate': [(0, 0, 0), (0, 0, 0)],
}
rows = []
for name, origins in fixtures.items():
    for variant in ['baseline', 'candidate']:
        path = OUT / f'timing-cold-0-{name}-{variant}.geometry.json'
        check(path, origins)
        rows.append({'case': name, 'variant': variant, 'components': len(origins),
                     'exact_unit_face_coverage': True,
                     'interpretation': 'positive winding multiplicity' if name == 'merge-duplicate' else 'disjoint unit-box union'})
destination = OUT / 'independent-box-semantics.json'
assert not destination.exists()
destination.write_text(json.dumps(rows, indent=2) + '\n')
print(json.dumps(rows), flush=True)
