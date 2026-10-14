"""Independent exact half-space checks for every returned face and input point.

The native library is never called. Integer cyclotomic polynomials prove zero;
outward-rounded rational Taylor enclosures prove strict signs. No float sign is
used, and an inconclusive interval is never interpreted as zero.
"""
from fractions import Fraction as Q
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import re

E = Path(__file__).resolve().parent
OUT = E / 'sphere-native-support-v1'
BITS = 96
SCALE = 1 << BITS
ATAN_TERMS = 80
COS_TERMS = 40
CASES = [('fifteen_degrees', 24, 12), ('original_medium', 32, 16),
         ('fifth_turn', 10, 7), ('seventh_turn', 7, 5)]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def dump(path, value):
    path.write_text(json.dumps(value, indent=2) + '\n')


def floor(value):
    return value.numerator // value.denominator


def ceil(value):
    return -floor(-value)


def atan_interval(denominator):
    # Alternating series: the next term bounds the signed remainder.
    value = sum((Q((-1) ** k, (2 * k + 1) * denominator ** (2 * k + 1))
                 for k in range(ATAN_TERMS)), Q(0))
    next_term = Q((-1) ** ATAN_TERMS,
                  (2 * ATAN_TERMS + 1) * denominator ** (2 * ATAN_TERMS + 1))
    return min(value, value + next_term), max(value, value + next_term)


def pi_interval():
    # atan(1/2) + atan(1/3) = atan(1) = pi/4. The addition formula has
    # positive inputs with product 1/6 < 1, hence no branch correction.
    a, b = atan_interval(2), atan_interval(3)
    assert (Q(1, 2) + Q(1, 3)) / (1 - Q(1, 6)) == 1
    lo, hi = 4 * (a[0] + b[0]), 4 * (a[1] + b[1])
    scale = 1 << 128
    lo, hi = Q(floor(lo * scale), scale), Q(ceil(hi * scale), scale)
    assert 3 < lo < hi < 4 and hi - lo < Q(1, 1 << 120)
    return lo, hi


def cosine_interval(index, order, pi):
    index %= order
    index = min(index, order - index)
    if index == 0:
        return SCALE, SCALE
    if 2 * index == order:
        return -SCALE, -SCALE
    lo, hi = (Q(2 * index, order) * bound for bound in pi)
    center = (lo + hi) / 2
    # Taylor polynomial through degree 79 (odd coefficients are zero).
    # All real derivatives of cosine have absolute value <= 1, so the
    # Lagrange remainder is at most |center|^80 / 80!. Cosine is also
    # 1-Lipschitz, adding at most half the argument interval width.
    value = sum(((-1) ** k * center ** (2 * k) / math.factorial(2 * k)
                 for k in range(COS_TERMS)), Q(0))
    error = abs(center) ** (2 * COS_TERMS) / math.factorial(2 * COS_TERMS) + (hi - lo) / 2
    return floor((value - error) * SCALE), ceil((value + error) * SCALE)


def evaluate_real(polynomial, cosines):
    lo = hi = 0
    for coefficient, interval in zip(polynomial, cosines):
        left, right = coefficient * interval[0], coefficient * interval[1]
        lo += min(left, right)
        hi += max(left, right)
    assert len(polynomial) <= len(cosines)
    return lo, hi


def subtract(left, right):
    return left[0] - right[1], left[1] - right[0]


def product(left, right):
    corners = [a * b for a in left for b in right]
    return min(corners), max(corners)


def main():
    assert not OUT.exists()
    assessment = E / 'sphere-cyclotomic-assessment-v2.json'
    assert json.loads(assessment.read_text())['complete']
    inputs = {str(p.relative_to(E)): sha(p) for p in [Path(__file__), assessment,
        E / 'check_sphere_trace_cyclotomic_v1.py', E / 'sphere-native-support-scope-v1.json',
        *[E / f'sphere-cyclotomic-v2/candidate-{case}.stdout' for case, _, _ in CASES]]}
    spec = importlib.util.spec_from_file_location('support_field', E / 'check_sphere_trace_cyclotomic_v1.py')
    oracle = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(oracle)
    OUT.mkdir()
    pi = pi_interval()
    dump(OUT / 'protocol.json', {'inputs_sha256': inputs, 'interval_bits': BITS,
        'atan_terms': ATAN_TERMS, 'cos_terms': COS_TERMS,
        'pi_interval': [[bound.numerator, bound.denominator] for bound in pi],
        'complete_face_point_pairs_expected': 612288, 'runtime_admitted': False})
    cases = []
    for case, segments, stacks in CASES:
        field = oracle.Field(segments, stacks)
        points = [field.point(i) for i in range(segments * (stacks - 1) + 2)]
        cosines = [cosine_interval(k, field.order, pi) for k in range(len(field.modulus) - 1)]
        intervals = [[evaluate_real(coordinate, cosines) for coordinate in point] for point in points]
        raw = (E / f'sphere-cyclotomic-v2/candidate-{case}.stdout').read_text()
        order = json.loads(next(line.split('=', 1)[1] for line in raw.splitlines() if line.startswith('position_order=')))
        assert sorted(order) == list(range(len(points)))
        rows = [tuple(order[int(i)] for i in row) for row in re.findall(r'Triangle \{ v0: (\d+), v1: (\d+), v2: (\d+) \}', raw)]
        faces = []
        totals = {'strictly_inside': 0, 'coplanar': 0}
        for face_index, (a, b, c) in enumerate(rows):
            ab = [field.sub(x, y) for x, y in zip(points[b], points[a])]
            ac = [field.sub(x, y) for x, y in zip(points[c], points[a])]
            normal = [field.sub(field.mul(ab[i], ac[j]), field.mul(ab[j], ac[i]))
                      for i, j in [(1, 2), (2, 0), (0, 1)]]
            assert any(normal), (case, face_index, 'degenerate')
            normal_intervals = [evaluate_real(coordinate, cosines) for coordinate in normal]
            # Independent negative control: a + normal is strictly outside
            # the oriented plane since normal dot normal is positive.
            assert sum(product(axis, axis)[0] for axis in normal_intervals) > 0
            zero_indices = []
            largest_negative_upper = None
            for index, point in enumerate(points):
                lo = hi = 0
                for axis in range(3):
                    lower, upper = product(normal_intervals[axis], subtract(intervals[index][axis], intervals[a][axis]))
                    lo += lower
                    hi += upper
                assert lo <= hi
                if hi < 0:
                    totals['strictly_inside'] += 1
                    largest_negative_upper = hi if largest_negative_upper is None else max(hi, largest_negative_upper)
                elif lo > 0:
                    raise AssertionError((case, face_index, index, 'outside', lo, hi))
                else:
                    remainder = field.orient(points[a], points[b], points[c], point)
                    assert not remainder, (case, face_index, index, 'unresolved_nonzero', remainder)
                    totals['coplanar'] += 1
                    zero_indices.append(index)
            assert set([a, b, c]).issubset(zero_indices) and largest_negative_upper is not None
            faces.append({'indices': [a, b, c], 'anchor_normal': normal,
                          'coplanar_original_indices': zero_indices,
                          'largest_strict_upper_numerator_over_2_pow_192': largest_negative_upper})
        assert sum(totals.values()) == len(rows) * len(points)
        dump(OUT / (case + '.json'), {'case': case, 'root_of_unity_order': field.order,
            'cosine_enclosures_over_2_pow_96': cosines, 'faces': faces, 'totals': totals})
        row = {'case': case, 'points': len(points), 'triangles': len(rows),
               'face_point_pairs': len(rows) * len(points), **totals,
               'all_faces_nondegenerate_and_supporting': True, 'all_face_orientations_outward': True}
        cases.append(row)
        print(json.dumps(row), flush=True)
    assert sum(row['face_point_pairs'] for row in cases) == 612288
    assert all(sha(E / p) == h for p, h in inputs.items())
    dump(OUT / 'result.json', {'complete': True, 'cases': cases, 'all_inputs_unchanged': True,
        'artifacts_sha256': {p.name: sha(p) for p in sorted(OUT.iterdir()) if p.is_file()},
        'claim': 'Every returned triangle is nondegenerate and its outward plane contains or excludes every unchanged original point on the interior side. Combined with the independently checked closed oriented output rows and complete original vertex set, this validates these four convex hull outputs.',
        'limits': 'Finite corpus verification with exact integer/rational Python arithmetic, not formal verification of the native implementation.',
        'runtime_admitted': False, 'plan_complete': False})
    print(json.dumps({'complete': True, 'face_point_pairs': 612288, 'result_sha256': sha(OUT / 'result.json')}), flush=True)


if __name__ == '__main__':
    main()
