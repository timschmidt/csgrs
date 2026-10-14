"""Independent integer-polynomial check of the three traced sphere determinants."""
from functools import cache
import hashlib
import json
from math import lcm
from pathlib import Path

E = Path(__file__).resolve().parent


def trim(p):
    p = list(p)
    while p and p[-1] == 0:
        p.pop()
    return p


def add(a, b):
    return trim([(a[i] if i < len(a) else 0) + (b[i] if i < len(b) else 0)
                 for i in range(max(len(a), len(b)))])


def neg(a):
    return [-v for v in a]


def multiply(a, b):
    if not a or not b:
        return []
    p = [0] * (len(a) + len(b) - 1)
    for i, x in enumerate(a):
        for j, y in enumerate(b):
            p[i + j] += x * y
    return trim(p)


def divide_monic(a, b):
    assert b and b[-1] == 1
    a = trim(a)
    q = [0] * max(0, len(a) - len(b) + 1)
    while len(a) >= len(b):
        shift, coefficient = len(a) - len(b), a[-1]
        q[shift] = coefficient
        for i, x in enumerate(b):
            a[shift + i] -= coefficient * x
        a = trim(a)
    return trim(q), a


@cache
def cyclotomic(n):
    p = [-1] + [0] * (n - 1) + [1]
    for divisor in range(1, n):
        if n % divisor == 0:
            p, remainder = divide_monic(p, cyclotomic(divisor))
            assert not remainder
    return p


class Field:
    def __init__(self, segments, stacks):
        self.order = lcm(segments, 2 * stacks, 4)
        self.modulus = cyclotomic(self.order)
        self.segments, self.stacks = segments, stacks

    def reduce(self, p):
        return divide_monic(p, self.modulus)[1]

    def mul(self, a, b):
        return self.reduce(multiply(a, b))

    def sub(self, a, b):
        return add(a, neg(b))

    def root(self, k):
        return self.reduce([0] * (k % self.order) + [1])

    def sine_twice(self, k):
        # 2*sin(t) = -i*(exp(it)-exp(-it)), i = zeta^(order/4).
        return neg(self.mul(self.root(self.order // 4), self.sub(self.root(k), self.root(-k))))

    def cosine_twice(self, k):
        return add(self.root(k), self.root(-k))

    def point(self, index):
        # Coordinates are scaled by 4/radius, so all coefficients are integers.
        if index == 0:
            return [[], [4], []]
        if index == self.segments * (self.stacks - 1) + 1:
            return [[], [-4], []]
        longitude, latitude_zero = divmod(index - 1, self.stacks - 1)
        assert longitude < self.segments
        latitude = latitude_zero + 1
        lon = longitude * (self.order // self.segments)
        lat = latitude * (self.order // (2 * self.stacks))
        return [self.mul(self.cosine_twice(lon), self.sine_twice(lat)),
                [2 * c for c in self.cosine_twice(lat)],
                self.mul(self.sine_twice(lon), self.sine_twice(lat))]

    def orient(self, a, b, c, d):
        a, b, c = [[self.sub(x, y) for x, y in zip(p, d)] for p in [a, b, c]]
        terms = [self.mul(a[0], self.sub(self.mul(b[1], c[2]), self.mul(b[2], c[1]))),
                 self.mul(a[1], self.sub(self.mul(b[2], c[0]), self.mul(b[0], c[2]))),
                 self.mul(a[2], self.sub(self.mul(b[0], c[1]), self.mul(b[1], c[0])))]
        return self.reduce(add(add(terms[0], terms[1]), terms[2]))


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    destination = E / 'sphere-orientation-exact-oracle-v1.json'
    assert not destination.exists()
    source = E / 'sphere-orientation-trace-v1'
    trace = json.loads((source / 'result.json').read_text())
    assert trace['complete'] and trace['commands_passed'] == 7
    assert all(sha(source / p) == h for p, h in trace['artifacts_sha256'].items())
    assert cyclotomic(32) == [1] + [0] * 15 + [1]
    # A rational tetrahedron prevents an accidentally always-zero determinant.
    control = Field(4, 2)
    assert control.orient([[1], [], []], [[], [1], []], [[], [], [1]], [[], [], []]) == [1]
    assert control.orient([[1], [], []], [[], [], [1]], [[], [1], []], [[], [], []]) == [-1]
    cases = {'original_medium': (10, 32, 16), 'fifth_turn': (8, 10, 7), 'seventh_turn': (8, 7, 5)}
    results = []
    for observation in trace['observations']:
        radius, segments, stacks = cases[observation['case']]
        field = Field(segments, stacks)
        count = segments * (stacks - 1) + 2
        for row in observation['traces']:
            a, b, c, d, point_count = row
            assert point_count == count and all(0 <= i < count for i in [a, b, c, d])
            points = [field.point(i) for i in [a, b, c, d]]
            remainder = field.orient(*points)
            assert not remainder
            # Use the first original sample that proves this face is not a
            # degenerate plane. This control does not replace the failed input.
            nonzero = next((i, value) for i in range(count)
                           if (value := field.orient(*points[:3], field.point(i))))
            results.append({'case': observation['case'], 'radius': radius, 'segments': segments, 'stacks': stacks,
                'indices': [a, b, c, d], 'root_of_unity_order': field.order,
                'cyclotomic_polynomial_low_to_high': field.modulus,
                'scaled_points_polynomials': points, 'determinant_remainder': remainder,
                'exactly_zero': True, 'original_determinant_scale': f'{radius}^3/64',
                'nonzero_face_control_index': nonzero[0], 'nonzero_control_remainder': nonzero[1]})
    result = {'complete': True, 'trace_result_sha256': sha(source / 'result.json'),
        'oracle_source_sha256': sha(__file__), 'checks': results, 'rational_orientation_controls_passed': 2,
        'interpretation': 'All three traced determinants are exactly zero in the cyclotomic field of their unchanged sampled coordinates. Integer polynomial reduction is an independent mathematical oracle, not a new native certificate or a floating approximation.',
        'runtime_admitted': False, 'sphere_certificate_pending': True, 'plan_complete': False}
    destination.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({'complete': True, 'checks': [{k: r[k] for k in ['case', 'indices', 'root_of_unity_order',
        'exactly_zero', 'nonzero_face_control_index']} for r in results], 'result_sha256': sha(destination)}))


if __name__ == '__main__':
    main()
