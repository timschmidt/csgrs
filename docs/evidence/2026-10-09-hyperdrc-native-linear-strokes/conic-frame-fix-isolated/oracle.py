"""Independent rational circle/line oracle; uses no Hyper implementation."""

from fractions import Fraction as Q
import json
from math import isqrt


def roots(a, b, c):
    if a == 0:
        assert b != 0
        return [-c / b]
    discriminant = b * b - 4 * a * c
    if discriminant < 0:
        return []
    numerator = isqrt(discriminant.numerator)
    denominator = isqrt(discriminant.denominator)
    square_root = Q(numerator, denominator)
    assert square_root * square_root == discriminant
    return sorted({(-b - square_root) / (2 * a), (-b + square_root) / (2 * a)})


def point(t):
    return (1 - t * t) / (1 + t * t), 2 * t / (1 + t * t)


lines = [
    (1, 0, Q(3, 5)), (0, 1, Q(3, 5)), (1, 1, Q(7, 5)),
    (3, 4, Q(5)), (1, 0, Q(1)), (0, 1, Q(1)),
    (1, 1, Q(1)), (1, 0, Q(5, 4)), (1, 0, Q(-3, 5)),
    (0, 1, Q(-3, 5)), (0, 1, Q(0)), (1, 0, Q(0)),
]
ranges = [(Q(0), Q(1)), (Q(1, 4), Q(3, 4)), (Q(1, 3), Q(1, 2))]
rows = []
for a, b, c in lines:
    contacts = []
    for t in roots(-(a + c), 2 * Q(b), a - c):
        if not 0 <= t <= 1:
            continue
        x, y = point(t)
        assert x * x + y * y == 1
        assert a * x + b * y == c
        dx = -4 * t / (1 + t * t) ** 2
        dy = 2 * (1 - t * t) / (1 + t * t) ** 2
        cross = -b * dy - a * dx
        contacts.append(dict(parameter=str(t), point=[str(x), str(y)],
                             crossing_sign=(cross > 0) - (cross < 0)))
    rows.append(dict(normal=[a, b], constant=str(c), contacts=contacts))

# The regression computes range images with these independent integer ratios.
transport = []
for start, end in ranges:
    for reversed_curve in [False, True]:
        local = Q(1, 4)
        source = (start + 3 * end) / 4 if reversed_curve else (3 * start + end) / 4
        restored = (source - start) / (end - start)
        if reversed_curve:
            restored = 1 - restored
        assert restored == local
        transport.append(dict(range=[str(start), str(end)], reversed=reversed_curve,
                              source_parameter=str(source), local_parameter=str(local)))

assert point(1 - Q(2, 3)) == (Q(4, 5), Q(3, 5))
assert point(1 - Q(1, 3)) == (Q(5, 13), Q(12, 13))
print(json.dumps(dict(
    curve="C(t)=((1-t^2)/(1+t^2),2t/(1+t^2)), 0<=t<=1",
    method="Solve the exact substituted quadratic and verify the circle, line, and derivative cross product.",
    lines=rows,
    transport=transport,
    regression_queries=dict(line_contacts=2*2*2*3*2*2*2*12,
                            shared_point_transport=2*2*2*3*2*2*2,
                            primary_parameter_map=2*2*2*3*2*2*2),
), indent=2))
