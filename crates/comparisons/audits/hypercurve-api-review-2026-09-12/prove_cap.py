"""Exact Bernstein certificates for the rational cap used by probe.rs."""
from fractions import Fraction as F
from math import comb


def add(a, b):
    return [
        (a[i] if i < len(a) else F(0)) + (b[i] if i < len(b) else F(0))
        for i in range(max(len(a), len(b)))
    ]


def scale(a, k):
    return [c * k for c in a]


def mul(a, b):
    result = [F(0)] * (len(a) + len(b) - 1)
    for i, x in enumerate(a):
        for j, y in enumerate(b):
            result[i + j] += x * y
    return result


def sub(a, b):
    return add(a, scale(b, -1))


def derivative(a):
    return [i * c for i, c in enumerate(a)][1:]


def power(values):
    n = len(values) - 1
    result = [F(0)] * (n + 1)
    for i, v in enumerate(values):
        for j in range(n - i + 1):
            result[i + j] += v * comb(n, i) * comb(n - i, j) * (-1) ** j
    return result


def bernstein(a):
    a = list(a)
    while len(a) > 1 and a[-1] == 0:
        a.pop()
    n = len(a) - 1
    return [
        sum(a[i] * F(comb(k, i), comb(n, i)) for i in range(k + 1))
        for k in range(n + 1)
    ]


weights = [1, 2, 3, 4, 5, 7]
xs = list(range(6))
ys = [0, 1, 2, 2, 1, 0]
w = power(weights)
x = power([a * b for a, b in zip(weights, xs)])
y = power([a * b for a, b in zip(weights, ys)])
vx = sub(mul(derivative(x), w), mul(x, derivative(w)))
vy = sub(mul(derivative(y), w), mul(y, derivative(w)))
turn = sub(mul(vx, derivative(vy)), mul(vy, derivative(vx)))
vx_bernstein = bernstein(vx)
turn_bernstein = bernstein(turn)
assert w == list(map(F, [1, 5, 0, 0, 0, 1]))
assert all(c > 0 for c in vx_bernstein)
assert all(c <= 0 for c in turn_bernstein)
assert any(c < 0 for c in turn_bernstein)
print("W power:", w)
print("tangent x Bernstein:", vx_bernstein)
print("curvature numerator Bernstein:", turn_bernstein)
print("x tangent strictly positive:", all(c > 0 for c in vx_bernstein))
print("curvature nonpositive:", all(c <= 0 for c in turn_bernstein))

endpoint_speeds = []
for t in [0, 1]:
    evaluate = lambda a: sum(c * F(t) ** i for i, c in enumerate(a))
    dx = evaluate(vx) / evaluate(w) ** 2
    dy = evaluate(vy) / evaluate(w) ** 2
    assert evaluate(turn) == 0
    endpoint_speeds.append(dx * dx + dy * dy)
assert endpoint_speeds == [F(200), F(1250, 49)]
print("Endpoint squared speeds:", endpoint_speeds)
print("Both endpoint curvatures vanish, so regular parallel derivatives have the same endpoint speeds.")
