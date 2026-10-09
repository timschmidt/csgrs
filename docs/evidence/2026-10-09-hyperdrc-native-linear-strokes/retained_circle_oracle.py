"""Exact, independent oracle for the retraced supported-circle regression.

Uses only Python's standard-library rational arithmetic. Run from any directory;
the assertions verify the retained JSON certificate without using Hyper types.
"""

from fractions import Fraction as Q
from math import comb


def power_coefficients(bernstein):
    degree = len(bernstein) - 1
    result = [Q(0)] * (degree + 1)
    for i, value in enumerate(bernstein):
        for j in range(degree - i + 1):
            result[i + j] += value * comb(degree, i) * comb(degree - i, j) * (-1) ** j
    return result


def multiply(left, right):
    result = [Q(0)] * (len(left) + len(right) - 1)
    for i, a in enumerate(left):
        for j, b in enumerate(right):
            result[i + j] += a * b
    return result


def evaluate(coefficients, parameter):
    return sum(value * parameter ** i for i, value in enumerate(coefficients))


def derivative(coefficients):
    return [i * value for i, value in enumerate(coefficients)][1:]


controls = [
    (Q(1), Q(0), Q(1)),
    (Q(1), Q(2), Q(1)),
    (Q(-5, 3), Q(8, 3), Q(11, 3)),
    (Q(1), Q(2), Q(1)),
    (Q(1), Q(0), Q(1)),
]
x, y, w = [power_coefficients(axis) for axis in zip(*controls)]
assert x == [1, 0, -16, 32, -16]
assert y == [0, 8, -8, 0, 0]
assert w == [1, 0, 16, -32, 16]
assert all(a + b == c for a, b, c in zip(multiply(x, x), multiply(y, y), multiply(w, w)))
assert all(control[2] > 0 for control in controls)  # No denominator pole on [0,1].
assert controls == controls[::-1]  # C(1-t)=C(t); the oriented integral is zero.


def point(t):
    return tuple(evaluate(axis, t) / evaluate(w, t) for axis in (x, y))


def tangent(t):
    weight, dw = evaluate(w, t), evaluate(derivative(w), t)
    return tuple(
        (evaluate(derivative(axis), t) * weight - evaluate(axis, t) * dw) / weight ** 2
        for axis in (x, y)
    )


assert tangent(Q(0)) == (0, 8)
assert tangent(Q(1)) == (0, -8)
assert point(Q(1, 4)) == point(Q(3, 4)) == (Q(7, 25), Q(24, 25))
print("Verified: exact circle support, opposite endpoint tangents, retraced image, zero oriented area.")
