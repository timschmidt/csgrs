#!/usr/bin/env python3
"""Exact analytic acceptance for the external IVP corpus; no solver-to-solver oracle."""
import argparse
from fractions import Fraction as Q
import json
import math
from pathlib import Path

if not __debug__:
    raise RuntimeError("validation requires Python assertions enabled")

CASES = [(kind, box) for kind in ("constant", "exponential", "riccati", "oscillator",
                                "time_growth", "turnaround", "noncommuting", "two_rate",
                                "oscillator_long", "riccati_long")
         for box in (False, True)]
END_WIDTH = Q(1, 64)


def horizon(kind):
    """Exact horizon, per-piece step cap and whole-tube width limit."""
    if kind == "oscillator_long":
        return Q(8), Q(1, 8), Q(4)
    if kind == "riccati_long":
        return Q(1, 2), Q(1, 16), Q(4)
    return Q(1, 4), Q(1, 64) if kind == "two_rate" else Q(1, 16), Q(1)


def product(u, v):
    """Exact hull of {x*y : x in u, y in v} for closed rational intervals."""
    corners = [x * y for x in u for y in v]
    return min(corners), max(corners)


def series(kind, x):
    """Rational bounds from a positive-series tail or the alternating-series theorem."""
    if kind == "exp":
        term = total = Q(1)
        for k in range(1, 50):
            term *= x / k
            total += term
        first = term * x / 50
        return total, total + first / (1 - x / 51)
    term = Q(1) if kind == "cos" else x
    total = term
    for k in range(1, 40):
        a = 2 * k if kind == "cos" else 2 * k + 1
        term *= -x * x / (a * (a - 1))
        total += term
    a = 80 if kind == "cos" else 81
    following = term * -x * x / (a * (a - 1))
    return min(total, total + following), max(total, total + following)


def sqrt_bounds(x):
    scale = 1 << 180
    lower = math.isqrt(x.numerator * scale * scale // x.denominator)
    lo = Q(lower, scale)
    return lo, lo if lo * lo == x else Q(lower + 1, scale)


def oracle(kind, box):
    radius = Q(1, 1024) if box else Q(0)
    a, b, t = 1 - radius, 1 + radius, horizon(kind)[0]
    initial = [(a, b)]
    if kind == "constant":
        end = [(a + 2 * t, b + 2 * t)]
        tube = [(a, end[0][1])]
    elif kind == "exponential":
        lo, hi = series("exp", t)
        end = [(a * lo, b * hi)]
        tube = [(a, end[0][1])]
    elif kind in ("riccati", "riccati_long"):
        end = [(a / (1 - t * a), b / (1 - t * b))]
        tube = [(a, end[0][1])]
    elif kind == "oscillator":
        initial.append((-radius, radius))
        cl, cu = series("cos", t)
        sl, su = series("sin", t)
        end = [(a * cl - radius * su, b * cu + radius * su),
               (-b * su - radius * cu, -a * sl + radius * cu)]
        # x extrema: minimum at T, maximum at atan(radius/b), in [0,T].
        # y decreases throughout [0,T] for this entire initial rectangle.
        tube = [(end[0][0], sqrt_bounds(b * b + radius * radius)[1]),
                (end[1][0], radius)]
    elif kind == "oscillator_long":
        initial.append((-radius, radius))
        c, s = series("cos", t), series("sin", t)
        x0, y0 = (a, b), (-radius, radius)
        x = [sum(p) for p in zip(product(x0, c), product(y0, s))]
        minus_x0 = (-b, -a)
        y = [sum(p) for p in zip(product(minus_x0, s), product(y0, c))]
        end = [tuple(x), tuple(y)]
        # Horizon 8 exceeds a full turn, so both coordinates reach +-|(x0,y0)|;
        # the (b, radius) corner has the largest modulus.
        modulus = sqrt_bounds(b * b + radius * radius)[1]
        tube = [(-modulus, modulus), (-modulus, modulus)]
    elif kind == "time_growth":
        lo, hi = series("exp", t*t/2)
        end = [(a*lo, b*hi)]
        tube = [(a, end[0][1])]
    elif kind == "turnaround":
        end = initial.copy()  # t-4t² = 0 at T=1/4.
        tube = [(a, b+Q(1, 16))]  # Interior maximum at t=1/8.
    elif kind == "noncommuting":
        initial += [(-radius, radius), (a, b)]
        end = [(a-t*radius+t**3*a/6, b+t*radius+t**3*b/6),
               (-radius+t*t*a/2, radius+t*t*b/2), (a, b)]
        # The lower x envelope decreases until sqrt(2r/a), then increases.
        # At this root its value is a-(2/3)r*sqrt(2r/a). An upper sqrt bound
        # gives a certified lower bound on that minimum. Other extrema are monotone.
        assert 0 <= 2*radius/a <= t*t
        minimum = a-Q(2, 3)*radius*sqrt_bounds(2*radius/a)[1]
        tube = [(minimum, end[0][1]), (-radius, end[1][1]), (a, b)]
    elif kind == "two_rate":
        initial.append((a, b))
        end = []
        for rate in (1, 8):
            lo, hi = series("exp", rate*t)
            end.append((a/hi, b/lo))
        tube = [(value[0], b) for value in end]
    else:
        raise ValueError(f"unknown IVP: {kind}")
    return initial, end, tube


def decode(row, pair):
    if row["encoding"] == "dyadic":
        lo, hi, exponent = pair
        assert isinstance(exponent, int) and -10000 <= exponent <= 10000
        scale = Q(2) ** exponent
        result = Q(int(lo)) * scale, Q(int(hi)) * scale
    else:
        assert row["encoding"] == "ieee_hex"
        result = tuple(Q.from_float(float.fromhex(v)) for v in pair)
    assert result[0] <= result[1]
    return result


def validate(rows):
    names = {f"{kind}_{'box' if box else 'point'}": (kind, box) for kind, box in CASES}
    assert len(rows) == len(names), "missing/duplicate corpus records"
    assert {row["case"] for row in rows} == set(names), "wrong corpus"
    result = []
    for row in rows:
        kind, box = names[row["case"]]
        assert row["status"] == "certified"
        assert 0 < row["pieces"] <= 256
        assert decode(row, row["start"]) == (0, 0)
        end_time, cap, tube_width = horizon(kind)
        assert decode(row, row["end_time"]) == (end_time, end_time)
        times = [decode(row, value) for value in row["time_domains"]]
        assert len(times) == row["pieces"] + 1
        assert times[0] == (0, 0) and times[-1] == (end_time, end_time)
        for left, right in zip(times, times[1:]):
            assert 0 < right[0] - left[1] <= right[1] - left[0] <= cap
        initial, expected_end, expected_tube = oracle(kind, box)
        assert [decode(row, v) for v in row["initial"]] == initial
        widths = {}
        for label, expected, limit in [("end", expected_end, END_WIDTH),
                                        ("tube", expected_tube, tube_width)]:
            actual = [decode(row, v) for v in row[label]]
            assert len(actual) == len(expected)
            for lane, ((lo, hi), (el, eh)) in enumerate(zip(actual, expected)):
                assert lo <= el <= eh <= hi, (row["case"], label, lane, "analytic containment")
                assert hi - lo <= limit, (row["case"], label, lane, "width limit")
            widths[label] = [str(hi - lo) for lo, hi in actual]
        result.append({"case": row["case"], "solver": row["solver"],
                       "analytic_containment": True, "exact_widths": widths})
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("records", type=Path, nargs="+")
    args = parser.parse_args()
    output = []
    for path in args.records:
        rows = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
        output.extend(validate([row for row in rows if row.get("kind", "certificate") == "certificate"]))
    print(json.dumps(output, indent=2))


if __name__ == "__main__":
    main()
