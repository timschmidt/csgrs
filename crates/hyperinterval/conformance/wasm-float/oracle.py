#!/usr/bin/env python3
"""Generate exact Fraction expectations, without native floating arithmetic.

Positive IEEE encodings are ordered. Binary search of that finite order finds
the adjacent representable bounds of an exact rational, or of its square root
by comparing exact squares. This does not call the Rust/native interval backend.
"""

import argparse
from fractions import Fraction
from functools import lru_cache
import json
from pathlib import Path
import random


class Failure(Exception):
    def __init__(self, code):
        self.code = code


class Format:
    def __init__(self, width):
        self.width = width
        self.fraction = 23 if width == 32 else 52
        self.exponent = width - self.fraction - 1
        self.bias = (1 << (self.exponent - 1)) - 1
        self.sign = 1 << (width - 1)
        self.infinity = ((1 << self.exponent) - 1) << self.fraction
        self.maximum = self.infinity - 1

    @lru_cache(maxsize=65536)
    def decode(self, bits):
        exponent = (bits & (self.sign - 1)) >> self.fraction
        fraction = bits & ((1 << self.fraction) - 1)
        if exponent == (1 << self.exponent) - 1:
            raise Failure(16)
        mantissa = fraction + ((1 << self.fraction) if exponent else 0)
        power = (exponent or 1) - self.bias - self.fraction
        value = Fraction(mantissa << power, 1) if power >= 0 else Fraction(mantissa, 1 << -power)
        return -value if bits & self.sign else value

    def positive_bounds(self, value, square=False):
        assert value >= 0

        def compare(bits):
            exact = self.decode(bits)
            return exact * exact if square else exact

        if compare(self.maximum) < value:
            raise Failure(18)
        low, high = 0, self.maximum
        while low < high:
            middle = (low + high + 1) // 2
            if compare(middle) <= value:
                low = middle
            else:
                high = middle - 1
        return low, low if compare(low) == value else low + 1

    def bounds(self, value):
        if value >= 0:
            return self.positive_bounds(value)
        lo, hi = self.positive_bounds(-value)
        return hi | self.sign, (lo | self.sign) if lo else 0

    def project(self, values):
        return self.bounds(min(values))[0], self.bounds(max(values))[1]

    def interval(self, a, b, empty):
        if empty:
            return None
        lo, hi = self.decode(a), self.decode(b)
        if lo > hi:
            raise Failure(17)
        return lo, hi

    def adjacent(self, value, upward):
        bits = self.bounds(value)[0]
        if not value:
            return 1 if upward else self.sign | 1
        bits += 1 if (value > 0) == upward else -1
        self.decode(bits)  # An infinite required neighbor is overflow below.
        return 0 if bits == self.sign else bits

    def image(self, operation, x, y, scale, offset, exponent):
        if operation == 4 and y is not None and y[0] <= 0 <= y[1]:
            raise Failure(19)
        if operation == 7:
            scale, offset = self.decode(scale), self.decode(offset)
        if operation == 12:
            return (1, None) if x is None and y is None else (0, self.project((x or y) + (y or x)))
        if x is None or (operation in [1, 2, 3, 4, 10, 11, 13, 20] and y is None):
            return 1, None
        a, b = x
        if operation == 0:
            values = x
        elif operation == 1:
            values = [a + y[0], b + y[1]]
        elif operation == 2:
            values = [a - y[1], b - y[0]]
        elif operation == 3:
            values = [v * w for v in x for w in y]
        elif operation == 4:
            values = [v / w for v in x for w in y]
        elif operation == 5:
            values = [a * a, b * b] + ([Fraction(0)] if a <= 0 <= b else [])
        elif operation == 6:
            if b < 0:
                raise Failure(20)
            return (2 if a < 0 else 0), (
                self.positive_bounds(max(a, 0), square=True)[0],
                self.positive_bounds(b, square=True)[1],
            )
        elif operation == 7:
            values = [a * scale + offset, b * scale + offset]
        elif operation == 8:
            values = [-b, -a]
        elif operation == 9:
            values = [abs(a), abs(b)] + ([Fraction(0)] if a <= 0 <= b else [])
        elif operation in [10, 11]:
            function = min if operation == 10 else max
            values = [function(a, y[0]), function(b, y[1])]
        elif operation == 13:
            values = [max(a, y[0]), min(b, y[1])]
            if values[0] > values[1]:
                return 1, None
        elif operation == 14:
            if a <= 0 <= b:
                raise Failure(19)
            values = [1 / a, 1 / b]
        elif operation == 15:
            values = [(a + b) / 2]
        elif operation == 16:
            values = [(b - a) / 2]
        elif operation == 17:
            values = [b - a]
        elif operation == 18:
            try:
                return 0, (self.adjacent(a, False), self.adjacent(b, True))
            except Failure as error:
                assert error.code == 16
                raise Failure(18) from error
        elif operation == 19:
            # This path is used for cases with known tight endpoint results.
            values = [a ** exponent, b ** exponent]
            if exponent and exponent % 2 == 0 and a <= 0 <= b:
                values.append(Fraction(0))
        elif operation == 20:
            _, first = self.image(1, x, y, 0, 0, 0)
            _, second = self.image(5, tuple(map(self.decode, first)), None, 0, 0, 0)
            return self.image(2, tuple(map(self.decode, second)), x, 0, 0, 0)
        else:
            raise AssertionError(operation)
        return 0, self.project(values)

    def case(self, operation, flags, inputs, exponent=2):
        a, b, c, d = inputs
        try:
            x = self.interval(a, b, flags & 1)
            y = self.interval(c, d, flags & 2) if operation in [1, 2, 3, 4, 10, 11, 12, 13, 20] else None
            status, result = self.image(operation, x, y, c, d, exponent)
        except Failure as error:
            status, result = error.code, None
        return [self.width, operation, flags, *[hex(v) for v in inputs], exponent, status,
                *([hex(v) for v in result] if result is not None else [])]

    def properties(self, flags, a, b, c, d, point):
        try:
            x = self.interval(a, b, flags & 1)
            y = self.interval(c, d, flags & 2)
            point = self.decode(point)
            disjoint = x is None or y is None or x[1] < y[0] or y[1] < x[0]
            width = None if x is None or y is None else (x[1] - x[0], y[1] - y[0])
            facts = [
                x is None,
                x is not None and x[0] == x[1],
                x is not None and x[0] <= point <= x[1],
                x is None or y is not None and y[0] <= x[0] <= x[1] <= y[1],
                x is None or y is not None and y[0] < x[0] <= x[1] < y[1],
                x is not None and y is not None and x[1] < y[0],
                disjoint,
                not disjoint,
                width is not None and width[0] < width[1],
                width is not None and width[0] == width[1],
                width is not None and width[0] > width[1],
                x is not None and x[0] < 0,
                x is not None and x[0] <= 0 <= x[1],
                x is not None and x[1] > 0,
            ]
            return sum(int(value) << i for i, value in enumerate(facts))
        except Failure as error:
            return (1 << 63) | error.code


def cases(seed, samples):
    rng = random.Random(seed)
    for width in [32, 64]:
        fmt = Format(width)
        one = fmt.bias << fmt.fraction
        half = one - (1 << fmt.fraction)
        two = one + (1 << fmt.fraction)
        edges = [0, fmt.sign, 1, 2, (1 << fmt.fraction) - 1, 1 << fmt.fraction,
                 (1 << fmt.fraction) + 1, half, one - 1, one, one + 1, two,
                 fmt.maximum - 1, fmt.maximum]
        edges += [v | fmt.sign for v in edges[2:]]
        invalid = [fmt.infinity, fmt.infinity | fmt.sign, fmt.infinity | 1,
                   fmt.infinity | (1 << (fmt.fraction - 1)), (1 << width) - 1]
        for value in edges + invalid:
            for operation in range(21):
                yield fmt.case(operation, 0, [value, value, one, two])
            for flags in [0, 1, 2, 3]:
                for operation in [0, 1, 4, 6, 7, 12, 13, 20]:
                    yield fmt.case(operation, flags, [one, value, 0, value])
        for a in edges:
            for b in edges:
                for operation in [1, 2, 3, 4, 7, 15, 16, 17]:
                    yield fmt.case(operation, 0, [a, a, b, b])
        # Every finite normal exponent and every subnormal leading-bit position.
        # This enumerates exponent classes, not all IEEE significands.
        bands = [
            (exponent << fmt.fraction) | ((1 << fmt.fraction) - 1 if exponent % 2 else 1)
            for exponent in range(1, (1 << fmt.exponent) - 1)
        ] + [1 << bit for bit in range(fmt.fraction)]
        for value in bands:
            for operation in [0, 1, 2, 3, 4, 5, 6, 7, 14, 15, 16, 17, 18, 20]:
                yield fmt.case(operation, 0, [value, value, one + 1, one + 1])
        for _ in range(samples):
            points = [rng.getrandbits(width) for _ in range(4)]
            # Deterministic finite random bit patterns cover exponent/sign classes.
            points = [v if v & (fmt.sign - 1) < fmt.infinity else v ^ (1 << fmt.fraction) for v in points]
            a, b = sorted(points[:2], key=fmt.decode)
            c, d = sorted(points[2:], key=fmt.decode)
            for operation in range(21):
                yield fmt.case(operation, 0, [a, b, c, d], rng.randrange(3))
        for a, b in [(-2, 3), (-2, -1), (0, 2), (1, 2)]:
            for exponent in [3, 4, 5, 8, 10]:
                yield fmt.case(19, 0, [fmt.bounds(Fraction(a))[0], fmt.bounds(Fraction(b))[1], 0, 0], exponent)
        # The exact fused result remains finite despite an overflowing product.
        yield fmt.case(7, 0, [fmt.maximum, fmt.maximum, two, fmt.maximum | fmt.sign])
        # Nonempty underflow, half-subnormal radii and exact cancellation.
        yield fmt.case(7, 0, [1, 1, half, 0])
        yield fmt.case(15, 0, [0, 1, 0, 0])
        yield fmt.case(16, 0, [0, 1, 0, 0])
        yield fmt.case(1, 0, [one, one, one | fmt.sign, one | fmt.sign])


def near_one_power(fmt, n):
    """Project (1 + epsilon)^n by bounding the uncomputed binomial tail."""
    epsilon = Fraction(1, 1 << fmt.fraction)
    term = total = Fraction(1)
    for k in range(1, 65):
        term *= (n - k + 1) * epsilon / k
        total += term
        ratio = (n - k) * epsilon / (k + 1)
        # Successive term ratios decrease. The geometric series bounds all
        # remaining terms with rational arithmetic independent of IEEE power.
        assert 0 <= ratio < 1
        lower = fmt.bounds(total)
        if lower == fmt.bounds(total + term * ratio / (1 - ratio)):
            return lower
    raise AssertionError("binomial tail did not resolve both adjacent endpoints")


def higher_powers(fmt):
    one = fmt.bounds(Fraction(1))[0]
    half = fmt.bounds(Fraction(1, 2))[0]
    two = fmt.bounds(Fraction(2))[0]
    # Powers of zero and units can be evaluated exactly even at u32::MAX.
    # Once (1/2)^n is smaller than the least subnormal, the directed positive
    # bounds remain [0, least subnormal] at subsequent positive products.
    for exponent in ((1 << 31) - 1, 1 << 31, (1 << 32) - 1):
        for a, b in [(0, 0), (one | fmt.sign, one), (one | fmt.sign, one | fmt.sign)]:
            yield fmt.case(19, 0, [a, b, 0, 0], exponent)
        for magnitude in (1, half, two):
            for negative in (False, True):
                bits = magnitude | (fmt.sign if negative else 0)
                row = [fmt.width, 19, 0, hex(bits), hex(bits), "0x0", "0x0", exponent]
                if magnitude == two:
                    yield [*row, 18]
                elif negative and exponent % 2:
                    yield [*row, 0, hex(fmt.sign | 1), "0x0"]
                else:
                    yield [*row, 0, "0x0", "0x1"]

    # With |x| between 3/4 and 5/4, all nonzero intermediate products for
    # these exponents stay normal in both formats. A directed rounding of a
    # normal product changes its magnitude by at most a factor 1 +/- epsilon.
    # Any multiplication tree for x^n has n-1 internal products, counting
    # reused squares with multiplicity. Bound both directions independently
    # of the Rust evaluation order; a tighter result is valid.
    epsilon = Fraction(1, 1 << fmt.fraction)
    low = fmt.bounds(Fraction(3, 4))[0] + 1
    high = fmt.bounds(Fraction(5, 4))[0] - 1
    for n in (3, 4, 7, 15, 16, 31, 63, 127):
        down, up = (1 - epsilon) ** (n - 1), (1 + epsilon) ** (n - 1)
        for a, b in [(low, low), (one + 1, one + 1), (one - 1, high),
                     (high | fmt.sign, low | fmt.sign), (high | fmt.sign, low),
                     (0, low), (one | fmt.sign, 0)]:
            left, right = fmt.decode(a), fmt.decode(b)
            exact = [left ** n, right ** n]
            if n % 2 == 0 and left <= 0 <= right:
                exact.append(Fraction(0))
            tight = fmt.project(exact)
            outer = fmt.project([v * factor for v in exact for factor in (down, up)])
            yield [fmt.width, 19, 0, hex(a), hex(b), "0x0", "0x0", n, 0,
                   *map(hex, tight), *map(hex, outer)]

    # Exercise high exponent bits with a finite, nonconstant result. The
    # controls above test parity, overflow and underflow at the same boundary.
    exponents = ((0x55555, 0xAAAAA, (1 << 20) - 1, 1 << 20, (1 << 20) + 1)
                 if fmt.width == 32 else
                 (0x55555555, 0xAAAAAAAA, (1 << 31) - 1, 1 << 31, (1 << 32) - 1))
    assert near_one_power(fmt, 31) == fmt.bounds((1 + epsilon) ** 31)
    for n in exponents:
        tight = near_one_power(fmt, n)
        # All intermediates stay in [1, 2). When two lower bounds for powers
        # i,j are multiplied, 1+(i+j)*epsilon is representable and no larger
        # than their product. The upper bound allows n-1 directed roundings.
        outer = (one + n, near_one_power(fmt, 2 * n - 1)[1])
        assert outer[0] <= tight[0] <= tight[1] <= outer[1] < two
        for sign in (0, fmt.sign):
            source = hex(sign | (one + 1))
            if sign and n % 2:
                ends = [sign | magnitude for magnitude in (*reversed(tight), *reversed(outer))]
            else:
                ends = [*tight, *outer]
            yield [fmt.width, 19, 0, source, source, "0x0", "0x0", n, 0, *map(hex, ends)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--seed", type=int, default=20261115)
    parser.add_argument("--samples", type=int, default=200)
    args = parser.parse_args()
    for width, thirds, roots in [
        (32, (0x3EAAAAAA, 0x3EAAAAAB), (0x3FB504F3, 0x3FB504F4)),
        (64, (0x3FD5555555555555, 0x3FD5555555555556), (0x3FF6A09E667F3BCC, 0x3FF6A09E667F3BCD)),
    ]:
        fmt = Format(width)
        assert fmt.bounds(Fraction(1, 3)) == thirds
        assert fmt.positive_bounds(Fraction(2), square=True) == roots
        assert fmt.decode(roots[0]) ** 2 < 2 < fmt.decode(roots[1]) ** 2
        assert fmt.bounds(fmt.decode(1) / 2) == (0, 1)
        assert fmt.bounds(-fmt.decode(1) / 2) == (fmt.sign | 1, 0)
    rows = list(cases(args.seed, args.samples))
    for width in (32, 64):
        rows.extend(higher_powers(Format(width)))
    properties = []
    seen = set()
    formats = {width: Format(width) for width in [32, 64]}
    for row in rows:
        width, _, flags, a, b, c, d = row[:7]
        for point in [a, c]:
            key = (width, flags, a, b, c, d, point)
            if key not in seen:
                seen.add(key)
                result = formats[width].properties(flags, *map(lambda value: int(value, 16), [a, b, c, d, point]))
                properties.append([*key, hex(result)])
    lattice = []
    for width in [32, 64]:
        fmt = Format(width)
        for dimension in [0, 1, 2, 3, 4, 16, 64, 256, 1024]:
            for seed in range(17):
                lo = hi = 0
                for i in range(dimension):
                    a, b = (i + seed) % 17 - 8, (i + 2 * seed) % 13 - 6
                    # Four add/subtract pairs only widen A; intersecting A
                    # recovers it. The final Cartesian expression is hull(A,B).A.
                    ends = [min(a, b), max(a + i % 3, b + i % 2)]
                    products = [x * y for x in ends for y in [a, a + i % 3]]
                    lo += min(products)
                    hi += max(products)
                lower = fmt.bounds(Fraction(lo, 256))[0]
                upper = fmt.bounds(Fraction(hi, 256))[1]
                assert fmt.decode(lower) == Fraction(lo, 256)
                assert fmt.decode(upper) == Fraction(hi, 256)
                lattice.append([width, dimension, seed, hex(lower), hex(upper), lo, hi])
    args.output.write_text(json.dumps({"seed": args.seed, "samples_per_width": args.samples, "cases": rows, "properties": properties, "lattice": lattice}, separators=(",", ":")) + "\n")
    print(json.dumps({"cases": len(rows), "output": str(args.output)}))


if __name__ == "__main__":
    main()
