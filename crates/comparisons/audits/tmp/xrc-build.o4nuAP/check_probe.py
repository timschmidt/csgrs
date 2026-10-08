#!/usr/bin/env python3
from __future__ import annotations

from fractions import Fraction
import itertools
import math
import random
import subprocess
import sys

PROBE = sys.argv[1] if len(sys.argv) > 1 else "./probe"


def invoke(b: int, n: int, op: str, *args: int) -> tuple[str, int | str]:
    cmd = [PROBE, str(b), str(n), op, *(str(x) for x in args)]
    try:
        p = subprocess.run(cmd, text=True, capture_output=True, timeout=2.0)
    except subprocess.TimeoutExpired:
        return "TIMEOUT", ""
    if p.returncode:
        return f"EXIT_{p.returncode}", p.stderr.strip()
    try:
        return "OK", int(p.stdout.strip())
    except ValueError:
        return "BAD_OUTPUT", p.stdout.strip()


def enclosed(b: int, n: int, z: int, value: Fraction) -> bool:
    # Strictly test |2^(b*n) value - z| < 1 without floating point.
    return abs((1 << (b * n)) * value.numerator - z * value.denominator) < value.denominator


def exact_value(op: str, args: tuple[int, ...]) -> Fraction:
    if op == "rat":
        return Fraction(args[0], args[1])
    if op in ("add", "sub", "mul"):
        x, y = Fraction(args[0], args[1]), Fraction(args[2], args[3])
        return {"add": x + y, "sub": x - y, "mul": x * y}[op]
    if op == "imul":
        return args[0] * Fraction(args[1], args[2])
    if op == "divi":
        return Fraction(args[0], args[1]) / args[2]
    if op == "sqr":
        return Fraction(args[0], args[1]) ** 2
    if op == "sqrt":
        value = Fraction(args[0], args[1])
        return Fraction(math.isqrt(value.numerator), math.isqrt(value.denominator))
    if op == "root":
        value, degree = Fraction(args[0], args[1]), args[2]
        sign = -1 if value.numerator < 0 else 1
        nr = round(abs(value.numerator) ** (1 / degree))
        dr = round(value.denominator ** (1 / degree))
        assert nr**degree == abs(value.numerator) and dr**degree == value.denominator
        return Fraction(sign * nr, dr)
    if op == "recip":
        return 1 / Fraction(args[0], args[1])
    if op == "powi":
        return Fraction(args[0], args[1]) ** args[2]
    raise AssertionError(op)


def main() -> None:
    rng = random.Random(0x58524312)
    cases: list[tuple[str, tuple[int, ...]]] = []
    rationals = [(-17, 13), (-8, 3), (-1, 7), (1, 9), (2, 1), (7, 5), (16, 3)]
    for a, d in rationals:
        cases.append(("rat", (a, d)))
        cases.append(("sqr", (a, d)))
        cases.append(("recip", (a, d)))
        for m in (-257, -17, -2, -1, 1, 2, 17, 257):
            cases.append(("imul", (m, a, d)))
        for m in (1, 2, 3, 17, 257):
            cases.append(("divi", (a, d, m)))
        for p in (-5, -3, -1, 0, 1, 2, 3, 6):
            cases.append(("powi", (a, d, p)))
    for _ in range(24):
        a, c = rng.randint(-50, 50), rng.randint(-50, 50)
        d1 = rng.randint(1, 31)
        d2 = rng.randint(1, 31)
        for op in ("add", "sub", "mul"):
            cases.append((op, (a, d1, c, d2)))
    for a, d in ((0, 1), (1, 1), (4, 1), (9, 1), (1, 4), (9, 16), (256, 81)):
        cases.append(("sqrt", (a, d)))
    for a, d, degree in ((0, 1, 3), (1, 1, 2), (8, 1, 3), (-8, 1, 3),
                         (16, 81, 4), (625, 64, 2)):
        cases.append(("root", (a, d, degree)))

    total = failures = process_failures = 0
    by_op: dict[str, list[int]] = {}
    by_b: dict[int, list[int]] = {}
    by_n: dict[int, list[int]] = {}
    boundary_failures = 0
    worst: tuple[Fraction, str] = (Fraction(0), "")
    examples: list[str] = []
    examples_by_op: dict[str, list[str]] = {}
    for b, n, (op, args) in itertools.product((1, 2, 3, 4, 5, 8), (0, 1, 2, 5, 13, 31), cases):
        total += 1
        status, result = invoke(b, n, op, *args)
        by_op.setdefault(op, [0, 0])
        by_op[op][0] += 1
        by_b.setdefault(b, [0, 0])[0] += 1
        by_n.setdefault(n, [0, 0])[0] += 1
        if status != "OK":
            process_failures += 1
            by_op[op][1] += 1
            by_b[b][1] += 1
            by_n[n][1] += 1
            if len(examples) < 24:
                examples.append(f"{status}: b={b} n={n} {op}{args}: {result}")
            if len(examples_by_op.setdefault(op, [])) < 4:
                examples_by_op[op].append(f"{status}: b={b} n={n} {op}{args}: {result}")
            continue
        value = exact_value(op, args)
        if not enclosed(b, n, int(result), value):
            failures += 1
            by_op[op][1] += 1
            by_b[b][1] += 1
            by_n[n][1] += 1
            delta_num = abs((1 << (b * n)) * value.numerator - int(result) * value.denominator)
            ratio = Fraction(delta_num, value.denominator)
            if ratio == 1:
                boundary_failures += 1
            detail = f"b={b} n={n} {op}{args} z={result}"
            if ratio > worst[0]:
                worst = (ratio, detail)
            if len(examples) < 24:
                examples.append(
                    f"INVARIANT: b={b} n={n} {op}{args} z={result} "
                    f"scaled_error={delta_num}/{value.denominator}"
                )
            if len(examples_by_op.setdefault(op, [])) < 4:
                examples_by_op[op].append(
                    f"INVARIANT: b={b} n={n} {op}{args} z={result} "
                    f"scaled_error={delta_num}/{value.denominator}"
                )

    print(f"total={total} invariant_failures={failures} process_failures={process_failures}")
    for op in sorted(by_op):
        print(f"{op}: {by_op[op][1]}/{by_op[op][0]} failed")
    print("by_b: " + " ".join(f"{b}={bad}/{count}" for b, (count, bad) in sorted(by_b.items())))
    print("by_n: " + " ".join(f"{n}={bad}/{count}" for n, (count, bad) in sorted(by_n.items())))
    print(f"exactly_one={boundary_failures} worst={float(worst[0]):.9g} ({worst[0]}) {worst[1]}")
    for op in sorted(examples_by_op):
        for example in examples_by_op[op]:
            print(example)


if __name__ == "__main__":
    main()
