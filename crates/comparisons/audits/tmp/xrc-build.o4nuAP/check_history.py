#!/usr/bin/env python3
from fractions import Fraction
import subprocess
import sys


EXECUTABLE = sys.argv[1] if len(sys.argv) > 1 else "./history_probe"


def exact_logistic(iterations: int) -> Fraction:
    x = Fraction(43, 64)
    for _ in range(iterations):
        x = 4 * x * (1 - x)
    return x


def run(b: int, iterations: int, warm: int, final: int):
    cp = subprocess.run(
        [EXECUTABLE, str(b), str(iterations), str(warm), str(final)],
        check=True,
        capture_output=True,
        text=True,
        timeout=30,
    )
    fields = dict(line.split("=", 1) for line in cp.stdout.splitlines())
    return int(fields["cold"]), int(fields["warm"]), int(fields["delta"])


def scaled_error(value: Fraction, approximation: int, scale_bits: int) -> Fraction:
    return abs(value * (1 << scale_bits) - approximation)


failures = []
cases = 0
for b in (1, 2, 4):
    for iterations in range(0, 15):
        exact = exact_logistic(iterations)
        for warm in (0, 1, 5, 10, 20, 30, 50):
            final = 100
            cold, warmed, delta = run(b, iterations, warm, final)
            cold_error = scaled_error(exact, cold, b * final)
            warm_error = scaled_error(exact, warmed, b * final)
            cases += 1
            if cold_error >= 1 or warm_error >= 1 or delta >= 2:
                failures.append((
                    b,
                    iterations,
                    warm,
                    delta,
                    cold_error,
                    warm_error,
                ))

print(f"cases={cases} failures={len(failures)}")
for row in failures[:40]:
    b, iterations, warm, delta, cold_error, warm_error = row
    print(
        f"b={b} iterations={iterations} warm={warm} delta={delta} "
        f"cold_error={float(cold_error):.9g} "
        f"warm_error={float(warm_error):.9g}"
    )
