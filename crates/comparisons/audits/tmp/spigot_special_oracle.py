#!/usr/bin/env python3
import subprocess
import time

import mpmath as mp

SPIGOT = "/tmp/spigot-audit.iKvIgb/gmp/spigot"
DIGITS = 100
mp.mp.dps = DIGITS + 80


def q(n, d):
    return mp.mpf(n) / d


cases = [
    ("gamma(-7/3)", "gamma(-7/3)", lambda: mp.gamma(q(-7, 3))),
    ("gamma(-3/2)", "gamma(-3/2)", lambda: mp.gamma(q(-3, 2))),
    ("gamma(-1/10)", "gamma(-1/10)", lambda: mp.gamma(q(-1, 10))),
    ("gamma(1/10)", "gamma(1/10)", lambda: mp.gamma(q(1, 10))),
    ("gamma(1/2)", "gamma(1/2)", lambda: mp.gamma(q(1, 2))),
    ("gamma(10/3)", "gamma(10/3)", lambda: mp.gamma(q(10, 3))),
    ("lgamma(-101/10)", "lgamma(-101/10)", lambda: mp.log(abs(mp.gamma(q(-101, 10))))),
    ("lgamma(101)", "lgamma(101)", lambda: mp.log(mp.gamma(101))),
    ("W(-1/e)", "W(-1/e)", lambda: mp.mpf(-1)),
    ("Wn(-1/e)", "Wn(-1/e)", lambda: mp.mpf(-1)),
    ("W(-367/1000)", "W(-367/1000)", lambda: mp.lambertw(q(-367, 1000), 0).real),
    ("Wn(-367/1000)", "Wn(-367/1000)", lambda: mp.lambertw(q(-367, 1000), -1).real),
    ("W(-1/10)", "W(-1/10)", lambda: mp.lambertw(q(-1, 10), 0).real),
    ("Wn(-1/10)", "Wn(-1/10)", lambda: mp.lambertw(q(-1, 10), -1).real),
    ("W(1/10)", "W(1/10)", lambda: mp.lambertw(q(1, 10), 0).real),
    ("W(200)", "W(200)", lambda: mp.lambertw(200, 0).real),
    ("zeta(-21/10)", "zeta(-21/10)", lambda: mp.zeta(q(-21, 10))),
    ("zeta(-2)", "zeta(-2)", lambda: mp.mpf(0)),
    ("zeta(1/1000)", "zeta(1/1000)", lambda: mp.zeta(q(1, 1000))),
    ("zeta(4/3)", "zeta(4/3)", lambda: mp.zeta(q(4, 3))),
    ("zeta(pi)", "zeta(pi)", lambda: mp.zeta(mp.pi)),
    ("agm(1,2)", "agm(1,2)", lambda: mp.agm(1, 2)),
    ("agm(1,100000)", "agm(1,100000)", lambda: mp.agm(1, 100000)),
    ("agm(pi,e)", "agm(pi,e)", lambda: mp.agm(mp.pi, mp.e)),
    ("Hg(1;2;1/3)", "Hg(1;2;1/3)", lambda: mp.hyper([1], [2], q(1, 3))),
    (
        "Hg(1/2,1/2;1;1/2)",
        "Hg(1/2,1/2;1;1/2)",
        lambda: mp.hyper([q(1, 2), q(1, 2)], [1], q(1, 2)),
    ),
    (
        "Hg(1/2,1/3;5/4;-3/4)",
        "Hg(1/2,1/3;5/4;-3/4)",
        lambda: mp.hyper([q(1, 2), q(1, 3)], [q(5, 4)], q(-3, 4)),
    ),
    ("BesselJ(0,67)", "BesselJ(0,67)", lambda: mp.besselj(0, 67)),
    ("BesselJ(7,1/100)", "BesselJ(7,1/100)", lambda: mp.besselj(7, q(1, 100))),
    ("BesselI(-2,12)", "BesselI(-2,12)", lambda: mp.besseli(-2, 12)),
]

passed = 0
failed = []
started = time.perf_counter()
for name, expression, oracle in cases:
    case_start = time.perf_counter()
    try:
        result = subprocess.run(
            [SPIGOT, f"-d{DIGITS}", "--", expression],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=30,
            check=False,
        )
    except subprocess.TimeoutExpired:
        failed.append((name, "timeout"))
        print(f"TIMEOUT {name}")
        continue
    elapsed = time.perf_counter() - case_start
    output = result.stdout.strip()
    if result.returncode != 0:
        failed.append((name, f"exit {result.returncode}: {result.stderr.strip()}"))
        print(f"ERROR   {name}: exit {result.returncode}")
        continue
    try:
        actual = mp.mpf(output)
        expected = oracle()
    except Exception as exc:
        failed.append((name, f"parse/oracle: {exc}; output={output!r}"))
        print(f"ERROR   {name}: {exc}")
        continue
    error = abs(actual - expected)
    tolerance = mp.power(10, -(DIGITS - 3)) * max(mp.mpf(1), abs(expected))
    if error > tolerance:
        failed.append((name, f"error={mp.nstr(error, 8)} output={output}"))
        print(f"FAIL    {name}: error {mp.nstr(error, 8)}")
    else:
        passed += 1
        print(f"PASS    {name}: {elapsed:.3f}s, error {mp.nstr(error, 4)}")

print(
    f"summary: {passed}/{len(cases)} pass in {time.perf_counter() - started:.3f}s; "
    f"failures={failed!r}"
)
