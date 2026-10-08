#!/usr/bin/env python3
from __future__ import annotations

import mpmath as mp
import subprocess
import sys

sys.set_int_max_str_digits(0)

PROBE = "./probe"


def invoke(b: int, n: int, op: str, *args: int) -> tuple[str, int | str]:
    cmd = [PROBE, str(b), str(n), op, *(str(x) for x in args)]
    try:
        p = subprocess.run(cmd, text=True, capture_output=True, timeout=30.0)
    except subprocess.TimeoutExpired:
        return "TIMEOUT", ""
    if p.returncode:
        return f"EXIT_{p.returncode}", p.stderr.strip()
    try:
        return "OK", int(p.stdout.strip())
    except ValueError:
        return "BAD_OUTPUT", p.stdout.strip()


def main() -> None:
    results: dict[str, list[int]] = {}
    examples: dict[str, list[str]] = {}
    worst: dict[str, tuple[mp.mpf, str]] = {}
    total = failures = process_failures = 0
    cases: list[tuple[int, int, str, tuple[int, ...]]] = []
    for b in (1, 2, 3, 4, 5, 8):
        for n in (0, 1, 2, 5, 13, 31, 80, 200):
            cases.append((b, n, "pi", ()))
            for args in ((-3, 4), (-1, 3), (0, 1), (1, 3), (3, 4)):
                cases.append((b, n, "exp1", args))
            for args in ((-5, 2), (-1, 1), (-1, 3), (1, 3), (1, 1), (5, 2)):
                cases.append((b, n, "exp", args))
    for b in (1, 2, 3, 4):
        for n in (500, 1000, 2000, 5000):
            cases.append((b, n, "pi", ()))

    for b, n, op, args in cases:
        total += 1
        results.setdefault(op, [0, 0])[0] += 1
        status, result = invoke(b, n, op, *args)
        if status != "OK":
            process_failures += 1
            results[op][1] += 1
            if len(examples.setdefault(op, [])) < 20:
                examples[op].append(f"{status}: b={b} n={n} {op}{args}: {result}")
            continue
        mp.mp.dps = max(100, int(b * n * 0.302) + 100)
        value = mp.pi if op == "pi" else mp.exp(mp.mpf(args[0]) / args[1])
        error = abs(mp.ldexp(value, b * n) - int(result))
        if error >= 1:
            failures += 1
            results[op][1] += 1
            detail = f"b={b} n={n} {op}{args} z={result} error={mp.nstr(error, 14)}"
            if op not in worst or error > worst[op][0]:
                worst[op] = (error, detail)
            if len(examples.setdefault(op, [])) < 20:
                examples[op].append("INVARIANT: " + detail)

    print(f"total={total} invariant_failures={failures} process_failures={process_failures}")
    for op in sorted(results):
        count, bad = results[op]
        print(f"{op}: {bad}/{count} failed")
        if op in worst:
            print("worst: " + worst[op][1])
        for example in examples.get(op, []):
            print(example)


if __name__ == "__main__":
    main()
