#!/usr/bin/env python3
import concurrent.futures
import random
import re
import subprocess
import sys

import mpmath as mp

mp.mp.dps = 300
random.seed(0x1C6EA15)

FUNCTIONS = {
    "id": lambda x: x,
    "abs": abs,
    "sqrt": mp.sqrt,
    "exp": mp.exp,
    "log": mp.log,
    "sin": mp.sin,
    "cos": mp.cos,
    "tan": mp.tan,
    "asin": mp.asin,
    "acos": mp.acos,
    "atan": mp.atan,
    "sinh": mp.sinh,
    "cosh": mp.cosh,
    "tanh": mp.tanh,
    "asinh": mp.asinh,
    "acosh": mp.acosh,
    "atanh": mp.atanh,
}

def rationals(count, low, high):
    values = set()
    while len(values) < count:
        den = random.randint(1, 29)
        num = random.randint(int(low * den), int(high * den))
        if low <= num / den <= high:
            values.add((num, den))
    return sorted(values)

domains = {
    "id": rationals(16, -20, 20),
    "abs": rationals(16, -20, 20),
    "sqrt": rationals(16, 0.001, 20),
    "exp": rationals(16, -10, 10),
    "log": rationals(16, 0.001, 20),
    "sin": rationals(16, -20, 20),
    "cos": rationals(16, -20, 20),
    "tan": rationals(16, -20, 20),
    "asin": rationals(16, -0.99, 0.99),
    "acos": rationals(16, -0.99, 0.99),
    "atan": rationals(16, -20, 20),
    "sinh": rationals(16, -10, 10),
    "cosh": rationals(16, -10, 10),
    "tanh": rationals(16, -10, 10),
    "asinh": rationals(16, -20, 20),
    "acosh": rationals(16, 1.001, 20),
    "atanh": rationals(16, -0.99, 0.99),
}

cases = []
for digits in (20, 80, 200):
    for op, values in domains.items():
        for num, den in values:
            x = mp.mpf(num) / den
            cases.append((op, num, den, digits, FUNCTIONS[op](x)))

pattern = re.compile(r"^([^ ]+e[+-]?\d+) \+\-([^ ]+e[+-]?\d+)$")

def run(case):
    op, num, den, digits, expected = case
    try:
        result = subprocess.run(
            [sys.argv[1], op, str(num), str(den), str(digits)],
            text=True, capture_output=True, timeout=3)
    except subprocess.TimeoutExpired:
        return case, "TIMEOUT", None
    if result.returncode != 0:
        return case, f"EXIT {result.returncode}", (result.stderr or result.stdout).strip()[:160]
    match = pattern.match(result.stdout.strip())
    if not match:
        return case, "PARSE", result.stdout.strip()[:160]
    center = mp.mpf(match.group(1))
    radius = abs(mp.mpf(match.group(2)))
    error = abs(center - expected)
    if error <= radius:
        return case, "PASS", None
    ratio = mp.inf if radius == 0 else error / radius
    if ratio <= mp.mpf("1.05"):
        return case, "RADIUS_UNDERSTATED", mp.nstr(ratio, 10)
    return case, "BAD", f"ratio={mp.nstr(ratio, 10)} error={mp.nstr(error, 8)}"

with concurrent.futures.ThreadPoolExecutor(max_workers=8) as executor:
    results = list(executor.map(run, cases))

counts = {}
per_op = {}
for case, status, detail in results:
    counts[status] = counts.get(status, 0) + 1
    per_op.setdefault(case[0], {})[status] = per_op.setdefault(case[0], {}).get(status, 0) + 1
print(f"cases={len(results)} " + " ".join(f"{k}={counts[k]}" for k in sorted(counts)))
for op in sorted(per_op):
    print(op + ": " + " ".join(f"{k}={per_op[op][k]}" for k in sorted(per_op[op])))
shown = 0
for case, status, detail in sorted(results, key=lambda item: (item[1] != "BAD", item[1], item[0][0])):
    if status not in ("PASS", "RADIUS_UNDERSTATED") and shown < 30:
        op, num, den, digits, _ = case
        print(f"{status}: {op}({num}/{den}) digits={digits} {detail or ''}")
        shown += 1
