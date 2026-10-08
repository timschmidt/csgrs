#!/usr/bin/env python3
import concurrent.futures
import re
import subprocess
import sys

import mpmath as mp

mp.mp.dps = 120

CASES = []

def add(op, values, fn):
    for num, den in values:
        CASES.append((op, num, den, fn(mp.mpf(num) / den)))

basic = [(-7, 3), (-1, 1), (-1, 3), (0, 1), (1, 7), (1, 3), (1, 1), (7, 3)]
positive = [(1, 100), (1, 5), (1, 4), (1, 2), (1, 1), (2, 1), (3, 1), (4, 1), (10, 1)]
unit = [(-1, 1), (-3, 4), (-1, 2), (-1, 1000), (0, 1), (1, 1000), (1, 2), (3, 4), (1, 1)]
open_unit = unit[1:-1]
wide = [(-10, 1), (-3, 1), (-1001, 1000), (-1, 1), (-999, 1000), (0, 1),
        (999, 1000), (1, 1), (1001, 1000), (3, 1), (10, 1)]

add("id", basic, lambda x: x)
add("abs", basic, abs)
add("sqrt", positive, mp.sqrt)
add("exp", wide, mp.exp)
add("log", positive, mp.log)
add("sin", wide, mp.sin)
add("cos", wide, mp.cos)
add("tan", wide, mp.tan)
add("asin", unit, mp.asin)
add("acos", unit, mp.acos)
add("atan", wide, mp.atan)
add("sinh", wide, mp.sinh)
add("cosh", wide, mp.cosh)
add("tanh", wide, mp.tanh)
add("asinh", wide, mp.asinh)
add("acosh", [(1, 1), (1001, 1000), (2, 1), (10, 1)], mp.acosh)
add("atanh", open_unit, mp.atanh)
add("powself", [(1, 10), (1, 2), (1, 1), (2, 1), (5, 2)], lambda x: mp.power(x, x))
add("pythag", [(-10, 1), (-1, 1), (0, 1), (1, 1), (10, 1)], lambda x: mp.mpf(1))
add("logexp", wide, lambda x: x)
add("expsquare", [(-2, 1), (-1, 1), (0, 1), (1, 1), (2, 1)], lambda x: mp.mpf(0))
CASES.extend([("pi", 0, 1, mp.pi), ("e", 0, 1, mp.e)])

PATTERN = re.compile(r"^([^ ]+e[+-]?\d+) \+\-([^ ]+e[+-]?\d+)$")

def check(case):
    op, num, den, expected = case
    command = [sys.argv[1], op, str(num), str(den), "40"]
    try:
        result = subprocess.run(command, text=True, capture_output=True, timeout=4)
    except subprocess.TimeoutExpired:
        return op, num, den, "TIMEOUT", ""
    output = result.stdout.strip()
    if result.returncode != 0:
        detail = (result.stderr.strip() or output).replace("\n", " | ")[:240]
        return op, num, den, f"EXIT {result.returncode}", detail
    match = PATTERN.match(output)
    if not match:
        return op, num, den, "PARSE", output[:240]
    center = mp.mpf(match.group(1))
    radius = abs(mp.mpf(match.group(2)))
    error = abs(center - expected)
    if error > radius:
        return op, num, den, "BAD ENCLOSURE", f"error={mp.nstr(error, 8)} radius={mp.nstr(radius, 8)} output={output}"
    return op, num, den, "PASS", ""

with concurrent.futures.ThreadPoolExecutor(max_workers=8) as executor:
    results = list(executor.map(check, CASES))

counts = {}
for result in results:
    counts[result[3]] = counts.get(result[3], 0) + 1
print(f"cases={len(results)} " + " ".join(f"{key}={counts[key]}" for key in sorted(counts)))
for op, num, den, status, detail in results:
    if status != "PASS":
        print(f"{status}: {op}({num}/{den}) {detail}")
