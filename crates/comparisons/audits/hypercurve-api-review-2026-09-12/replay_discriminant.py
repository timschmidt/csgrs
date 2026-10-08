"""Independently replay the extracted exact scalar with rational arithmetic.

Requires SymPy; run beside discriminant-scalar.json. The input contains exact
expression nodes only, with no floating point samples or refinement caches.
"""
from collections import Counter
import json
from pathlib import Path

import sympy as sym

source = json.loads(Path(__file__).with_name("discriminant-scalar.json").read_text())
memo = {}
kinds = Counter()
radicands = []


def integer(words):
    return sum(int(word) << (32 * index) for index, word in enumerate(words))


def rational(value):
    return sym.Rational(
        value["sign"] * integer(value["numerator"]), integer(value["denominator"])
    )


def evaluate(node):
    key = json.dumps(node, separators=(",", ":"))
    if key in memo:
        return memo[key]
    kind, argument = next(iter(node["internal"].items()))
    kinds[kind] += 1
    if kind == "Ratio":
        result = rational(argument)
    elif kind == "Int":
        result = sym.Integer(argument[0] * integer(argument[1]))
    elif kind == "Add":
        result = evaluate(argument[0]) + evaluate(argument[1])
    elif kind == "Multiply":
        result = evaluate(argument[0]) * evaluate(argument[1])
    elif kind == "Negate":
        result = -evaluate(argument)
    elif kind == "Offset":
        result = evaluate(argument[0]) * sym.Integer(2) ** argument[1]
    elif kind == "Inverse":
        result = sym.radsimp(1 / evaluate(argument))
    elif kind == "Square":
        result = evaluate(argument) ** 2
    elif kind == "Sqrt":
        child = evaluate(argument)
        radicands.append(child)
        result = sym.sqrt(child)
    else:
        raise ValueError(f"Unsupported exact expression node: {kind}")
    memo[key] = sym.expand(result)
    return memo[key]


value = sym.simplify(rational(source["rational"]) * evaluate(source["computable"]))
assert value == 0
print("Exact discriminant coefficient:", value)
print("Distinct structural nodes:", len(memo), dict(kinds))
print("Square-root radicands:", radicands)
