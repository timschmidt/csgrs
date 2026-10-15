"""Independent bounded ring expansion of one captured scalar, for diagnosis only."""
from fractions import Fraction
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
raw = E / 'diagnostic-v2/drill-16-strict.stderr'
value = json.loads(raw.read_text().splitlines()[0].removeprefix('unresolved_scalar='))
visits = 0
atoms = set()


def magnitude(limbs):
    assert len(limbs) <= 128
    assert all(0 <= n < 2**32 for n in limbs)
    return sum(n << (32 * i) for i, n in enumerate(limbs))


def rational(r):
    assert r['sign'] in [-1, 0, 1]
    return Fraction(r['sign'] * magnitude(r['numerator']), magnitude(r['denominator']))


def scale(p, c):
    return {m: v*c for m, v in p.items() if v*c}


def add(p, q):
    out = p.copy()
    for m, v in q.items():
        out[m] = out.get(m, Fraction()) + v
        if not out[m]:
            del out[m]
    assert len(out) <= 1024
    return out


def multiply(p, q):
    assert len(p)*len(q) <= 4096
    out = {}
    for m, v in p.items():
        for n, w in q.items():
            key = tuple(sorted(m+n))
            assert len(key) <= 32
            out[key] = out.get(key, Fraction()) + v*w
    return {m: v for m, v in out.items() if v}


def expand(node):
    global visits
    visits += 1
    assert visits <= 4096
    expr = node['internal']
    if expr == 'One':
        return {(): Fraction(1)}
    assert isinstance(expr, dict) and len(expr) == 1
    op, args = next(iter(expr.items()))
    if op == 'Int':
        return scale({(): Fraction(1)}, args[0]*magnitude(args[1]))
    if op == 'Ratio':
        return scale({(): Fraction(1)}, rational(args))
    if op == 'Add':
        return add(expand(args[0]), expand(args[1]))
    if op == 'Multiply':
        return multiply(expand(args[0]), expand(args[1]))
    if op == 'Negate':
        return scale(expand(args), -1)
    if op == 'Offset':
        assert abs(args[1]) <= 4096
        return scale(expand(args[0]), Fraction(2)**args[1])
    if op == 'Square':
        base = expand(args)
        return multiply(base, base)
    atom = json.dumps(expr, sort_keys=True, separators=(',', ':'))
    atoms.add(atom)
    return {(atom,): Fraction(1)}


assert value['class'] == 'Irrational'
polynomial = scale(expand(value['computable']), rational(value['rational']))
assert polynomial == {}
assert add(polynomial, {(): Fraction(1)}) == {(): Fraction(1)}
print(json.dumps({
    'captured_scalar_sha256': hashlib.sha256(raw.read_bytes()).hexdigest(),
    'script_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    'arithmetic_node_visits': visits,
    'opaque_atoms': sorted(atoms),
    'remaining_terms': len(polynomial),
    'positive_control_add_one': 'exactly 1',
    'scope': 'One captured scalar is zero by rational commutative-ring laws; no trigonometric or square-root relation is assumed. Diagnostic only, not a production algorithm or a proof of the complete Boolean result.',
}, indent=2))
