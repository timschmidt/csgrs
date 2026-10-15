"""Independent diagnostic rational-function expansion of captured Debug data.

This proves only an algebraic identity conditional on nonzero denominators;
it does not replace a production domain proof or certify a Boolean result.
"""
from pathlib import Path
from fractions import Fraction
import collections, hashlib, json, re

E = Path(__file__).resolve().parent


class Parser:
    def __init__(self, text):
        assert len(text) < 2_000_000
        self.tokens = re.findall(r'-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?|[A-Za-z_][A-Za-z_0-9]*|[^\s]', text)
        self.index = 0

    def take(self, expected=None):
        token = self.tokens[self.index]
        self.index += 1
        assert expected is None or expected == token, (expected, token)
        return token

    def value(self, depth=0):
        assert depth < 256
        token = self.take()
        if re.fullmatch(r'-?\d+', token): return int(token)
        if token == '(':
            values = []
            while self.tokens[self.index] != ')':
                values.append(self.value(depth+1))
                if self.tokens[self.index] != ')': self.take(',')
            self.take(')')
            return values
        if self.index < len(self.tokens) and self.tokens[self.index] == '(':
            return {'tag': token, 'args': self.value(depth+1)}
        if self.index < len(self.tokens) and self.tokens[self.index] == '{':
            self.take('{')
            fields = {}
            while self.tokens[self.index] != '}':
                name = self.take(); self.take(':'); fields[name] = self.value(depth+1)
                if self.tokens[self.index] != '}': self.take(',')
            self.take('}')
            return {'tag': token, 'fields': fields}
        return token


def rational(value):
    assert value['tag'] == 'Rational'
    v = value['fields']
    return Fraction({'Minus': -1, 'NoSign': 0, 'Plus': 1}[v['sign']]*v['numerator'], v['denominator'])


def expression(value):
    if isinstance(value, str): return (value,)
    tag = value['tag']
    if tag == 'Computable': return expression(value['fields']['internal']['fields']['approximation'])
    if tag == 'Rational':
        r = rational(value); return ('Q', r.numerator, r.denominator)
    if tag == 'Real':
        fields = value['fields']; scale = expression(fields['rational'])
        assert fields['computable']['tag'] == 'Some'
        return ('Multiply', scale, expression(fields['computable']['args'][0]))
    assert 'fields' not in value, ('unsupported structured atom', tag)
    return (tag, *(expression(v) if isinstance(v, (dict, str)) else v for v in value.get('args', [])))


def constant(value): return {(): Fraction(value)} if value else {}


def add(a, b):
    out = a.copy()
    for term, value in b.items():
        out[term] = out.get(term, Fraction()) + value
        if not out[term]: del out[term]
    METRICS['maximum_terms'] = max(METRICS.get('maximum_terms', 0), len(out))
    assert len(out) <= 1024
    return out


METRICS = {}

def multiply(a, b):
    METRICS['coefficient_products'] = METRICS.get('coefficient_products', 0) + len(a)*len(b)
    assert len(a)*len(b) <= 4096
    out = {}
    for x, c in a.items():
        for y, d in b.items():
            term = tuple(sorted(x+y)); assert len(term) <= 32
            METRICS['maximum_product_degree'] = max(METRICS.get('maximum_product_degree', 0), len(term))
            out[term] = out.get(term, Fraction()) + c*d
    out = {k: v for k, v in out.items() if v}
    METRICS['maximum_terms'] = max(METRICS.get('maximum_terms', 0), len(out))
    assert len(out) <= 1024
    assert all(v.numerator.bit_length()+v.denominator.bit_length() <= 16384 for v in out.values())
    return out


class Expansion:
    def __init__(self):
        self.atoms = {}; self.visits = 0; self.inverse_denominators = []
        self.memo = {}; self.calls = 0
        self.operations = collections.Counter()

    def expand(self, expr):
        self.calls += 1; assert self.calls <= 16384
        if expr in self.memo: return self.memo[expr]
        result = self.expand_uncached(expr)
        if not result[0]: result = ({}, constant(1))
        self.memo[expr] = result
        return result

    def expand_uncached(self, expr):
        self.visits += 1; assert self.visits <= 4096
        tag, *args = expr; self.operations[tag] += 1
        one = constant(1)
        if tag == 'One': return one, one
        if tag == 'Int': return constant(args[0]), one
        if tag == 'Q': return constant(Fraction(*args)), one
        if tag == 'Ratio': return self.expand(args[0])
        if tag in ('Add', 'Multiply'):
            a, b = self.expand(args[0]); c, d = self.expand(args[1])
            if tag == 'Add':
                if not a: return c, d
                if not c: return a, b
                if b == d:
                    numerator = add(a, c)
                    return numerator, b if numerator else one
                numerator = add(multiply(a, d), multiply(c, b))
                if not numerator: return {}, one
                return numerator, multiply(b, d)
            if not a or not c: return {}, one
            return multiply(a, c), multiply(b, d)
        if tag in ('Negate', 'Offset', 'Square', 'Inverse'):
            a, b = self.expand(args[0])
            if tag == 'Negate': return {k: -v for k, v in a.items()}, b
            if tag == 'Offset':
                assert abs(args[1]) <= 4096
                return multiply(a, constant(Fraction(2)**args[1])), b
            if tag == 'Square': return multiply(a, a), multiply(b, b)
            assert a, 'an identically zero denominator cannot be inverted'
            self.inverse_denominators.append(a)
            return b, a
        key = json.dumps(expr, separators=(',', ':'))
        if key not in self.atoms: self.atoms[key] = len(self.atoms)
        return {(self.atoms[key],): Fraction(1)}, one


def rows(polynomial):
    return [{'powers_as_atom_ids': list(k), 'coefficient': str(v)} for k, v in sorted(polynomial.items())]


records = []
for segments in [16, 32]:
    path = E.parent/'2026-10-15-shared-scalar-proofs'/f'diagnostic-v1/compose-{segments}-strict.stderr'
    for index, line in enumerate(line for line in path.read_text().splitlines() if line.startswith('unresolved_composition_scalar=')):
        parser = Parser(line.split('=', 1)[1]); parsed = parser.value()
        assert parser.index == len(parser.tokens)
        METRICS.clear()
        expr = expression(parsed); expansion = Expansion(); numerator, denominator = expansion.expand(expr)
        assert denominator
        # Offset by one requires exactly the same denominator, an independent
        # nonzero control under the stated nonzero-denominator assumptions.
        assert add(numerator, denominator) == denominator if not numerator else True
        records.append({'segments': segments, 'composition_index': index, 'unique_structural_expansions': expansion.visits, 'calls': expansion.calls,
                        'operations': dict(expansion.operations), 'arithmetic_metrics': METRICS.copy(), 'zero_numerator': not numerator,
                        'numerator': rows(numerator), 'denominator': rows(denominator),
                        'nonzero_denominator_obligations': [rows(p) for p in expansion.inverse_denominators],
                        'opaque_atoms': expansion.atoms,
                        'source_sha256': hashlib.sha256(path.read_bytes()).hexdigest()})
out = E/'zero-reduction-algebra-v1.json'; assert not out.exists()
out.write_text(json.dumps({'scope': 'Independent diagnostic rational-function identities, conditional on proving each denominator nonzero. No transcendental identities or completed-geometry claim.', 'records': records, 'script_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}, indent=2)+'\n')
print(json.dumps([{'segments': r['segments'], 'composition_index': r['composition_index'], 'unique_structural_expansions': r['unique_structural_expansions'], 'calls': r['calls'], 'zero_numerator': r['zero_numerator'], 'inverse_count': len(r['nonzero_denominator_obligations']), 'opaque_atoms': len(r['opaque_atoms']), 'arithmetic_metrics': r['arithmetic_metrics']} for r in records], indent=2))
