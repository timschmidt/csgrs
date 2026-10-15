"""Independent conditional algebra; no native domain or resource proof."""
from pathlib import Path

E = Path(__file__).resolve().parent
previous = E.parent / '2026-10-15-quotient-factor-proofs'
zero_source = (previous / 'analyze_zero_reduction_v1.py').read_text()
exec(zero_source.split('\nrecords = []')[0])
factor_source = (previous / 'analyze_factor_reduction_v1.py').read_text()
exec(factor_source[factor_source.index('\ndef divide_exact'):factor_source.index('\nExpansion=ReducedExpansion')])
unlimited_multiply = multiply


class DegreeLimit(Exception):
    pass


def limited_multiply(a, b):
    if a and b and max(map(len, a)) + max(map(len, b)) > 8:
        METRICS['degree_declines'] = METRICS.get('degree_declines', 0) + 1
        raise DegreeLimit()
    return unlimited_multiply(a, b)


class RetryExpansion(Expansion):
    def expand_uncached(self, expr):
        try:
            return super().expand_uncached(expr)
        except DegreeLimit:
            tag, *args = expr
            if tag not in ('Add', 'Multiply', 'Square'):
                raise
            children = args if tag != 'Square' else [args[0], args[0]]
            if any(child not in self.memo for child in children):
                raise
            reduced = []
            changed = False
            for child in children:
                numerator, denominator = self.memo[child]
                quotient = None if denominator == constant(1) else divide_exact(numerator, denominator)
                if quotient is not None:
                    numerator, denominator = quotient, constant(1)
                    changed = True
                reduced.append((numerator, denominator))
            if not changed:
                raise
            METRICS['retries'] = METRICS.get('retries', 0) + 1
            (a, b), (c, d) = reduced
            if tag == 'Add':
                if not a:
                    return c, d
                if not c:
                    return a, b
                if b == d:
                    return add(a, c), b
                numerator = add(multiply(a, d), multiply(c, b))
                return (numerator, multiply(b, d)) if numerator else ({}, constant(1))
            if not a or not c:
                return {}, constant(1)
            return multiply(a, c), multiply(b, d)


records = []
for mode in ['chain', 'direct']:
    for segments in [16, 32]:
        path = E / f'diagnostic-v2/compose-{segments}-{mode}.stderr'
        captures = [line for line in path.read_text().splitlines()
                    if line.startswith('unresolved_composition_scalar=')]
        assert len(captures) == (3 if mode == 'chain' else 1), (path, len(captures))
        for capture_index, capture in enumerate(captures):
            parser = Parser(capture.split('=', 1)[1])
            expr = expression(parser.value())
            assert parser.index == len(parser.tokens)
            trials = []
            for name, implementation, product in [
                ('zero-normalization', Expansion, unlimited_multiply),
                ('eager-exact-division', ReducedExpansion, unlimited_multiply),
                ('division-only-after-degree-decline', RetryExpansion, limited_multiply),
            ]:
                METRICS.clear()
                multiply = product
                expansion = implementation()
                try:
                    numerator, denominator = expansion.expand(expr)
                    assert denominator
                    trials.append({'kind': name, 'expanded': True, 'zero_numerator': not numerator,
                                   'numerator': rows(numerator), 'denominator': rows(denominator)})
                except DegreeLimit:
                    trials.append({'kind': name, 'expanded': False})
                trials[-1].update({'metrics': METRICS.copy(), 'unique_structural_expansions': expansion.visits,
                                  'calls': expansion.calls, 'opaque_atoms': expansion.atoms,
                                  'nonzero_denominator_obligations': [rows(p) for p in expansion.inverse_denominators]})
            records.append({'segments': segments, 'mode': mode, 'capture_index': capture_index, 'trials': trials,
                            'source_sha256': hashlib.sha256(path.read_bytes()).hexdigest()})
out = E / 'current-algebra-v1.json'
assert not out.exists()
out.write_text(json.dumps({'scope': 'Conditional rational-function identities only; inverse operands need native nonzero proofs. Structural memoization and diagnostic operation counts are not production Arc identity or budget evidence.',
                          'records': records, 'script_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}, indent=2) + '\n')
print(json.dumps([{'segments': r['segments'], 'mode': r['mode'],
                   'trials': [{k: t[k] for k in ['kind', 'expanded', 'metrics']} for t in r['trials']]}
                  for r in records], indent=2))
