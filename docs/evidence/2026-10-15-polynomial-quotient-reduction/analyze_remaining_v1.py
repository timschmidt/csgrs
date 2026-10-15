"""Independent formal algebra for the remaining current 32-segment predicate."""
from pathlib import Path
E = Path(__file__).resolve().parent
previous = E.parent / '2026-10-15-quotient-factor-proofs'
source = (previous / 'analyze_zero_reduction_v1.py').read_text()
exec(source.split('\nrecords = []')[0])
factors = (previous / 'analyze_factor_reduction_v1.py').read_text()
exec(factors[factors.index('\ndef divide_exact'):factors.index('\nExpansion=ReducedExpansion')])
original_multiply = multiply
products = []
def multiply(a, b):
    result = original_multiply(a, b)
    running = {}; peak = 0
    # Native insertion order: lexicographically sorted stored monomials.
    for x, c in sorted(a.items()):
        for y, d in sorted(b.items()):
            term = tuple(sorted(x+y))
            running[term] = running.get(term, Fraction()) + c*d
            if not running[term]: del running[term]
            peak = max(peak, len(running))
    assert running == result
    if peak > 64 or len(result) > 64:
        products.append({'left': rows(a), 'right': rows(b), 'result': rows(result),
                         'peak_stored_terms': peak, 'final_terms': len(result)})
    return result

records = []
for mode in ['chain', 'direct']:
    path = E / f'diagnostic-v3/compose-32-{mode}.stderr'
    captures = [line for line in path.read_text().splitlines() if line.startswith('unresolved_composition_scalar=')]
    assert len(captures) == 1
    parser = Parser(captures[0].split('=', 1)[1]); expr = expression(parser.value())
    assert parser.index == len(parser.tokens)
    trials = []
    for name, cls in [('quotient-expansion', Expansion), ('eager-exact-division', ReducedExpansion)]:
        METRICS.clear(); products.clear()
        expansion = cls(); numerator, denominator = expansion.expand(expr)
        assert denominator
        dag = []
        ids = {value:index for index,value in enumerate(expansion.memo)}
        for value, (n, d) in expansion.memo.items():
            tag,*args=value
            dag.append({'id':ids[value], 'tag':tag,
                'children':[ids.get(child, str(child)) if isinstance(child,tuple) else child for child in args],
                'numerator_terms':len(n), 'denominator_terms':len(d),
                'numerator_degree':max(map(len,n),default=0), 'denominator_degree':max(map(len,d),default=0)})
        trials.append({'name':name, 'zero_numerator':not numerator, 'numerator':rows(numerator),
            'denominator':rows(denominator), 'metrics':METRICS.copy(), 'large_products':list(products),
            'opaque_atoms':expansion.atoms, 'nonzero_denominator_obligations':[rows(p) for p in expansion.inverse_denominators],
            'unique_structural_expansions':expansion.visits, 'calls':expansion.calls, 'dag':dag})
    records.append({'mode':mode,'source_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'trials':trials})
out=E/'remaining-algebra-v1.json';assert not out.exists()
out.write_text(json.dumps({'scope':'Conditional exact rational-function identities only, using diagnostic limits. The production 64-term limit is not raised. No native domain, work, or completed-geometry proof.', 'records':records,'script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()},indent=2)+'\n')
print(json.dumps([{'mode':r['mode'],'trials':[{k:t[k] for k in ['name','zero_numerator','metrics','unique_structural_expansions','calls']} for t in r['trials']]} for r in records],indent=2))
