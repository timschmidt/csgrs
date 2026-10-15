"""Conditional independent polynomial-division experiment, not a native proof."""
from pathlib import Path
E=Path(__file__).resolve().parent
source=(E/'analyze_zero_reduction_v1.py').read_text()
exec(source.split('\nrecords = []')[0])


def divide_exact(numerator, denominator):
    assert denominator
    METRICS['division_attempts']=METRICS.get('division_attempts',0)+1
    remainder=numerator.copy(); quotient={}
    leading=max(denominator,key=lambda term:(len(term),term))
    while remainder:
        METRICS['division_steps']=METRICS.get('division_steps',0)+1
        assert METRICS['division_steps'] <= 8192
        term=max(remainder,key=lambda term:(len(term),term))
        factors=list(term)
        for atom in leading:
            if atom not in factors: return None
            factors.remove(atom)
        factor={tuple(factors): remainder[term]/denominator[leading]}
        quotient=add(quotient,factor)
        product=multiply(factor,denominator)
        remainder=add(remainder,{key:-value for key,value in product.items()})
    METRICS['division_successes']=METRICS.get('division_successes',0)+1
    return quotient


class ReducedExpansion(Expansion):
    def expand(self,expr):
        self.calls += 1; assert self.calls <= 16384
        if expr in self.memo: return self.memo[expr]
        numerator,denominator=self.expand_uncached(expr)
        if not numerator: denominator=constant(1)
        elif denominator != constant(1):
            reduced=divide_exact(numerator,denominator)
            if reduced is not None: numerator,denominator=reduced,constant(1)
        result=numerator,denominator
        self.memo[expr]=result
        return result


Expansion=ReducedExpansion
exec('records = []'+source.split('\nrecords = []')[1].replace('zero-reduction-algebra-v1.json','factor-reduction-algebra-v1.json'))
