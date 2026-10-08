from pathlib import Path
import json,time
import sympy as s

audit=Path(__file__).resolve().parent
facts=json.loads((audit/'nonph-contact-isolation-20260923-factor-analysis.json').read_text())
source=json.loads((audit/'nonph-contact-isolation-20260923-trace1-dumps/fiber-2.json').read_text())
def rat(text):
    if ' ' not in text: return s.Rational(text)
    whole,fraction=text.split(' ')
    return (-1 if whole.startswith('-') else 1)*(abs(s.Rational(whole))+s.Rational(fraction))
t,u=s.symbols('t u'); factor=next(x for x in facts['base_factors'] if x['selected_roots']==1)
base=s.Poly(sum(s.Rational(c)*t**i for i,c in enumerate(factor['coefficients'])),t)
field=s.QQ.alg_field_from_poly(base,alias='alpha',root_index=0)
print('field ready',field.ext.minpoly.degree(),flush=True)
coefficients=[]
for j in range(max(map(len,source['incidence']))):
    row=[s.QQ.convert(rat(c[j]) if j<len(c) else 0) for c in source['incidence']]
    coefficients.append(field(list(reversed(row))))
polynomial=s.Poly.from_list(list(reversed(coefficients)),u,domain=field)
start=time.monotonic(); print('gcd start degree',polynomial.degree(),flush=True)
gcd=polynomial.gcd(polynomial.diff())
elapsed=time.monotonic()-start
print('gcd degree',gcd.degree(),'seconds',elapsed,flush=True)
quotient=polynomial.exquo(gcd)
print('square-free quotient degree',quotient.degree(),flush=True)
def encode(poly):
    return [[str(c) for c in reversed(coefficient.to_list())] for coefficient in reversed(poly.rep.to_list())]
result=dict(gcd_degree=gcd.degree(),gcd_seconds=elapsed,gcd=encode(gcd),quotient=encode(quotient),
            base=factor['coefficients'])
(audit/'nonph-contact-isolation-20260923-field-analysis.json').write_text(json.dumps(result,indent=2)+'\n')
