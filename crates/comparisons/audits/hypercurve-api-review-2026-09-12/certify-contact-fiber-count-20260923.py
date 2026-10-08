from pathlib import Path
from fractions import Fraction as Q
import json,sympy as s,time
root=Path('/tmp/hypercurve-contact-isolation-2026-09-23/hypersolve')
audit=Path(__file__).resolve().parent
start=time.monotonic()
d=json.loads((root/'tests/data/nonph_fillet_circle_contact.json').read_text())
t,u=s.symbols('t u')
f=sum(s.Rational(c)*t**i*u**j for i,row in enumerate(d['incidence']) for j,c in enumerate(row))
p=s.Poly(sum(s.Rational(c)*t**i for i,c in enumerate(d['base'])),t)
selected=s.Poly(sum(s.Rational(c)*t**i for i,c in enumerate(json.loads((audit/'nonph-contact-isolation-20260923-factor-analysis.json').read_text())['base_factors'][2]['coefficients'])),t)
assert p.rem(selected).is_zero
lo,hi=map(Q,d['base_interval'])
assert p.count_roots(s.Rational(lo),s.Rational(hi))==1
assert selected.count_roots(s.Rational(lo),s.Rational(hi))==1

def sign(expression):
    poly=s.Poly(expression,t)
    if poly.rem(selected).is_zero: return 0
    a=b=Q(0)
    for c in poly.all_coeffs():
        cs=Q(int(c.p),int(c.q))
        products=[a*lo,a*hi,b*lo,b*hi]
        a,b=min(products)+cs,max(products)+cs
    if a>0:return 1
    if b<0:return -1
    raise AssertionError('isolator does not separate sign')
sequence=s.subresultants(f,s.diff(f,u),u)
leading=[]; left=[]; right=[]
for i,r in enumerate(sequence):
    poly=s.Poly(r,u)
    assert poly.degree()==8-i
    if all(sign(c)==0 for c in poly.all_coeffs()):break
    leading.append(sign(poly.LC())); assert leading[-1]!=0
    factor=-1 if (i//2)%2 else 1
    left.append(factor*sign(r.subs(u,0)));right.append(factor*sign(r.subs(u,1)))
def variations(signs):
    signs=[x for x in signs if x]
    return sum(a!=b for a,b in zip(signs,signs[1:]))
count=variations(left)-variations(right); assert count==1
assert left[0]!=0 and right[0]!=0
result=dict(source_root_count=1,selected_factor_degree=selected.degree(),leading_signs=leading,lower_sturm_signs=left,upper_sturm_signs=right,distinct_root_count=count,seconds=time.monotonic()-start)
(audit/'contact-fiber-20260923-independent-count.json').write_text(json.dumps(result,indent=2)+'\n')
print(result)
