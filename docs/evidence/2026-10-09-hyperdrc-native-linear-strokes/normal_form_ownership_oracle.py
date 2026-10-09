"""Independent Fraction algebra for the six public normal-form probe families.

Monomials are pi^p * a^q * sqrt(2)^r * sqrt(3)^s, r,s in {0,1}.
The atom a is formal; its value is irrelevant to the cancellation identities.
Division is only by a nonzero rational*pi monomial, except the explicitly
verified conjugate reciprocal of 1+sqrt(2). No Hyper implementation is imported.
"""
from fractions import Fraction as F
import json
from pathlib import Path
import sys

ONE = (0, 0, 0, 0)
def constant(n):
    return {} if n == 0 else {ONE: F(n)}
def add(a, b):
    result = a.copy()
    for key, value in b.items():
        result[key] = result.get(key, F(0)) + value
        if not result[key]:
            del result[key]
    return result
def scale(a, n):
    return {key: value*n for key, value in a.items() if value*n}
def sub(a, b):
    return add(a, scale(b, -1))
def mul(a, b):
    result = {}
    for (p,q,r,s), x in a.items():
        for (t,u,v,w), y in b.items():
            key=(p+t,q+u,(r+v)%2,(s+w)%2)
            result=add(result,{key:x*y*2**((r+v)//2)*3**((s+w)//2)})
    return result
def monomial_inverse(a):
    assert len(a)==1
    (p,q,r,s),x=next(iter(a.items()))
    assert q==r==s==0 and x
    return {(-p,0,0,0):1/x}

one=constant(1);pi={(1,0,0,0):F(1)}
root2={(0,0,1,0):F(1)};root3={(0,0,0,1):F(1)}
atom={(0,1,0,0):F(1)}
reciprocal=sub(root2,one)
assert mul(add(one,root2),reciprocal)==one
rows=[]
for integer in range(1,8):
    k=constant(integer)
    two_radicals=add(root2,mul(k,root3))
    third=scale(pi,F(1,3));shifted=sub(pi,one)
    pi2=mul(pi,pi)
    transported=mul(mul(mul(pi,sub(shifted,third)),pi),monomial_inverse(pi2))
    sum_roots=add(root2,root3)
    angle_shifted=add(atom,pi)
    angle_transport=mul(mul(mul(pi,sub(pi,angle_shifted)),pi),monomial_inverse(pi2))
    expressions=[
        sub(sub(sub(mul(two_radicals,two_radicals),constant(2)),scale(mul(k,k),3)),scale(mul(k,mul(root2,root3)),2)),
        mul(sub(add(reciprocal,one),root2),k),
        mul(sub(add(scale(transported,-1),shifted),third),k),
        sub(mul(mul(mul(sum_roots,pi),monomial_inverse(mul(k,pi))),k),sum_roots),
        mul(sub(scale(angle_transport,-1),atom),k),
        add(sub(mul(add(k,root2),sub(k,root2)),mul(k,k)),constant(2)),
    ]
    for kind,value in enumerate(expressions):
        for offset in (-1,0,1):
            assert add(value,constant(offset))==constant(offset)
            rows.append(dict(family=kind,k=integer,offset=offset,expected_rational=str(F(offset))))
out=Path(sys.argv[1])
out.write_text(json.dumps(dict(scope=__doc__,conjugate_reciprocal_verified=True,identities=len(rows),rows=rows),indent=2)+'\n')
print(len(rows),'exact Fraction identities verified')
