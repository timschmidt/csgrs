"""Exact independent interval bounds for the signed residual controls."""
from fractions import Fraction as Q
from pathlib import Path
import hashlib, json

def add(a,b): return a[0]+b[0],a[1]+b[1]
def neg(a): return -a[1],-a[0]
def mul(a,b):
    products=[x*y for x in a for y in b]
    return min(products),max(products)
def constant(x): return Q(x),Q(x)

# Alternating sine-series bounds for 0 < x < 1.
atoms=[(Q(1,n)-Q(1,n)**3/6,Q(1,n)) for n in [3,5,7,11]]
a,b,u,d=atoms
six=constant(6); two=constant(2)
a=add(mul(two,a),six);x=add(mul(two,b),six)
z=add(a,mul(constant(Q(1,6)),add(x,neg(a))))
u=mul(two,u);d=add(mul(two,d),six)
y=add(d,mul(constant(Q(1,6)),add(add(u,six),neg(d))))
h=add(mul(z,add(u,neg(six))),neg(mul(add(y,constant(-12)),x)))
assert 6 < z[0] <= z[1] < 7
assert -1 < h[0] <= h[1] < -Q(1,2)
result={'scope':'Rational interval arithmetic plus the alternating sine-series bound; independent of Hyperreal and the polynomial recovery.',
        'atom_intervals':[[str(x) for x in pair] for pair in atoms],
        'z_interval':list(map(str,z)), 'h_interval':list(map(str,h)),
        'residual_identity':'-epsilon*z*h^2',
        'strict_sign':'opposite the sign of nonzero epsilon',
        'script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
p=Path(__file__).with_name('residual-oracle-v1.json');assert not p.exists()
p.write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
