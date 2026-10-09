"""Independent Fraction oracle for C(4t(1-t)); no Hyper stack dependency."""
from fractions import Fraction as Q
from math import comb
from pathlib import Path
import json
controls=[(Q(1),Q(0),Q(1)),(Q(1),Q(2),Q(1)),(Q(-5,3),Q(8,3),Q(11,3)),(Q(1),Q(2),Q(1)),(Q(1),Q(0),Q(1))]
power=[]
for axis in range(3):
 coefficients=[Q(0) for _ in range(5)]
 for i,point in enumerate(controls):
  for j in range(5-i):
   coefficients[i+j]+=point[axis]*comb(4,i)*comb(4-i,j)*(-1)**j
 power.append(coefficients)
assert power==[[1,0,-16,32,-16],[0,8,-8,0,0],[1,0,16,-32,16]]
def bernstein(t):
 h=[sum(comb(4,i)*t**i*(1-t)**(4-i)*p[k] for i,p in enumerate(controls)) for k in range(3)]
 return h[0]/h[2],h[1]/h[2]
rows=[]
for i in range(33):
 t=Q(i,32);s=4*t*(1-t)
 expected=((1-s*s)/(1+s*s),2*s/(1+s*s));actual=bernstein(t)
 assert 0<=s<=1 and actual==expected and actual==bernstein(1-t)
 assert actual[0]**2+actual[1]**2==1 and actual[0]>=0 and actual[1]>=0
 rows.append(dict(t=str(t),s=str(s),x=str(actual[0]),y=str(actual[1])))
def derivative(first):
 a,b=controls[:2] if first else controls[-2:]
 weight=a[2] if first else b[2]
 return tuple(4*(b[k]*a[2]-a[k]*b[2])/weight**2 for k in range(2))
assert derivative(True)==(0,8) and derivative(False)==(0,-8)
result=dict(identity='s=4t(1-t), C(s)=((1-s^2)/(1+s^2),2s/(1+s^2)). Homogeneous Bernstein controls give this rational parametrization exactly.',domain_proof='For t in [0,1], s in [0,1]; hence x and y are nonnegative. C(t) repeats under t -> 1-t, with equal endpoints and opposite endpoint derivatives. The image cannot be a full circle, and one directed native arc cannot preserve its out-and-back traversal.',first_derivative=['0','8'],last_derivative=['0','-8'],green_integral_proof='The two halves have the same image and opposite traversal, so their oriented line integrals cancel exactly.',samples=rows)
result['homogeneous_power_coefficients']=[[str(value) for value in row] for row in power]
Path(__file__).with_name('oracle.json').write_text(json.dumps(result,indent=2)+'\n')
print('Exact polynomial coefficients, 33 samples, and both endpoint derivatives verified')
