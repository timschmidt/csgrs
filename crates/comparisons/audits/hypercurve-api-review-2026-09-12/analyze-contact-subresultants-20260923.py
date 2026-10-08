from pathlib import Path
import json,time
import sympy as s
from fractions import Fraction
p=Path('/tmp/hypercurve-contact-isolation-2026-09-23/hypersolve/examples/contact_fiber_probe_20260923.json')
d=json.loads(p.read_text()); t,u=s.symbols('t u')
f=sum(s.Rational(c)*t**i*u**j for i,row in enumerate(d['incidence']) for j,c in enumerate(row))
p=s.Poly(sum(s.Rational(c)*t**i for i,c in enumerate(d['base'])),t)
start=time.monotonic(); sub=s.subresultants(f,s.diff(f,u),u)
print('subresultants',time.monotonic()-start,flush=True)
rows=[]
for r in sub:
    poly=s.Poly(r,u)
    coefs=[s.Poly(c,t).rem(p) for c in poly.all_coeffs()]
    selected=s.Poly(sum(s.Rational(c)*t**i for i,c in enumerate(json.loads(Path('nonph-contact-isolation-20260923-factor-analysis.json').read_text())['base_factors'][2]['coefficients'])),t)
    reduced=[c.rem(selected) for c in coefs]
    row=dict(degree=poly.degree(),base_degree=max(s.Poly(c,t).degree() for c in poly.all_coeffs()),selected_degree=max([poly.degree()-i for i,c in enumerate(reduced) if not c.is_zero],default=-1),max_bits=max(max(abs(int(v.p)).bit_length(),int(v.q).bit_length()) for c in coefs for v in c.all_coeffs()))
    print(row,flush=True); rows.append(row)
Path('contact-fiber-20260923-subresultants-analysis.json').write_text(json.dumps(dict(seconds=time.monotonic()-start,rows=rows),indent=2)+'\n')
