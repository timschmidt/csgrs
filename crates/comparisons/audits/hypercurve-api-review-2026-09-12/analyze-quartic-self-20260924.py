"""Independent rational algebra for the closed quartic offset fixture."""
import json
import time
from pathlib import Path
import sympy as s

t, u = s.symbols('t u')
x = 12*t*(1-t)**3 - 12*t**3*(1-t)
y = 18*t**2*(1-t)**2
p = s.Matrix([x, y])
q = p.subs(t, u)
hp = p.diff(t)
hq = q.diff(u)
delta = q-p
cross = hp[0]*hq[1]-hp[1]*hq[0]
first = s.Poly(s.expand(hp.dot(hp)*delta.dot(hq)**2-cross**2/16), t, u)
second = s.Poly(s.expand(hq.dot(hq)*delta.dot(hp)**2-cross**2/16), t, u)
start = time.monotonic()
report = {'source': [str(s.factor(v)) for v in p], 'original_degrees': [first.degree_list(), second.degree_list()]}
residuals = []
for equation in [first, second]:
    content, factors = s.factor_list(equation)
    print('factor', content, [(str(f.as_expr()), n) for f, n in factors], flush=True)
    diagonal_power = next(n for f,n in factors if f == s.Poly(t-u,t,u))
    residual = s.div(equation, s.Poly((t-u)**diagonal_power, t,u))[0]
    residuals.append(residual.primitive()[1])
report['residual_degrees'] = [r.degree_list() for r in residuals]
print('resultant start', report, flush=True)
resultant = s.Poly(s.resultant(residuals[0].as_expr(), residuals[1].as_expr(), u), t)
content, factors = s.factor_list(resultant)
report['resultant_degree'] = resultant.degree()
report['resultant_factors'] = [(str(f.as_expr()), n) for f, n in factors]
report['elapsed_seconds'] = time.monotonic()-start
print(json.dumps(report, indent=2), flush=True)
Path(__file__).with_suffix('.json').write_text(json.dumps(report, indent=2)+'\n')
