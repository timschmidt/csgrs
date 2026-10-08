from pathlib import Path
from fractions import Fraction
import json,time
import sympy as s

audit=Path(__file__).resolve().parent
directory=audit/'nonph-contact-isolation-20260923-trace1-dumps'
def fraction(value):
    if ' ' not in value: return Fraction(value)
    whole,part=value.split(' ')
    return (-1 if whole.startswith('-') else 1)*(abs(Fraction(whole))+Fraction(part))
def rat(value):
    q=fraction(value); return s.Rational(q.numerator,q.denominator)
rows=[]; t,u=s.symbols('t u'); summaries=[]
for path in sorted(directory.glob('*.json')):
    row=json.loads(path.read_text()); rows.append(row)
    values=[fraction(x) for x in row['base']]+[fraction(x) for c in row['incidence'] for x in c]
    summary=dict(name=path.name,base_degree=len(row['base'])-1,shape=[len(row['incidence']),max(map(len,row['incidence']))],
                 coefficient_bits=max(max(abs(x.numerator).bit_length(),x.denominator.bit_length()) for x in values))
    print(summary,flush=True); summaries.append(summary)
print('fibers 0/1 identical',rows[0]==rows[1],flush=True)
print('bases 0/2 identical',rows[0]['base']==rows[2]['base'],flush=True)
base=s.Poly(sum(rat(c)*t**i for i,c in enumerate(rows[0]['base'])),t)
start=time.monotonic(); content,factors=s.factor_list(base)
lower,upper=map(rat,rows[0]['base_interval'])
base_factors=[]
for polynomial,multiplicity in factors:
    entry=dict(degree=polynomial.degree(),multiplicity=multiplicity,
               selected_roots=int(polynomial.count_roots(lower,upper)),coefficients=[str(c) for c in reversed(polynomial.all_coeffs())])
    base_factors.append(entry)
    print('base factor', {k:v for k,v in entry.items() if k!='coefficients'},flush=True)
print('base factor seconds',time.monotonic()-start,flush=True)
incidences=[]
for index in [0,2]:
    polynomial=s.Poly(sum(rat(c)*t**i*u**j for i,row in enumerate(rows[index]['incidence']) for j,c in enumerate(row)),t,u)
    start=time.monotonic(); content,factors=s.factor_list(polynomial); entry=[]
    for factor,multiplicity in factors:
        info=dict(degrees=list(factor.degree_list()),multiplicity=multiplicity,terms=len(factor.terms()),expression=str(factor.as_expr()))
        entry.append(info); print('incidence',index,'factor',{k:v for k,v in info.items() if k!='expression'},flush=True)
    incidences.append(dict(index=index,factors=entry,elapsed_seconds=time.monotonic()-start))
result=dict(summaries=summaries,first_two_fibers_identical=rows[0]==rows[1],base_factors=base_factors,incidences=incidences)
(audit/'nonph-contact-isolation-20260923-factor-analysis.json').write_text(json.dumps(result,indent=2)+'\n')
