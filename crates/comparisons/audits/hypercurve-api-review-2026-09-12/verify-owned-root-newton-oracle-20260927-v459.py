from pathlib import Path
from fractions import Fraction as F
from functools import reduce
from math import gcd, lcm
import hashlib, json, signal, time

A = Path(__file__).resolve().parent
prefix = 'later-root-sign-field-20260927-v458'
capture = A / 'later-root-sign-input-20260927-v457-queries'
prior = json.loads((A / 'later-root-sign-input-20260927-v457-terminal.json').read_text())
assert prior['all_processes_reaped'] and prior['capture_complete']
ZERO = (F(0),) * 4
ONE = (F(1), F(0), F(0), F(0))
report = dict(cases=[], complete=False, method='Independent exact Q(sqrt(5),sqrt(50-20sqrt(5))) arithmetic with positive leading-unit normalization; exact projective query comparisons')

def qadd(a, b): return tuple(x+y for x,y in zip(a,b))
def qneg(a): return tuple(-x for x in a)
def qmul(a, b): return (a[0]*b[0]+5*a[1]*b[1], a[0]*b[1]+a[1]*b[0])
def qinv(a):
    norm=a[0]*a[0]-5*a[1]*a[1]
    assert norm
    return (a[0]/norm, -a[1]/norm)
def add(a,b): return tuple(x+y for x,y in zip(a,b))
def neg(a): return tuple(-x for x in a)
def mul(a,b):
    even=qadd(qmul(a[:2],b[:2]),qmul(qmul(a[2:],b[2:]),(F(50),F(-20))))
    odd=qadd(qmul(a[:2],b[2:]),qmul(a[2:],b[:2]))
    return even+odd
def inv(a):
    norm=qadd(qmul(a[:2],a[:2]),qneg(qmul(qmul(a[2:],a[2:]),(F(50),F(-20)))))
    inverse=qinv(norm)
    return qmul(a[:2],inverse)+qneg(qmul(a[2:],inverse))
def sign_rational(x): return (x>0)-(x<0)
def qsign(a):
    first,second=map(sign_rational,a)
    if not second: return first
    if not first or first==second: return second
    order=sign_rational(a[0]*a[0]-5*a[1]*a[1])
    return first if order>0 else second if order<0 else 0
def sign(a):
    first,second=qsign(a[:2]),qsign(a[2:])
    if not second: return first
    if not first or first==second: return second
    difference=qadd(qmul(a[:2],a[:2]),qneg(qmul(qmul(a[2:],a[2:]),(F(50),F(-20)))))
    order=qsign(difference)
    return first if order>0 else second if order<0 else 0
assert qsign((F(50),F(-20)))==1

def parse(value):
    if ' ' not in value: return F(value)
    sign=-1 if value.startswith('-') else 1
    whole,fraction=value.lstrip('-+').split()
    return sign*(F(whole)+F(fraction))

def decode(parts):
    assert parts is not None
    def quadratic(row):
        a,b,d=map(parse,row)
        assert not b or d==5
        return (a,b)
    even,odd=quadratic(parts[0]),quadratic(parts[1])
    assert odd==(0,0) or tuple(map(parse,parts[2]))==(50,-20,5)
    return even+odd
def trim(p):
    while p and p[-1]==ZERO: p.pop()
    return p
def primitive(p):
    values=[x for c in p for x in c]
    denominator=lcm(*(x.denominator for x in values))
    integers=[x.numerator*(denominator//x.denominator) for x in values]
    content=reduce(gcd,integers,0)
    if not content: return []
    return trim([tuple(F(x//content) for x in integers[i:i+4]) for i in range(0,len(integers),4)])
def derivative(p): return [tuple(x*i for x in c) for i,c in enumerate(p) if i]
def product(a,b):
    result=[ZERO]*(len(a)+len(b)-1)
    for i,x in enumerate(a):
        for j,y in enumerate(b): result[i+j]=add(result[i+j],mul(x,y))
    return trim(result)
def remainder(a,b):
    a=list(a);inverse=inv(b[-1])
    while len(a)>=len(b):
        leading=mul(a.pop(),inverse);start=len(a)+1-len(b)
        for i,c in enumerate(b[:-1]): a[start+i]=add(a[start+i],neg(mul(leading,c)))
        trim(a)
    return a
def unit_normalize(p):
    p=trim(list(p))
    if not p:return []
    scale=inv(p[-1])
    if sign(p[-1])<0:scale=neg(scale)
    return [mul(c,scale)for c in p]
def monic(p):
    p=trim(list(p))
    if not p:return [],0
    leading_sign=sign(p[-1]);scale=inv(p[-1])
    return [mul(c,scale)for c in p],leading_sign
def chain(a,b):
    a=unit_normalize(a);b=trim(list(b))
    if len(b)>=len(a):b=remainder(b,a)
    b=unit_normalize(b);result=[a]
    while b:
        result.append(b)
        bits=max(max(x.numerator.bit_length(),x.denominator.bit_length())for c in b for x in c)
        print('unit remainder degree',len(b)-1,'maximum bits',bits,flush=True)
        a,b=b,unit_normalize([neg(c)for c in remainder(a,b)])
    return result
def evaluate(p,x):
    value=ZERO
    for c in reversed(p): value=add(tuple(v*x for v in value),c)
    return value
def variations(chain,x):
    signs=[s for p in chain if (s:=sign(evaluate(p,x)))]
    return sum(a!=b for a,b in zip(signs,signs[1:]))
def timed_out(_signum,_frame): raise TimeoutError('exact field oracle time limit')

from math import isqrt
prefix='owned-root-newton-oracle-20260927-v459'
authority=json.loads((A/'later-root-sign-field-20260927-v458-terminal.json').read_text());assert authority['complete']
report=dict(cases=[],complete=False,method='Exact rational interval Newton on owned roots, outward dyadic rounding, independently certified quartic coefficient enclosures')
def iadd(a,b):return (a[0]+b[0],a[1]+b[1])
def ineg(a):return (-a[1],-a[0])
def imul(a,b):
    values=[x*y for x in a for y in b];return (min(values),max(values))
def idiv(a,b):
    assert b[0]>0 or b[1]<0
    return imul(a,(1/b[1],1/b[0]))
def rational_sqrt_bounds(lower,upper,bits):
    assert 0<=lower<=upper
    scale=1<<bits
    lo=isqrt((lower.numerator<<(2*bits))//lower.denominator)
    hi=isqrt((upper.numerator<<(2*bits))//upper.denominator)+1
    return F(lo,scale),F(hi,scale)
bases={}
def field_bounds(value,bits):
    if bits not in bases:
        s=rational_sqrt_bounds(F(5),F(5),bits)
        r=iadd((F(50),F(50)),imul((F(-20),F(-20)),s))
        t=rational_sqrt_bounds(*r,bits)
        bases[bits]=[(F(1),F(1)),s,t,imul(s,t)]
    result=(F(0),F(0))
    for coefficient,basis in zip(value,bases[bits]):result=iadd(result,imul((coefficient,coefficient),basis))
    return result
def polynomial_bounds(coefficients,interval):
    value=(F(0),F(0))
    for c in reversed(coefficients):value=iadd(imul(value,interval),c)
    return value
def outward(interval,bits):
    scale=1<<bits;a,b=interval
    lower=(a.numerator*scale)//a.denominator
    upper=-((-b.numerator*scale)//b.denominator)
    return F(lower,scale),F(upper,scale)
def width_bits(interval):
    w=interval[1]-interval[0]
    return None if not w else w.denominator.bit_length()-w.numerator.bit_length()
signal.signal(signal.SIGALRM,timed_out)
for proof in authority['cases']:
    path=capture/proof['input'];assert hashlib.sha256(path.read_bytes()).hexdigest()==proof['sha256']
    data=json.loads(path.read_text());started=time.monotonic();row=dict(input=path.name,sha256=proof['sha256'],expected_sign=proof['sign'],steps=[])
    try:
        signal.alarm(120)
        defining=[decode(c)for c in data['defining']];query=[decode(c)for c in data['predicate']]
        endpoints=[decode(c)for c in data['interval']];assert all(c[1:]==(0,0,0)for c in endpoints)
        owned=tuple(c[0]for c in endpoints);interval=owned;decided=None
        for bits in [512,1024,2048,4096,8192]:
            p=[field_bounds(c,bits)for c in defining]
            dp=[field_bounds(c,bits)for c in derivative(defining)]
            q=[field_bounds(c,bits)for c in query]
            for iteration in range(2):
                derivative_range=polynomial_bounds(dp,interval)
                if derivative_range[0]<=0<=derivative_range[1]:break
                midpoint=sum(interval)/2
                residual=polynomial_bounds(p,(midpoint,midpoint))
                contracted=outward(iadd((midpoint,midpoint),ineg(idiv(residual,derivative_range))),bits)
                interval=(max(interval[0],contracted[0]),min(interval[1],contracted[1]))
                assert owned[0]<=interval[0]<=interval[1]<=owned[1]
            q_range=polynomial_bounds(q,interval)
            decided=1 if q_range[0]>0 else -1 if q_range[1]<0 else None
            row['steps'].append(dict(precision=bits,width_bits=width_bits(interval),sign=decided))
            if decided is not None:break
        assert decided==proof['sign'],('query not separated',path.name)
        row.update(sign=decided,complete=True)
    except Exception as error:row.update(complete=False,error=str(error))
    finally:
        signal.alarm(0);row['elapsed_seconds']=time.monotonic()-started;report['cases'].append(row)
        (A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n');print(row,flush=True)
report['complete']=len(report['cases'])==len(authority['cases'])and all(c['complete']for c in report['cases'])
(A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
raise SystemExit(0 if report['complete']else 1)
