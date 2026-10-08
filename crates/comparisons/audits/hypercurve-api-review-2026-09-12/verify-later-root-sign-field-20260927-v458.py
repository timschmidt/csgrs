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
signal.signal(signal.SIGALRM,timed_out)
root_counts={};canonical_queries={}
for path in sorted(capture.glob('query-*.json')):
    digest=hashlib.sha256(path.read_bytes()).hexdigest();data=json.loads(path.read_text());start=time.monotonic()
    row=dict(input=path.name,sha256=digest)
    try:
        signal.alarm(180)
        defining=[decode(c) for c in data['defining']];query=[decode(c) for c in data['predicate']]
        endpoints=[decode(c) for c in data['interval']]
        assert all(c[1:]==(0,0,0) for c in endpoints)
        lower,upper=[c[0] for c in endpoints];assert lower<upper
        assert sign(evaluate(defining,lower)) and sign(evaluate(defining,upper))
        root_key=(tuple(defining),lower,upper)
        if root_key not in root_counts:
            root_chain=chain(defining,derivative(defining))
            root_counts[root_key]=variations(root_chain,lower)-variations(root_chain,upper)
        roots=root_counts[root_key];assert roots==data['distinct_root_count']==1
        canonical,leading_sign=monic(query)
        key=(root_key,tuple(canonical));previous=canonical_queries.get(key)
        row['raw_max_bits']=max(max(x.numerator.bit_length(),x.denominator.bit_length())for c in query for x in c)
        row['canonical_max_bits']=max(max(x.numerator.bit_length(),x.denominator.bit_length())for c in canonical for x in c)
        exponents=[abs(x.numerator).bit_length()-x.denominator.bit_length()for c in query for x in c if x]
        row['raw_coordinate_exponent_range']=[min(exponents),max(exponents)]
        if previous is not None:
            result=previous['canonical_sign']*leading_sign
            row['projective_replay_of']=previous['input']
        else:
            query_chain=chain(defining,product(derivative(defining),query))
            result=variations(query_chain,lower)-variations(query_chain,upper)
            canonical_queries[key]=dict(input=path.name,canonical_sign=result*leading_sign)
        assert result in [-1,0,1]
        row.update(root_count=roots,sign=result,complete=True)
    except Exception as error: row.update(complete=False,error=str(error))
    finally:
        signal.alarm(0);assert hashlib.sha256(path.read_bytes()).hexdigest()==digest
        row['elapsed_seconds']=time.monotonic()-start;report['cases'].append(row)
        (A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
        print(row,flush=True)
report['complete']=len(report['cases'])==len(prior['captures']) and all(c['complete'] for c in report['cases'])
report['distinct_root_selections']=len(root_counts);report['distinct_projective_queries']=len(canonical_queries)
(A/f'{prefix}-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
raise SystemExit(0 if report['complete'] else 1)
