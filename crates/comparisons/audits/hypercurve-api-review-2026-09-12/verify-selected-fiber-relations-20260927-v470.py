from pathlib import Path
from fractions import Fraction
import json, hashlib, signal, time
A=Path(__file__).resolve().parent
source=A/'verify-later-root-sign-field-20260927-v458.py'
ns={'__file__':str(source)}
src=source.read_text();exec(src[:src.index('def timed_out')],ns)
for name in ('decode','parse','ZERO','ONE','add','neg','mul','inv','trim','product','remainder','evaluate','sign'):
 globals()[name]=ns[name]
p=A/'selected-fiber-input-20260927-v466-queries/query-000.json'
digest=hashlib.sha256(p.read_bytes()).hexdigest();data=json.loads(p.read_text())
P=[decode(c)for c in data['root_polynomial']]
rows=[[[decode(c)for c in row]for row in poly]for poly in data['polynomials']]
R=[decode(c)[0]for c in data['root_interval']];T=[decode(c)[0]for c in data['fiber_interval']]
assert all(decode(c)[1:]==(0,0,0)for c in data['root_interval']+data['fiber_interval'])
assert T[1]<R[0]
print('root_interval',list(map(str,R)),flush=True)
print('fiber_interval',list(map(str,T)),flush=True)
print('display_only',list(map(float,R+T)),flush=True)

def plus(a,b):
 n=max(len(a),len(b));return trim([add(a[i]if i<len(a)else ZERO,b[i]if i<len(b)else ZERO)for i in range(n)])
def times(a,b):return remainder(product(a,b),P)
def sub(poly,N,D):
 degree=max(map(len,poly))-1
 powersN=[[ONE]];powersD=[[ONE]]
 for _ in range(degree):powersN.append(times(powersN[-1],N));powersD.append(times(powersD[-1],D))
 result=[]
 for j in range(degree+1):
  coefficient=trim([row[j]if j<len(row)else ZERO for row in poly])
  result=plus(result,times(coefficient,times(powersN[j],powersD[degree-j])))
 return trim(result)
def q(x):return (Fraction(x),Fraction(0),Fraction(0),Fraction(0))
relations=[('same',[ZERO,ONE],[ONE]),('opposite',[ZERO,q(-1)],[ONE]),('reflected_at_one',[q(2),q(-1)],[ONE]),('reciprocal',[ONE],[ZERO,ONE]),('complement',[ONE,q(-1)],[ONE]),('shifted_down',[q(-1),ONE],[ONE]),('shifted_up',[ONE,ONE],[ONE])]
report={'input_sha256':digest,'retained_interval_disjoint':True,'relations':[],'complete':False}
def timeout(*_):raise TimeoutError('relation oracle timed out')
signal.signal(signal.SIGALRM,timeout);signal.alarm(120)
for name,N,D in relations:
 start=time.monotonic();rs=[sub(poly,N,D)for poly in rows]
 record={'name':name,'both_vanish_modulo_defining':all(not r for r in rs),'remainder_degrees':[len(r)-1 for r in rs],'seconds':time.monotonic()-start}
 report['relations'].append(record);print(record,flush=True)
signal.alarm(0)
report['complete']=True
assert hashlib.sha256(p.read_bytes()).hexdigest()==digest
(A/'selected-fiber-relations-20260927-v470-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
