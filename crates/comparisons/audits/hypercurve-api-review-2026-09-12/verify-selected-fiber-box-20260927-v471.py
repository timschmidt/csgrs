from pathlib import Path
from fractions import Fraction as F
from math import comb
import json,hashlib,time,signal
A=Path(__file__).resolve().parent
s=A/'verify-owned-root-newton-oracle-20260927-v459.py';ns={'__file__':str(s)}
v=s.read_text();exec(v[:v.index('signal.signal(signal.SIGALRM')],ns)
for key in ('decode','ZERO','add','mul','derivative','field_bounds','polynomial_bounds','iadd','ineg','imul','idiv','outward','width_bits'):
 globals()[key]=ns[key]
p=A/'selected-fiber-input-20260927-v466-queries/query-000.json';digest=hashlib.sha256(p.read_bytes()).hexdigest();data=json.loads(p.read_text())
P=[decode(c)for c in data['root_polynomial']]
C,Q=[[[decode(c)for c in row]for row in poly]for poly in data['polynomials']]
R=tuple(decode(c)[0]for c in data['root_interval']);T=tuple(decode(c)[0]for c in data['fiber_interval'])
report={'input_sha256':digest,'steps':[],'complete':False,'common_root_excluded':False}
def timeout(*_):raise TimeoutError('interval oracle timeout')
signal.signal(signal.SIGALRM,timeout);signal.alarm(120)
def scalar(c,r):return tuple(x*r for x in c)
def shift(poly,r,t):
 n=len(poly);m=max(map(len,poly));result=[[ZERO for _ in range(m)]for _ in range(n)]
 for i,row in enumerate(poly):
  for j,c in enumerate(row):
   if c==ZERO:continue
   for u in range(i+1):
    for v in range(j+1):result[u][v]=add(result[u][v],scalar(c,comb(i,u)*comb(j,v)*r**(i-u)*t**(j-v)))
 return result
def boxes(poly,bits):return [[field_bounds(c,bits)for c in row]for row in poly]
def bieval(poly,r,t):return polynomial_bounds([polynomial_bounds(row,t)for row in poly],r)
def sign(box):return 1 if box[0]>0 else -1 if box[1]<0 else 0
for centered in [False,True]:
 r,t=R,T;rc=(R[0]+R[1])/2 if centered else F(0);tc=(T[0]+T[1])/2 if centered else F(0)
 started=time.monotonic();c=shift(C,rc,tc)if centered else C;q=shift(Q,rc,tc)if centered else Q
 print('centered',centered,'shift_seconds',round(time.monotonic()-started,3),flush=True)
 for bits in [64,128,256,512,1024,2048]:
  pbox=[field_bounds(c,bits)for c in P];dp=[field_bounds(c,bits)for c in derivative(P)]
  cb=boxes(c,bits);qb=boxes(q,bits)
  dc=[[tuple(x*j for x in value)for j,value in enumerate(row)if j]for row in cb]
  for step in range(3):
   d=polynomial_bounds(dp,r)
   if sign(d):
    mid=(r[0]+r[1])/2;image=outward(iadd((mid,mid),ineg(idiv(polynomial_bounds(pbox,(mid,mid)),d))),bits)
    r=(max(r[0],image[0]),min(r[1],image[1]));assert r[0]<=r[1]
   rr=(r[0]-rc,r[1]-rc);tt=(t[0]-tc,t[1]-tc)
   value=bieval(qb,rr,tt);cv=bieval(cb,rr,tt)
   record={'centered':centered,'bits':bits,'step':step,'base_width_bits':width_bits(r),'fiber_width_bits':width_bits(t),'predicate_sign':sign(value),'incidence_sign':sign(cv)}
   report['steps'].append(record);print(record,flush=True)
   if sign(value) or sign(cv):
    report.update(complete=True,common_root_excluded=True);break
   d=bieval(dc,rr,tt)
   if not sign(d):break
   mid=(t[0]+t[1])/2;cm=bieval(cb,rr,(mid-tc,mid-tc))
   image=outward(iadd((mid,mid),ineg(idiv(cm,d))),bits)
   t=(max(t[0],image[0]),min(t[1],image[1]))
   if t[0]>t[1]:report.update(complete=True,common_root_excluded=True);break
  if report['complete']:break
 if report['complete']:break
signal.alarm(0);report['complete']=True
assert hashlib.sha256(p.read_bytes()).hexdigest()==digest
(A/'selected-fiber-box-20260927-v471-terminal.json').write_text(json.dumps(report,indent=2)+'\n')
print('common_root_excluded',report['common_root_excluded'],flush=True)
