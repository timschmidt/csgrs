"""Compare every preserved budget replay receipt, retaining all observed differences."""
import hashlib
import json
from pathlib import Path
import sys
sys.set_int_max_str_digits(20000)
E=Path(__file__).resolve().parent
read=lambda p:json.loads(p.read_text())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
j=read(E/'domain-bound-replay-orchestration-v1745.json');assert j['complete']
changes=[];unreviewed=[];cases=0

def describe(value):
 if isinstance(value,str) and len(value)>256:
  return {'string_bytes':len(value.encode()),'sha256':hashlib.sha256(value.encode()).hexdigest()}
 return value

def compare(a,b,path,context):
 if a==b:return
 if isinstance(a,dict) and isinstance(b,dict):
  assert a.keys()==b.keys(),(context,path)
  for k in a:
   if k=='command':continue
   compare(a[k],b[k],path+[k],context)
  return
 if isinstance(a,list) and isinstance(b,list):
  assert len(a)==len(b)
  for i,(x,y) in enumerate(zip(a,b)):compare(x,y,path+[str(i)],context)
  return
 leaf=path[-1]
 row=context|{'path':'.'.join(path),'before':describe(a),'after':describe(b)}
 if leaf in ['consumed','remaining']:row['classification']='work-accounting change'
 elif leaf in ['answer','before_sign','after_sign'] and a is None and b in ['Positive','Negative','Zero']:
  row['classification']='additional certified sign'
 elif leaf in ['before_sample','sample'] and a is None and b is not None:
  row['classification']='additional numeric sample checked by replay oracle'
 elif 'sample' in leaf and isinstance(a,str) and isinstance(b,str) and abs(int(a)-int(b))<=2:
  row['classification']='changed representative within independently checked one-unit error bounds'
 else:
  row['classification']='requires review';unreviewed.append(row)
 changes.append(row)

for r in j['results']:
 old,new=E/r['baseline'],E/r['phase']
 a,b=read(old/'receipts.json'),read(new/'receipts.json')
 assert len(a)==len(b)==r['expected_cases']
 for x,y in zip(a,b):
  assert x['name']==y['name'] and x['command']['command'][1:]==y['command']['command'][1:]
  assert x['complete'] and y['complete'] and y['command']['exit_code']==0
  compare(x,y,[],{'family':r['family'],'case':x['name']})
 cases+=len(a)
assert cases==1584
out=E/'domain-bound-budget-replay-comparison-v1745.json';assert not out.exists()
out.write_text(json.dumps({'source_record_sha256':j['source_record_sha256'],'cases':cases,
 'status':'All non-accounting observations identical or reviewed as strengthening' if not unreviewed else 'Observed differences require review',
 'changes':changes,'unreviewed':unreviewed,
 'limits':'Each frozen replay independently checks its own mathematical oracle and retained zero-work proofs. This comparison also requires every previously definite sign to remain identical; numeric representatives may differ only within the independently checked one-unit contract. Work charges remain implementation-defined.'},indent=2)+'\n')
print(json.dumps({'cases':cases,'changed_fields':len(changes),'unreviewed':len(unreviewed),
 'classifications':{k:sum(r['classification']==k for r in changes) for k in sorted({r['classification'] for r in changes})}}))
assert not unreviewed,unreviewed[:3]
