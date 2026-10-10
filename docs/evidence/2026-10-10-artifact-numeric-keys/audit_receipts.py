"""Audit all ordinary public-API receipts, including nonnumeric findings."""
import json
from pathlib import Path
p=Path(__file__).resolve().parent
load=lambda kind:{r['id']:r for r in map(json.loads,(p/kind/'forward.stdout').read_text().splitlines())}
a,b=load('baseline'),load('candidate-corpus')
classes=['centroid places multiple references at ','BOM rotation for ','appears with multiple rotations: ','at multiple centroid rotations (']
changed=[]; unexpected=[]; diagnostics=[]
for key in a:
 old,new=a[key],b[key]
 def category(v):
  message=v.get('message','')
  return next((text for text in classes if text in message),None)
 keep=lambda row:[v for v in row['violations'] if category(v) is None]
 if keep(old)!=keep(new): unexpected.append(key)
 numeric=lambda row:[v for v in row['violations'] if category(v) is not None]
 if numeric(old)!=numeric(new): changed.append({'id':key,'before':numeric(old),'after':numeric(new)})
 if old['diagnostics']!=new['diagnostics']:diagnostics.append({'id':key,'before':old['diagnostics'],'after':new['diagnostics']})
assert not unexpected
assert all(not row['after'] and all(d['code'].endswith('::invalid-number') and ('mm' in d['message'] or 'deg' in d['message']) for d in row['before']) for row in diagnostics)
result={'numeric_receipt_changes':changed,'parser_diagnostic_changes':diagnostics,'unrelated_differences':unexpected,'interpretation':'All ordinary-corpus violation changes are numeric decision or exact message changes. Parser deltas only remove erroneous invalid-number warnings for already-supported mm/deg suffixes.'}
serialized=json.dumps(result,indent=2)+'\n'
assert (p/'candidate-corpus/differential-audit.json').read_text()==serialized
print('All receipt differences classified:',len(changed),'numeric;',len(diagnostics),'unit-suffix diagnostics.')
