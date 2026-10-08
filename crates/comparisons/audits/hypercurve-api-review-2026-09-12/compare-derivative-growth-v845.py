from pathlib import Path
import json,statistics,hashlib
A=Path(__file__).resolve().parent
pairs=[('derivative-growth-replay-v837','derivative-growth-replay-v842'),('derivative-domain-replay-v838','derivative-domain-replay-v843'),('high-order-derivative-replay-v840','high-order-derivative-replay-v844')]
for before,after in pairs:
 for prefix in [before,after]:
  r=json.loads((A/f'{prefix}-terminal.json').read_text());reap=json.loads((A/f'{prefix}-reaped.json').read_text())
  assert reap['outer_exit_code']==0 and r['all_processes_reaped']and r['probe_complete']
  assert all(p['returncode']==0 for p in r['processes'])
 assert json.loads((A/f'{before}-terminal.json').read_text())['source_sha256']==json.loads((A/f'{after}-terminal.json').read_text())['source_sha256']
def rows(prefix):
 result={}
 for line in(A/f'{prefix}-run.log').read_text().splitlines():
  if not line.startswith('policy='):continue
  row=dict(p.split('=',1)for p in line.split());assert row.pop('exact')=='true'
  row={k:int(v)for k,v in row.items()};key=tuple(row[k]for k in ['policy','degree','order','sample']);assert key not in result;result[key]=row
 return result
before=rows(pairs[0][0]);after=rows(pairs[0][1]);assert set(before)==set(after)and len(before)==36
for key,row in before.items():
 assert row['max_image_denominator']==after[key]['max_image_denominator']==2
 assert all(after[key][field]>0 for field in ['ns','allocations','requested_bytes'])
groups=[]
for group in sorted({k[:3]for k in before}):
 samples=[k for k in before if k[:3]==group];assert len(samples)==3
 row=dict(policy=group[0],degree=group[1],order=group[2],samples=3)
 for field in ['ns','allocations','requested_bytes']:
  old=statistics.median(before[k][field]for k in samples);new=statistics.median(after[k][field]for k in samples)
  row[field]=dict(before=old,after=new,ratio=new/old)
 groups.append(row)
old_domain=(A/f'{pairs[1][0]}-run.log').read_text();new_domain=(A/f'{pairs[1][1]}-run.log').read_text();assert old_domain==new_domain and old_domain.count('correct=true')==4
def high_rows(prefix):
 result={}
 for line in(A/f'{prefix}-run.log').read_text().splitlines():
  if not line.startswith('degree='):continue
  row=dict(p.split('=',1)for p in line.split());assert row.pop('exact')=='true'
  row={k:int(v)for k,v in row.items()};key=(row['degree'],row['order']);assert key not in result;result[key]=row
 return result
high_before=high_rows(pairs[2][0]);high_after=high_rows(pairs[2][1]);assert set(high_before)==set(high_after)=={(8,16),(40,80)}
high=[]
for key,old in high_before.items():
 row=dict(degree=key[0],order=key[1],samples=1)
 for field in ['ns','allocations','requested_bytes']:
  row[field]=dict(before=old[field],after=high_after[key][field],ratio=high_after[key][field]/old[field])
 high.append(row)
sources={}
for pair in pairs:
 for prefix in pair:
  p=A/f'{prefix}-run.log';sources[p.name]=hashlib.sha256(p.read_bytes()).hexdigest()
report=dict(independent_exact_samples=36,domain_cases=4,groups=groups,high_order_groups=high,source_logs=sources,scope='Same standalone source linked to normal release libraries before and after the selected-root derivative recurrence change. Fresh curves and parameters; three samples per policy/degree/order, all eight derivative values checked independently. Allocator counters measure Rust allocation/reallocation requests and requested bytes, not peak memory or native-library allocations. Timings include allocator instrumentation and are workload-local, not a uniform speedup claim.')
(A/'derivative-growth-comparison-v845.json').write_text(json.dumps(report,indent=2)+'\n')
for row in groups:print('policy',row['policy'],'degree',row['degree'],'order',row['order'],'allocation ratio',round(row['allocations']['ratio'],3),'byte ratio',round(row['requested_bytes']['ratio'],3),'instrumented time ratio',round(row['ns']['ratio'],3),flush=True)

for row in high:print("High-order",row,flush=True)
