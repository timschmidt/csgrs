"""Reuse unchanged cost probes and require the preregistered allocation gates."""
from pathlib import Path
W=Path(__file__).resolve().parent
old=W.parent/'containment-winding'
s=(old/'cost_v1.py').read_text()
s=s.replace("OLD=W.parent/'containment-input-validation'","OLD=W.parent/'containment-winding'")
start=s.index('qualified=[')
end=s.index('O.mkdir(exist_ok=False)',start)
s=s[:start]+'''qualified=[OLD/'candidate-validation-v3/result.json',W/'candidate-validation-v1/result.json',W/'wasm-validation-v1/result.json']
for p in qualified:
 r=read(p);assert r['complete'] and not r['unexpected_exits']
assert read(W/'wasm-validation-v1/result.json')['real_js_and_compiled_typescript_consumers_passed']
contract=read(W/'cost-contract-v1.json')
assert contract['pairs']==5
assert all(sha(p)==h for p,h in contract['probe_sha256'].items())
'''+s[end:]
s=s.replace("[('baseline',OLD,'v2'),('candidate',W,'v3')]","[('baseline',OLD,'v3'),('candidate',W,'v1')]")
s=s.replace("'optimization_claimed':False","'production_latency_claimed':False")
needle="dump(O/'result.json',result)"
assert s.count(needle)==1
checks='''
assert counts['baseline']==counts['candidate']=={'certified_completion':3360}
gate_rows=[]
for row in paired:
 assert all(p['same_certified_contract_completed'] for p in row['all_pairs'])
 if row['scope']=='control':
  passed=all(p['allocation_calls_delta']<0 and p['allocated_bytes_delta']<0 for p in row['all_pairs'])
  gate_rows.append({'scope':'control','policy':row['policy'],'phase':row['phase'],'passed':passed})
 elif row['case'] in contract['targeted_matrix_cases'] and row['phase']=='warm_query':
  reductions=[]
  for pair in range(5):
   key={k:row[k] for k in ['case','policy','phase']}
   r={v:next(x for x in reports[f'matrix-{pair:02d}-{v}'] if all(x.get(k)==value for k,value in key.items())) for v in variants}
   reductions.append(1-r['candidate']['allocation_calls']/r['baseline']['allocation_calls'])
  gate_rows.append({'scope':'matrix',**key,'allocation_call_reductions':reductions,'passed':all(r>=contract['required_allocation_call_reduction_on_each_warm_targeted_row'] for r in reductions)})
assert len(gate_rows)==16
result['preregistered_allocation_gates']=gate_rows
result['all_preregistered_allocation_gates_passed']=all(row['passed'] for row in gate_rows)
result['baseline_and_candidate_all_correct_certified']=True
result['claim_scope']='Five matched allocator-instrumented fresh-process pairs; no uninstrumented production speedup claim.'
'''
s=s.replace(needle,checks+'\n'+needle)
s+='\nassert result["all_preregistered_allocation_gates_passed"], "Preregistered allocation gates failed; retain all results and reject this optimization."\n'
(W/'cost_v1.py').write_text(s)
print('Prepared unchanged-probe cost runner with 16 preregistered allocation gates',flush=True)
