"""Preserve complete, incomplete and supplemental domain-sign cost evidence separately."""
import hashlib
import json
from pathlib import Path
import statistics
E=Path(__file__).resolve().parent
ROOT=E.parents[2]
read=lambda p:json.loads(p.read_text())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
source=read(E/'domain-sign-retaining-caps-v1621.json')['source_record_sha256']
old={
 'atan':'typed-api-prototype-v1514-atan-zero-term-baseline-cost',
 'integer':'typed-api-prototype-v1357-integer-scale-cost',
 'inverse_trig':'typed-api-prototype-v1358-integer-scale-inverse-cost',
 'normal_form':'typed-api-prototype-v1359-integer-scale-normal-form-cost',
 'default':'typed-api-prototype-v1360-integer-scale-default-cost',
 'algebraic':'typed-api-prototype-v1361-integer-scale-algebraic-cost',
 'acos':'typed-api-prototype-v1418-acos-warm-baseline-cost'}
standalone=read(E/'domain-sign-cost-orchestration-v1637.json');assert standalone['completed']
assert standalone['source_record_sha256']==source
families={};total=0
for row in standalone['results']:
 family=row['family'];before=E/old[family];after=E/row['phase']
 assert all(sha(before/f)==sha(after/f) for f in ['probe.rs','reproduce.py'])
 commands=lambda p:[r for r in read(p/'commands.json') if r['name'] not in ['format','build']]
 a,b=commands(before),commands(after);assert len(a)==len(b)==row['processes'];total+=len(b)
 for x,y in zip(a,b):
  assert x['name']==y['name'] and x['command'][1:]==y['command'][1:]
  va,vb=read(before/(x['name']+'.stdout')),read(after/(y['name']+'.stdout'))
  assert {k:v for k,v in va.items() if k!='elapsed_ns'}=={k:v for k,v in vb.items() if k!='elapsed_ns'}
 binaries={'baseline':Path(a[0]['command'][0]),'candidate':Path(b[0]['command'][0])}
 families[family]={'phases':{'baseline':before.name,'candidate':after.name},
  'processes':len(b),'non_timing_outputs_identical':True,
  'binary_sha256':{k:sha(p) for k,p in binaries.items()},'binary_bytes':{k:p.stat().st_size for k,p in binaries.items()}}
assert total==468
partial=E/'domain-sign-preconditioned-cost-v1638'
assert not (partial/'summary.json').exists()
rows=read(partial/'receipts.json');assert len(rows)==867
failed=read(partial/'failed-attempt.json')
assert failed['name']=='algebraic-19-baseline' and failed['exit_code']==-24 and not failed['wall_timeout']
assert failed['cpu_seconds_soft_hard']==[2,3] and failed['wall_seconds_limit']==4
assert failed['binary_sha256']==families['algebraic']['binary_sha256']['baseline']
samples={}
for r in rows:
 family,index,which=r['name'].rsplit('-',2)
 assert r['binary_sha256']==families[family]['binary_sha256'][which] and r['exit_code']==0
 key=(family,tuple(r['command'][1:]))
 samples.setdefault(key,{'baseline':[],'candidate':[]})[which].append(int(r['receipt']['elapsed_ns']))
partial_cases=[]
for (family,args),values in samples.items():
 counts={k:len(v) for k,v in values.items()}
 medians={k:statistics.median(v) for k,v in values.items() if v}
 partial_cases.append({'family':family,'arguments':list(args),'samples_per_side':counts,'median_ns':medians,
  'complete_nine_pairs':all(n==9 for n in counts.values()),
  'median_percent_change':100*(medians['candidate']/medians['baseline']-1) if len(medians)==2 else None})
assert sum(r['complete_nine_pairs'] for r in partial_cases)==48
focused=read(E/'domain-sign-focused-cost-v1639/summary.json')
algebraic=read(E/'domain-sign-algebraic-cost-v1639/summary.json')
for data in [focused,algebraic]:
 assert data['source_record_sha256']==source and data['guards_unchanged']
 assert all(sha(Path(p))==v for f in data['provenance'].values() for p,v in [(f['binaries'][k],h) for k,h in f['binary_sha256_at_paired_run'].items()])
assert focused['processes']==324 and focused['configuration']['cpu_seconds']==[2,3]
assert algebraic['processes']==108 and algebraic['configuration']['cpu_seconds']==[4,5]
wide=next(r for r in focused['cases']['integer'] if r['arguments'][0]=='wide_coarse')
assert wide['median_percent_change']>5
out=E/'domain-sign-cost-review-v1639.json';assert not out.exists()
out.write_text(json.dumps({
 'status':'Cost evidence for an isolated completeness extension; original fixed-CPU run remains incomplete',
 'source_record_sha256':source,'original_candidate_processes':total,'families':families,
 'original_fixed_cpu_run':{'phase':partial.name,'completed_processes':len(rows),'prelude_processes':len(read(partial/'prelude-receipts.json')),'failed_attempt':failed,'case_statistics':partial_cases,'complete':False},
 'focused_original_cap_followup':{'phase':'domain-sign-focused-cost-v1639','processes':324,'cases':focused['cases']},
 'separate_larger_cap_diagnostic':{'phase':'domain-sign-algebraic-cost-v1639','processes':108,'cases':algebraic['cases'],'cpu_seconds':[4,5],'wall_seconds':8,'passes_original_two_second_guard':False},
 'persistent_integer_cost':{'arguments':wide['arguments'],'median_percent_change':wide['median_percent_change'],'median_ns':wide['median_ns'],
  'median_ns_per_call':{k:v/10000 for k,v in wide['median_ns'].items()}},
 'interpretation':[
  'The repeated wide-coarse integer control costs 18.76 percent more: 48.73 to 57.87 nanoseconds per call in the 27-repetition follow-up. Its output and executable inputs are unchanged. No causal attribution to code placement is established.',
  'The first-run mid-negative acos and structural-cancellation increases do not persist in the predeclared 27-repetition follow-up. Keep both observations.',
  'All six algebraic fixtures agree on exact non-timing outputs under equal larger process caps. This is a separate timing diagnostic, not a passed original cap guard.',
  'The original interrupted process is the retained baseline. Its final receipt was absent; the interrupted stage is unknown. Comparable completed runs spend most wall time outside the timed query loop.',
  'The extension resolves eight previously exhausted public coarse queries at unchanged allowance and preserves previously available child proofs. The governing completeness-before-performance order permits this measured lower-rung tradeoff in the isolated checkpoint; it is not retained as a speed or memory-only optimization.',
  'Main runtime admission, full-board performance and any general speedup claim remain open. Completion of the replay gates is required before the final checkpoint record.'
 ],
 'all_successful_cost_processes_including_standalone':total+len(rows)+324+108,
 'separate_cpu_preparation_processes':len(read(partial/'prelude-receipts.json'))+324+108
},indent=2)+'\n')
print(json.dumps({'record':out.name,'successful_cost_processes':1767,'retained_baseline_sigxcpu':True,'persistent_integer_percent':wide['median_percent_change']}))
