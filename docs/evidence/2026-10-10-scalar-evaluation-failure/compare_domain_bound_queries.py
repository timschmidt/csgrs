"""Compare the frozen public schedules and verify retained definite facts."""
import collections
import hashlib
import json
from pathlib import Path
import re

E = Path(__file__).resolve().parent
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
caps = read(E / 'domain-bound-candidate-caps-v1693.json')
old = read(E / 'domain-bound-baseline-observations-v1683.json')
assert caps['complete'] and len(caps['tests']) == 9
private, public, controls, retries = [], [], [], []
for entry in caps['tests']:
    assert entry['result']['exit_code'] == entry['cap']['exit_code'] == 0
    assert not entry['cap']['wall_timeout']
    assert entry['result']['source_unchanged'] and entry['result']['main_unchanged']
    assert sha(E / entry['phase'] / 'source-hashes.json.gz') == caps['source_record_sha256']
    for line in (E / entry['phase'] / 'stderr.log').read_text().splitlines():
        if line.startswith('domain bound public '):
            match = re.fullmatch(
                r'domain bound public kind=(\w+), shift=(\d+), squared=(true|false), negative=(true|false), '
                r'state=([\w-]+), units=(\d+), answer=(Ok\(-?\d+\)|Err\(\w+\)), consumed=(\d+), '
                r'before_sign=(None|Some\(\w+\)), sign=(None|Some\(\w+\)), before=(.*), after=(.*), '
                r'allocations=Allocations \{ calls: (\d+), bytes: (\d+), largest: (\d+) \}', line)
            assert match, line
            kind,shift,squared,negative,state,units,answer,consumed,before_sign,sign,cache_before,cache_after,calls,size,largest = match.groups()
            public.append({'kind':kind,'shift':int(shift),'squared':squared=='true','negative':negative=='true',
                'state':state,'units':int(units),'answer':answer,'consumed':int(consumed),
                'before_sign':before_sign,'sign':sign,'cache_before':cache_before,'cache_after':cache_after,
                'allocations':{'calls':int(calls),'bytes':int(size),'largest':int(largest)}})
        elif line.startswith(('domain bound unit ', 'domain bound lower ', 'domain bound required ')):
            assert ('accepted=true, exhausted=false' in line or 'answer=Ok(()), exhausted=false' in line), line
            assert 'calls: 0, bytes: 0, largest: 0' in line
            private.append(line)
        elif line.startswith(('domain bound endpoint ', 'domain bound invalid ', 'domain bound upper-only ')):
            controls.append(line)
        elif line.startswith('domain bound retry '):
            retries.append(line)
assert len(private) == 56 and len(public) == 1584 and len(controls) == 48 and len(retries) == 36
assert controls == old['controls'] and retries == old['retries']
keys = ['kind', 'shift', 'squared', 'negative', 'state', 'units']
gains, differences = [], []
for before, after in zip(old['public'], public):
    key = {k:before[k] for k in keys}
    assert all(before[k] == after[k] for k in keys)
    if before['answer'].startswith('Ok('):
        assert after['answer'] == before['answer'], (key, before, after)
    elif after['answer'].startswith('Ok('):
        gains.append(key | {'before':before['answer'],'after':after['answer']})
    else:
        assert before['answer'] == after['answer'] == 'Err(Exhausted)'
    for field in ['before_sign', 'sign']:
        if before[field] != 'None':
            assert after[field] == before[field], (key, field, before[field], after[field])
    assert after['consumed'] <= after['units']
    changed = {k:{'before':before[k],'after':after[k]} for k in before if before[k] != after[k]}
    if changed:
        differences.append(key | {'changes':changed})
assert len(gains) == 38
assert sum(r['answer'].startswith('Ok(') for r in public) == 453
source = caps['source_record_sha256']
observations = E / 'domain-bound-candidate-observations-v1693.json'
out = E / 'domain-bound-query-comparison-v1693.json'
assert not observations.exists() and not out.exists()
observations.write_text(json.dumps({'source_record_sha256':source,'private':private,'public':public,
                                   'controls':controls,'retries':retries},indent=2)+'\n')
summary = {
    'status':'Every earlier successful sample and definite fact is preserved; 38 public schedules additionally complete',
    'source_record_sha256':source,
    'baseline_observations':'domain-bound-baseline-observations-v1683.json',
    'candidate_observations':observations.name,'candidate_observations_sha256':sha(observations),
    'public_queries':1584,'successful_queries_before_after':[415,453],
    'public_completion_gains':gains,'public_completion_losses':0,
    'changed_previous_successful_samples':0,'lost_or_contradictory_definite_signs':0,
    'candidate_counts_by_kind':{kind:dict(collections.Counter('success' if r['answer'].startswith('Ok(') else 'exhausted'
                                   for r in public if r['kind']==kind)) for kind in ['asin','acos','atanh']},
    'private_domain_queries':56,'cheap_private_successes_before_after':[4,56],
    'all_private_successes_allocate_zero_rust_bytes':True,
    'endpoint_invalid_and_upper_bound_controls_identical':48,
    'fine_retries_identical':36,'zero_work_cached_retry_queries':144,
    'changed_public_rows':differences,
    'limits':'These fixed schedules and directed numerical oracles establish scoped completeness and certificate retention. Other shared constants may evolve after explicit pi warming. The implementation still has authoritative rational/numerical fallbacks and no global byte/time guarantee. General concurrency, reconstruction coverage and main-runtime admission remain open.'
}
out.write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps({'queries':1584,'public_successes_before_after':[415,453],
                  'additional_public_completions':len(gains),'lost_prior_completions':0,
                  'cheap_private_successes_before_after':[4,56]}))
