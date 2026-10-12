"""Record measured exhaustion and unchanged-runtime controls before repair."""
import collections
import gzip
import hashlib
import json
from pathlib import Path
import re
import sys

sys.dont_write_bytecode = True
from parse_test_results_v3 import parse_phase

E = Path(__file__).resolve().parent
SOURCE = E.parents[2] / 'target/scalar-evaluation-candidate/source'
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
caps = read(E / 'domain-bound-baseline-caps-v1682.json')
assert caps['complete'] and len(caps['tests']) == 9
assert all(r['result']['exit_code'] == r['expected_exit_code'] == r['cap']['exit_code']
           and not r['cap']['wall_timeout'] for r in caps['tests'])
unit = E / 'typed-api-prototype-v1683-domain-bound-baseline-tests'
result = read(unit / 'result.json')
assert result['exit_code'] == 0 and result['source_unchanged'] and result['main_unchanged']
assert sha(unit / 'source-hashes.json.gz') == caps['source_record_sha256']
tests = parse_phase(unit)
assert tests['top_level_passed'] == 1071 and tests['child_process_passed'] == 1
assert tests['top_level_failed'] == tests['child_process_failed'] == 0
before = json.loads(gzip.decompress((E / 'typed-api-prototype-v1624-domain-sign-release-build/source-hashes.json.gz').read_bytes()))
source = json.loads(gzip.decompress((E / caps['source_record']).read_bytes()))
changed = [n for n,h in source.items() if before.get(n) != h]
assert set(changed) == {'crates/hyperreal/src/computable/node.rs',
                       'crates/hyperreal/src/computable/node/log_range_probe_tests.rs',
                       'crates/hyperreal/src/computable/node/domain_bound_tests.rs'}
for name,digest in source.items():
    assert sha(SOURCE / name) == digest
old_dir = E / 'typed-api-prototype-v1624-domain-sign-release-build/inputs'
node = 'crates/hyperreal/src/computable/node.rs'
assert (SOURCE / node).read_text() == (old_dir / node).read_text() + 'include!("node/domain_bound_tests.rs");\n'
log = 'crates/hyperreal/src/computable/node/log_range_probe_tests.rs'
old_comment = ('        // A public smooth-log expression warms the shared ln(2)/ln(3)\n'
               '        // constants. Neither the measured root nor its children are reused.\n')
new_comment = ('        // This public ln(4/3) call takes the direct rational ln1p path. It\n'
               '        // does not establish that shared ln(2)/ln(3) caches are warm. The\n'
               '        // measured roots are fresh; shared caches evolve in the test order.\n')
old_text = (old_dir / log).read_text()
assert old_text.count(old_comment) == 1
assert (SOURCE / log).read_text() == old_text.replace(old_comment, new_comment)

private_rows = []
public_rows = []
controls = []
retries = []
for entry in caps['tests']:
    stderr = (E / entry['phase'] / 'stderr.log').read_text()
    for line in stderr.splitlines():
        if line.startswith('domain bound public '):
            match = re.fullmatch(
                r'domain bound public kind=(\w+), shift=(\d+), squared=(true|false), negative=(true|false), '
                r'state=([\w-]+), units=(\d+), answer=(Ok\(-?\d+\)|Err\(\w+\)), consumed=(\d+), '
                r'before_sign=(None|Some\(\w+\)), sign=(None|Some\(\w+\)), before=(.*), after=(.*), '
                r'allocations=Allocations \{ calls: (\d+), bytes: (\d+), largest: (\d+) \}', line)
            assert match, line
            kind,shift,squared,negative,state,units,answer,consumed,before_sign,sign,cache_before,cache_after,calls,size,largest = match.groups()
            public_rows.append({'kind':kind,'shift':int(shift),'squared':squared=='true','negative':negative=='true',
                'state':state,'units':int(units),'answer':answer,'consumed':int(consumed),
                'before_sign':before_sign,'sign':sign,'cache_before':cache_before,'cache_after':cache_after,
                'allocations':{'calls':int(calls),'bytes':int(size),'largest':int(largest)}})
        elif line.startswith(('domain bound unit ', 'domain bound lower ', 'domain bound required ')):
            private_rows.append({'line':line,'complete':('accepted=true, exhausted=false' in line
                                                        or 'answer=Ok(()), exhausted=false' in line)})
        elif line.startswith(('domain bound endpoint ', 'domain bound invalid ', 'domain bound upper-only ')):
            controls.append(line)
        elif line.startswith('domain bound retry '):
            retries.append(line)
assert len(private_rows) == 56 and sum(not r['complete'] for r in private_rows) == 52
assert len(controls) == 48 and len(retries) == 36
assert all('first=Err(Exhausted)' in r for r in retries)
assert len(public_rows) == 1584
assert sum(r['answer'].startswith('Ok(') for r in public_rows) == 415
assert sum(r['answer'] == 'Err(Exhausted)' for r in public_rows) == 1169
observations = E / 'domain-bound-baseline-observations-v1683.json'
out = E / 'domain-bound-baseline-v1683.json'
assert not observations.exists() and not out.exists()
observations.write_text(json.dumps({'source_record_sha256':caps['source_record_sha256'],
                                  'private':private_rows,'public':public_rows,'controls':controls,'retries':retries},indent=2)+'\n')
summary = {
    'status':'Production unchanged from v1672; measured domain-bound proof-order exhaustion and all negative controls frozen before repair',
    'previous_checkpoint':'typed-api-prototype-v1672-state.json',
    'source_record':caps['source_record'],'source_record_sha256':caps['source_record_sha256'],
    'changed_sources_sha256':{name:source[name] for name in changed},
    'old_test_behavior_unchanged':True,
    'comment_correction':'Only the misleading log shared-constant warmup comment changed. Every old test expression, ordering and assertion is preserved.',
    'cap_record':'domain-bound-baseline-caps-v1682.json',
    'observations_record':observations.name,'observations_sha256':sha(observations),
    'private_domain_observations':56,'private_cheap_completions':4,'private_avoidable_exhaustions':52,
    'invalid_and_endpoint_controls':48,
    'public_queries':1584,'public_successes':415,'public_exhaustions':1169,
    'public_counts_by_kind':{kind:dict(collections.Counter('success' if r['answer'].startswith('Ok(') else 'exhausted'
                                for r in public_rows if r['kind']==kind)) for kind in ['asin','acos','atanh']},
    'unlimited_fine_retries':36,'zero_work_retry_queries':144,
    'baseline_units':tests,
    'proposed_change':'Consult only existing root magnitude facts before bounded rational recognition, preserving independent existence proofs and all cold fallbacks. An upper magnitude below zero proves the open unit interval; a positive exact binade at or above zero proves at least one. An upper estimate never supplies that lower bound.',
    'limits':[
        'Private finite/domain helpers are diagnostics; the public survey uses ordinary asin/acos/atanh construction, public approximation budgets and public serialization round-trips.',
        'The lower-bound diagnostic explicitly establishes child existence through the internal validator; no native public domain-certification API is claimed.',
        'Survey cases run in fixed order with pi prewarmed to -512. Other shared constants may evolve; these are not globally cold-process comparisons.',
        'MPFR supplies independent directed enclosures for every successful public result and fine retry. Exhaustions are not numerical correctness observations.',
        'No production method changed. No byte/time budget, universal completeness or main-runtime admission claim follows. All original hard fixtures remain unchanged.',
        'Main runtime and all 49 preexisting changes remain preserved. All 39 properties, 14 targets and crate future work remain in scope.'
    ]
}
out.write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps({k:summary[k] for k in ['status','source_record_sha256','private_avoidable_exhaustions','public_successes','public_exhaustions']}))
