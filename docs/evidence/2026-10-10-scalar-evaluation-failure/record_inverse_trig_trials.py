#!/usr/bin/env python3
"""Retain focused inverse-trig results, including unresolved ordinary costs."""
import gzip
import hashlib
import json
from pathlib import Path
import subprocess

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT/'target/scalar-evaluation-candidate/source'
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
record = E/'typed-api-prototype-v748-inverse-trig-replay-build/source-hashes.json.gz'
expected = 'd54ed906ab99c845956e05470eb5226faa36420248a6abfb30623cbc772e3e97'
assert sha(record) == expected
prior = read(E/'typed-api-prototype-v729-state.json')
old = json.loads(gzip.decompress((E/prior['source_record']).read_bytes()))
current = json.loads(gzip.decompress(record.read_bytes()))
assert all(sha(SOURCE/n) == h for n,h in current.items())
changed = {n: {'before':old.get(n),'after':h} for n,h in current.items() if old.get(n) != h}
assert len(changed) == 5
assert all(old[n] == current[n] for n in old if 'test' in n)
checks = {}
for n in ['typed-api-prototype-v746-inverse-trig-clippy','typed-api-prototype-v747-inverse-trig-scalar-tests','typed-api-prototype-v748-inverse-trig-replay-build','typed-api-prototype-v749-inverse-trig-repair-cap']:
    r = read(E/n/'result.json')
    assert r['exit_code'] == 0 and r['source_unchanged'] and r['main_unchanged']
    assert sha(E/n/'source-hashes.json.gz') == expected
    checks[n] = r
assert '987 passed; 0 failed;' in (E/'typed-api-prototype-v747-inverse-trig-scalar-tests/stdout.log').read_text()
assert '9 passed; 0 failed;' in (E/'typed-api-prototype-v749-inverse-trig-repair-cap/stdout.log').read_text()
failures = {}
for n in range(735,743):
    d = E/f'typed-api-prototype-v{n}-inverse-trig-regression-cap'
    r = read(d/'result.json')
    log = (d/'stdout.log').read_text()
    cap = json.loads(log[log.rfind('{"command":'):])
    assert r['exit_code'] == (0 if n > 740 else 152 if n == 738 else 101)
    failures[d.name] = {'result':r,'cap':cap}
costs = {}
for a,b in [('typed-api-prototype-v732-inverse-trig-cost-baseline','typed-api-prototype-v751-inverse-trig-cost'),('typed-api-prototype-v695-normal-form-cost','typed-api-prototype-v752-inverse-trig-normal-form-cost'),('typed-api-prototype-v696-normal-form-default-cost','typed-api-prototype-v753-inverse-trig-default-cost'),('typed-api-prototype-v697-normal-form-algebraic-cost','typed-api-prototype-v754-inverse-trig-algebraic-cost')]:
    before,after = read(E/a/'summary.json'),read(E/b/'summary.json')
    assert after['source_hashes_sha256'] == expected
    for f in ['probe.rs','reproduce.py']: assert sha(E/a/f) == sha(E/b/f)
    costs[b] = {'baseline':a,'before_sha256':sha(E/a/'summary.json'),'after_sha256':sha(E/b/'summary.json'),
        'median_percent_change':{k:round(100*(after['cases'][k]['median_ns']/v['median_ns']-1),2) for k,v in before['cases'].items()}}
replays = {}
for n,suffix,count in [(750,'inverse-trig-replay',384),(755,'inverse-trig-evaluation-replay',384),(756,'inverse-trig-algebraic-replay',288),(757,'inverse-trig-wide-replay',144),(758,'inverse-trig-normal-form-replay',384)]:
    d = E/f'typed-api-prototype-v{n}-{suffix}'
    r = read(d/'summary.json')
    assert r['source_hashes_sha256'] == expected and r['guards_unchanged'] and r['cases'] == count and r['incomplete_cases'] == 0
    replays[d.name] = {'cases':count,'summary_sha256':sha(d/'summary.json')}
paired = read(E/'inverse-trig-paired-cost-v759/summary.json')
assert paired['source_record_sha256'] == expected and paired['processes'] == 432 and paired['guards_unchanged']
data = {
    'status':'Isolated inverse-trig accounting repair passes focused correctness; ordinary-cost overhead and full checkpoint remain pending; no runtime admission',
    'previous_checkpoint':'typed-api-prototype-v729-state.json','baseline_commit':'11b2d466b','source_record':str(record.relative_to(E)),
    'source_record_sha256':expected,'changed_sources':changed,'contract_decision':'p4-contract-decision-v173.json',
    'changes':[
        'Meter inverse-trig linear and multiple-argument recognition visits, coefficient operations, rational argument copies, and fallible exact argument comparisons.',
        'Meter retained pi multiples, Laurent coefficient projection, canonical sine terms and supported power-of-two coefficient construction. Reject unsupported shifts before allocation debit.',
        'Propagate one private accounting specialization through synchronous temporary forms; ordinary requests keep the original recognition rules without dynamic inner-loop charges.',
        'Keep independent finite-domain checks and prevent sign publication after nested budget exhaustion. Completed certificates remain reusable.'
    ],
    'retained_initial_regressions':failures,
    'source_corrections':{'v733':'Test coefficient needs an explicit BigInt type; v734 compiles with identical fixtures/assertions.','v745':'Strict Clippy requested the question-mark form for the right-hand rational-leaf fallback; v746 passes.'},
    'orchestration_corrections':['v730 baseline invocation used the singular stack-test phase; corrected to stack-tests in v731 before any workloads.','The initial metadata collector expected a cap receipt on a separate line. SIGXCPU leaves the test name on that line, so parse the retained JSON object from its command key; no validation outputs changed. The following git add had no path list and staged nothing.'],
    'regression_coverage':{'tests':9,'parser_success_work':[38,44,26],'cached_domain_sign_success_work':168,
        'cutpoints':'All allowances from zero through the measured successful allowance plus two.',
        'coefficient_bits':[16,256,1024],'coefficient_costs':[44,152,860],'atom_comparison_depth':512,
        'shared_dag_depth':28,'shared_dag_allowance':128,'public_retry_schedules':28,'original_existing_tests_unchanged':True},
    'passing_validation':checks,'replays':replays,'cost_observations':costs,
    'paired_cost':'inverse-trig-paired-cost-v759/summary.json',
    'paired_percent_changes':{k:[round(r['median_percent_change'],2) for r in v] for k,v in paired['cases'].items()},
    'next_work':['Check whether fallible temporary-form helper calls inhibit ordinary-path inlining; preserve the active checks and every baseline fixture.','Repeat focused and complete checkpoint validation on the final source, including all original replays and conditional proof checks.'],
    'limits':['Nine unit tests and 384 new replay cases are finite coverage; cooperative work does not bound exact-operation time or allocation.','Main runtime remains unadmitted and all original 49 worktree entries remain untouched.','All 39 properties, 14 targets, crate future work and 13 whole-crate proof groups retain their prior statuses.']
}
path = E/'inverse-trig-initial-trials-v760.json'
assert not path.exists()
path.write_text(json.dumps(data,indent=2)+'\n')
expected_status = read(E.parent/'2026-10-10-artifact-numeric-keys/retention.json')['staged_verification']['preexisting_status']
status = subprocess.check_output(['git','status','--porcelain'],cwd=ROOT,text=True).splitlines()
assert [x for x in status if not x[3:].startswith(str(E.relative_to(ROOT)))] == expected_status
print(json.dumps({'changed_sources':len(changed),'replay_cases':sum(x['cases'] for x in replays.values()),'paired_changes':data['paired_percent_changes']}))
