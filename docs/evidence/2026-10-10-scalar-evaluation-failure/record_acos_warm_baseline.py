#!/usr/bin/env python3
"""Freeze measured reachability, failed allocation guards and timing inputs."""
import difflib
import gzip
import hashlib
import json
from pathlib import Path
import re

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT/'target/scalar-evaluation-candidate/source'
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
load_source = lambda phase: json.loads(gzip.decompress((E/phase/'source-hashes.json.gz').read_bytes()))
out = E/'acos-warm-baseline-v1422.json'
assert not out.exists()
previous = read(E/'typed-api-prototype-v1404-state.json')
assert sha(E/'typed-api-prototype-v1404-state.json') == 'fad2d16a1483824f4d6587e26a22049bfac5b715e4a2ecf9f8a74661b6f34e77'
before = json.loads(gzip.decompress((E/previous['source_record']).read_bytes()))
phase = 'typed-api-prototype-v1419-acos-warm-regression-build'
after = load_source(phase)
changed = [name for name, digest in after.items() if before.get(name) != digest]
test = 'crates/hyperreal/src/computable/node/acos_warm_probe_tests.rs'
assert sorted(changed) == sorted(['crates/hyperreal/src/computable/node.rs', test])
for name, digest in after.items():
    assert sha(SOURCE/name) == digest, name
facade = 'crates/hyperreal/src/computable/node.rs'
assert (SOURCE/facade).read_text().replace('include!("node/acos_warm_probe_tests.rs");\n', '') == (E/Path(previous['source_record']).parent/'inputs'/facade).read_text()

pattern = re.compile(r'acos warm survey bits=(\d+), negative=(true|false), state=(\w+), units=(\d+), answer=(Ok\(-?\d+\)|Err\(Exhausted\)), consumed=(\d+), remaining=(\d+), finite=(true|false), before=(.*?), after=(.*?), allocations=Allocations \{ calls: (\d+), bytes: (\d+), largest: (\d+) \}')
def parse(path):
    rows = []
    for match in pattern.finditer(path.read_text()):
        bits, negative, state, units, answer, consumed, remaining, finite, before, after, calls, allocated, largest = match.groups()
        rows.append({'bits':int(bits), 'negative':negative=='true', 'state':state,
            'units':int(units), 'answer':answer, 'consumed':int(consumed), 'remaining':int(remaining),
            'finite':finite=='true', 'before':before, 'after':after,
            'allocation_calls':int(calls), 'allocation_bytes':int(allocated), 'largest_request':int(largest)})
    return rows

full = 'typed-api-prototype-v1415-acos-warm-survey-full'
assert read(E/full/'result.json')['exit_code'] == 0
assert '8 passed; 0 failed;' in (E/full/'stdout.log').read_text()
rows = parse(E/full/'stderr.log')
key = lambda row: (row['bits'], row['negative'], row['state'], row['units'])
by_key = {key(row):row for row in rows}
assert len(rows) == len(by_key) == 160
assert set(by_key) == {(b,n,s,u) for b in [65,4097,65537,1000001] for n in [False,True]
    for s in ['cold','domain','coarse','complete'] for u in [0,1,2,4,8]}
survey_caps = read(E/'acos-warm-survey-caps-v1414.json')
assert [r['result']['exit_code'] for r in survey_caps['tests'].values()] == [0,0,0,0,0,0,152,152]
differences = []
for p, cap in survey_caps['tests'].items():
    samples = parse(E/p/'stderr.log')
    for row in samples:
        if row != by_key[key(row)]:
            differences.append({'phase':p,'capped':row,'full':by_key[key(row)]})
    assert len(samples) == (20 if cap['result']['exit_code'] == 0 else 6)

allocation_caps = read(E/'acos-warm-allocation-caps-v1421.json')
assert [r['result']['exit_code'] for r in allocation_caps['tests'].values()] == [101,101]
for p, cap in allocation_caps['tests'].items():
    assert cap['cap']['exit_code'] == 101 and not cap['cap']['wall_timeout']
    log = (E/p/'stderr.log').read_text()
    assert log.count('acos warm allocation bits=') == 4
    assert 'coarse-warmed acos copied avoidable rational intermediates' in log

cost = read(E/'typed-api-prototype-v1418-acos-warm-baseline-cost/summary.json')
assert cost['processes'] == 72 and cost['guards_unchanged']
assert all(len(row[k]) == 1 for row in cost['cases'].values() for k in ['checksums','output_bytes','reference'])
probe = ROOT/'target/typed-api-prototype-v1418-acos-warm-baseline-cost/probe'
assert sha(probe) == cost['binary_sha256']
earlier = E/'typed-api-prototype-v1405-acos-warm-survey-build/inputs'/test
corrected = E/'typed-api-prototype-v1406-acos-warm-corrected-survey-build/inputs'/test
patch = E/'acos-warm-survey-test-correction-v1422.patch'
assert not patch.exists()
patch.write_text(''.join(difflib.unified_diff(earlier.read_text().splitlines(True), corrected.read_text().splitlines(True), fromfile='v1405/'+test, tofile='v1406/'+test)))
data = {
    'status':'Measured isolated baseline; no production change or runtime admission',
    'previous_checkpoint':'typed-api-prototype-v1404-state.json',
    'previous_checkpoint_sha256':sha(E/'typed-api-prototype-v1404-state.json'),
    'source_record':phase+'/source-hashes.json.gz', 'source_record_sha256':sha(E/phase/'source-hashes.json.gz'),
    'source_changes':changed, 'production_behavior_unchanged':True,
    'full_survey_phase':full, 'schedules':rows, 'capped_full_survey_record':'acos-warm-survey-caps-v1414.json',
    'cap_vs_full_receipt_differences':differences,
    'aggregate_cap_failures':'The two million-bit twenty-schedule processes each reached SIGXCPU after six logged schedules. These logs cover only partial surveys and do not establish an individual query timeout. The original full-size processes and complete uncapped Cargo survey are both retained.',
    'allocation_regression_record':'acos-warm-allocation-caps-v1421.json',
    'allocation_guard':'Both signed tests collect all four widths before asserting at most eight coefficient-sized buffers plus 8192 fixed Rust allocation bytes. Both fail on existing code under the unchanged 192 MiB / CPU 2,3 / wall 4 caps. This ceiling is a targeted regression criterion, not an EvaluationBudget contract.',
    'test_correction':{'patch':patch.name, 'sha256':sha(patch),
        'reason':'Before execution, correct the new coarse-read expectation: a finer completed cache may rescale to a different valid integer. Failed queries retain the original coarse entry; successful finer queries must preserve the same value/error contract. This implements the accepted P4 rule. v1405 was build-only; v1406 contains the corrected eight survey tests.'},
    'cost_record':'typed-api-prototype-v1418-acos-warm-baseline-cost/summary.json',
    'cost_binary':str(probe.relative_to(ROOT)), 'cost_binary_sha256':sha(probe), 'cost_binary_bytes':probe.stat().st_size,
    'cost_compile_correction':'v1417 retains the initial probe compile failure. v1418 uses a separate immutable probe/harness with an explicit (usize, i32, bool, bool) tuple annotation; numerical inputs, loops and operations are unchanged.',
    'findings':[
        'Public rational acos retains the expected positive/negative rational variant and exact coefficient widths. Public approx(1) warms the finite-real fact and a coarse numeric cache. Public approx_with_budget(0) then misses that cache and reaches width-sensitive rational arithmetic.',
        'The million-bit coarse-warmed positive query returns Ok(0) using one cooperative unit and requests 11425512 Rust bytes. The negative query requests 11425952 bytes and returns Exhausted without replacing its completed coarse value.',
        'Completed million-bit root cache hits allocate 40 positive / 48 negative bytes and consume no work. Cold low-allowance queries fail before coefficient allocation. Domain-only warming is a separately identified private diagnostic.',
        'All 160 full schedules preserve exact stored coefficients, allowance accounting, successful numerical results, exhausted-query cache retention, unlimited retry and zero-work reuse of completed numeric/sign certificates.'
    ],
    'next_candidate':'Prototype the identical half-angle rational (d-n)/(d+n), swapping numerator/denominator for a negative raw operand, after existing domain and endpoint checks. Compare exact results/work receipts and all frozen workloads. Do not insert a new width debit merely to disguise the copies or weaken available certificates.',
    'limits':['EvaluationBudget excludes per-operation bytes/time; measured copies are not a demonstrated contract violation.',
        'Rust allocation counters exclude fixtures, oracle/reference work, native allocations, RSS and peak-live memory.',
        'The 72 timing trials are local public-query workloads with prepared caches, not a full-board benchmark or proof.',
        'The full properties plan, all original hard fixtures, main runtime non-admission and original 49 worktree entries remain unchanged.']
}
out.write_text(json.dumps(data,indent=2)+'\n')
print(json.dumps({'record':out.name,'schedules':len(rows),'receipt_differences':len(differences),'allocation_regressions':2,'cost_processes':72}))
