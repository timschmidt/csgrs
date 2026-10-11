#!/usr/bin/env python3
"""Bind the exact residual rewrite to frozen allocation and numerical evidence."""
import difflib
import gzip
import json
from pathlib import Path
import re
import sys

sys.dont_write_bytecode = True
import record_algebraic_budget_implementation as evidence
from parse_test_results_v3 import parse_phase

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT/'target/scalar-evaluation-candidate/source'
EXPECTED = '94999b4ae4cf57ea63431cc2c2671c96335fa23049870e1abaf222a0c226d080'
read, sha, write = evidence.read, evidence.sha, evidence.write

def allocations(phases):
    rows = {}
    for phase in phases:
        for line in (E/phase/'stderr.log').read_text().splitlines():
            if line.startswith('acos warm allocation '):
                fixture, detail = line.split(', allocations=')
                match = re.fullmatch(r'Allocations \{ calls: (\d+), bytes: (\d+), largest: (\d+) \}', detail)
                assert match and fixture not in rows
                rows[fixture] = dict(zip(['calls','bytes','largest'],map(int,match.groups())))
    return rows

def main():
    baseline = read(E/'acos-warm-baseline-v1422.json')
    initial_record = E/baseline['source_record']
    initial = json.loads(gzip.decompress(initial_record.read_bytes()))
    record = E/'typed-api-prototype-v1436-acos-residual-build/source-hashes.json.gz'
    assert sha(record) == EXPECTED
    source = json.loads(gzip.decompress(record.read_bytes()))
    kernel = 'crates/hyperreal/src/computable/approximation/inverse_trig.rs'
    assert {n for n,v in source.items() if initial.get(n) != v} == {kernel}
    assert all(sha(SOURCE/n) == v for n,v in source.items())
    old = initial_record.parent/'inputs'/kernel
    assert sha(old) == initial[kernel]
    patch = ''.join(difflib.unified_diff(old.read_text().splitlines(True), (SOURCE/kernel).read_text().splitlines(True), fromfile='before/'+kernel,tofile='after/'+kernel))
    patch_path = E/'acos-residual-runtime-diff-v1448.patch'
    assert not patch_path.exists()
    patch_path.write_text(patch)
    prior = read(E/'typed-api-prototype-v1404-state.json')
    previous_source = json.loads(gzip.decompress((E/prior['source_record']).read_bytes()))
    assert all(source[n] == h for n,h in previous_source.items() if 'test' in n)

    passes = {}
    names = ['typed-api-prototype-v1423-acos-residual-build','typed-api-prototype-v1426-acos-residual-unit-tests',
             'typed-api-prototype-v1435-acos-residual-clippy','typed-api-prototype-v1436-acos-residual-build']
    caps = read(E/'acos-warm-allocation-caps-v1425.json')['tests'] | read(E/'acos-warm-survey-caps-v1434.json')['tests']
    for phase in names + list(caps):
        result = read(E/phase/'result.json')
        assert result['exit_code'] == 0 and result['source_unchanged'] and result['main_unchanged']
        assert sha(E/phase/'source-hashes.json.gz') == EXPECTED
        passes[phase] = result
    for row in caps.values():
        cap = row['cap']
        assert cap['exit_code'] == 0 and not cap['wall_timeout']
        assert cap['address_space_bytes'] == 192*1024*1024
        assert cap['cpu_seconds_soft_hard'] == [2,3] and cap['wall_seconds_limit'] == 4
    units = parse_phase(E/'typed-api-prototype-v1426-acos-residual-unit-tests')
    assert units['top_level_passed'] == 1064 and units['child_process_passed'] == 1
    assert units['top_level_failed'] == units['child_process_failed'] == 0
    assert not any(l.startswith('warning:') for l in (E/'typed-api-prototype-v1435-acos-residual-clippy/stderr.log').read_text().splitlines())
    old_alloc = allocations(read(E/'acos-warm-allocation-caps-v1421.json')['tests'])
    new_alloc = allocations(read(E/'acos-warm-allocation-caps-v1425.json')['tests'])
    assert len(old_alloc) == len(new_alloc) == 8 and old_alloc.keys() == new_alloc.keys()
    measured = {k:{'baseline':old_alloc[k],'candidate':new_alloc[k]} for k in old_alloc}
    assert all(v['candidate']['bytes'] < v['baseline']['bytes'] for v in measured.values())

    # The complete baseline and new capped runs contain identical schedules.
    # Compare every observable field including work, cache state and existence;
    # only the explicitly measured allocator counts may change.
    pattern = re.compile(r'acos warm survey bits=(\d+), negative=(true|false), state=(\w+), units=(\d+), answer=(Ok\(-?\d+\)|Err\(Exhausted\)), consumed=(\d+), remaining=(\d+), finite=(true|false), before=(.*?), after=(.*?), allocations=Allocations \{ calls: (\d+), bytes: (\d+), largest: (\d+) \}')
    schedules = []
    for phase in read(E/'acos-warm-survey-caps-v1434.json')['tests']:
        for m in pattern.finditer((E/phase/'stderr.log').read_text()):
            bits,neg,state,u,answer,consumed,remaining,finite,before,after,calls,allocated,largest=m.groups()
            schedules.append({'bits':int(bits),'negative':neg=='true','state':state,'units':int(u),
                'answer':answer,'consumed':int(consumed),'remaining':int(remaining),'finite':finite=='true',
                'before':before,'after':after,'allocation_calls':int(calls),'allocation_bytes':int(allocated),'largest_request':int(largest)})
    key=lambda r:(r['bits'],r['negative'],r['state'],r['units'])
    strip=lambda r:{k:v for k,v in r.items() if k not in {'allocation_calls','allocation_bytes','largest_request'}}
    old_rows=sorted(baseline['schedules'],key=key);new_rows=sorted(schedules,key=key)
    assert len(new_rows)==160 and [strip(r) for r in old_rows]==[strip(r) for r in new_rows]

    replays = {}
    for a,b,suffix,count in [(1362,1442,'inverse',384),(1363,1443,'evaluation',384),
            (1364,1444,'algebraic',288),(1365,1445,'wide',144),(1366,1446,'normal-form',384)]:
        old_phase=f'typed-api-prototype-v{a}-integer-scale-{suffix}-replay'
        new_phase=f'typed-api-prototype-v{b}-acos-residual-{suffix}-replay'
        data=evidence.check_artifacts(new_phase)
        assert data['cases']==count and data['incomplete_cases']==0 and data['source_hashes_sha256']==EXPECTED
        assert all(data.get(k,0)==0 for k in ['invalid_samples','invalid_numeric_samples','typed_errors'])
        assert all(sha(E/old_phase/n)==sha(E/new_phase/n) for n in ['probe.rs','reproduce.py'])
        strip_rows=lambda rows:[{k:v for k,v in r.items() if k!='command'} for r in rows]
        assert strip_rows(read(E/old_phase/'receipts.json'))==strip_rows(read(E/new_phase/'receipts.json'))
        replays[new_phase]={'cases':count,'receipts_identical_except_commands':True,'summary_sha256':sha(E/new_phase/'summary.json')}
    paired=evidence.check_artifacts('acos-residual-paired-cost-v1448')
    assert paired['source_record_sha256']==EXPECTED and paired['processes']==720 and paired['guards_unchanged']
    evidence.EXPECTED=EXPECTED
    costs, records = {}, []
    for family,a,b,suffix,checksum in [
        ('inverse-trig',1358,1438,'inverse-cost','certified'),('normal-form',1359,1439,'normal-form-cost','certified'),
        ('default',1360,1440,'default-cost','checksums'),('algebraic',1361,1441,'algebraic-cost','certified')]:
        name=f'acos-residual-{family}-cost-comparison-v1448.json'
        costs[family]=evidence.cost(f'typed-api-prototype-v{a}-integer-scale-{suffix}',f'typed-api-prototype-v{b}-acos-residual-{suffix}',name,checksum)
        records.append(name)
    for family,a,b in [('acos','typed-api-prototype-v1418-acos-warm-baseline-cost','typed-api-prototype-v1437-acos-residual-targeted-cost'),
                       ('integer','typed-api-prototype-v1357-integer-scale-cost','typed-api-prototype-v1447-acos-residual-integer-cost')]:
        old_cost,new_cost=evidence.check_artifacts(a),evidence.check_artifacts(b)
        assert old_cost['processes']==new_cost['processes']==72
        assert all(sha(E/a/n)==sha(E/b/n) for n in ['probe.rs','reproduce.py'])
        cases={}
        for case,old_case in old_cost['cases'].items():
            new_case=new_cost['cases'][case]
            for field in ['iterations','checksums']+(['reference','output_bytes'] if family=='acos' else []):
                assert old_case[field]==new_case[field]
            assert len(old_case['elapsed_ns'])==len(new_case['elapsed_ns'])==9
            cases[case]={'before':old_case,'after':new_case,'median_ratio':new_case['median_ns']/old_case['median_ns']}
        name=f'acos-residual-{family}-cost-comparison-v1448.json'
        write(name,{'baseline_phase':a,'candidate_phase':b,'cases':cases,'source_record_sha256':EXPECTED,'probe_and_harness_identical':True})
        records.append(name)
        costs[family]={k:round(100*(v['median_ratio']-1),2) for k,v in cases.items()}
    write('acos-residual-implementation-v1448.json', {
        'status':'Unretained first residual candidate: correctness passes, ordinary/cached cost regressions require investigation; no runtime admission',
        'previous_checkpoint':'typed-api-prototype-v1404-state.json','baseline_commit':'782329f0c',
        'baseline_record':'acos-warm-baseline-v1422.json','source_record':str(record.relative_to(E)),
        'source_record_sha256':EXPECTED,'runtime_diff':patch_path.name,'runtime_file':kernel,
        'all_prior_tests_and_ten_frozen_new_tests_unchanged':True,
        'change':'Form the identical signed half-angle residual from borrowed canonical magnitudes d-n and d+n, then reduce one Rational. Negative raw operands swap these positive magnitudes. Existing domain/endpoint guards, kernel selection, work charges and cache publication remain intact.',
        'allocation_observations':measured,'schedules':new_rows,'all_160_schedule_receipts_identical_except_allocations':True,
        'caps':caps,'passing_validation':passes,'unit_summary':units,'replays':replays,
        'cost_records':records,'original_trial_percent_changes':costs,
        'paired_cost':'acos-residual-paired-cost-v1448/summary.json',
        'paired_percent_changes':{k:[round(r['median_percent_change'],2) for r in v] for k,v in paired['cases'].items()},
        'executable_sizes':{k:v['executable_bytes'] for k,v in paired['provenance'].items()},
        'limits':['Paired observations include +14.37 percent cached-positive, +31.65 cached-negative, +12.22 integer-wide-fine, +16.91 atan-small and +15.10 terms-wide medians. These are retained adverse observations; this candidate is not yet retained.',
            'This reduces avoidable temporary allocation; it introduces no byte/time budget and leaves output-sized exact operations possible at low cooperative allowances.',
            'The negative one-unit query still returns Exhausted and retains its completed coarse cache; the optimization does not change completeness to obtain the allocation gain.',
            'Rust allocation requests exclude fixture creation, native allocations, RSS and peak-live memory. Timing is local and all trials remain preserved.',
            'The conditional Verus proof target does not cover this inverse-trig numerical kernel. All 44 listed proof/guard inputs are unchanged; whole-crate proof remains open.',
            'Main runtime and all 49 original worktree entries remain unchanged; all 39 properties, 14 targets and crate future work remain in scope.']
    })
    print(json.dumps({'record':'acos-residual-implementation-v1448.json','allocation_cases':8,'schedules':160,
        'frozen_replays':1584,'paired_percent_changes':{k:[round(r['median_percent_change'],2) for r in v] for k,v in paired['cases'].items()}}))

if __name__=='__main__':
    main()
