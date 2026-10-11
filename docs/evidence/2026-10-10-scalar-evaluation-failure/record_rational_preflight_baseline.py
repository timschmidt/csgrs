#!/usr/bin/env python3
"""Freeze lazy rational/domain preflight failures before runtime repair."""
import gzip
import hashlib
import json
from pathlib import Path
import subprocess

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT/'target/scalar-evaluation-candidate/source'
read = lambda p:json.loads(p.read_text())
sha = lambda p:hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    out = E/'rational-preflight-baseline-v1146.json'
    assert not out.exists()
    previous = read(E/'typed-api-prototype-v1133-state.json')
    for name,value in previous['artifacts_sha256'].items(): assert sha(E/name) == value,name
    old_record = E/previous['source_record']
    before = json.loads(gzip.decompress(old_record.read_bytes()))
    record = E/'typed-api-prototype-v1136-rational-preflight-baseline-build/source-hashes.json.gz'
    source = json.loads(gzip.decompress(record.read_bytes()))
    assert sha(record) == 'c8affb5b0ec2497ed24b03f248d5e9baf43c7b4363753afc737c0638d80e00db'
    for name,value in source.items(): assert sha(SOURCE/name) == value,name
    changed = {n:{'before':before.get(n),'after':v} for n,v in source.items() if before.get(n) != v}
    prefix = 'crates/hyperreal/src/computable/'
    node,test = prefix+'node.rs',prefix+'node/rational_preflight_tests.rs'
    assert set(changed) == {node,test}
    assert (SOURCE/node).read_text().replace('include!("node/rational_preflight_tests.rs");\n','') == (old_record.parent/'inputs'/node).read_text()
    assert all(source[n] == v for n,v in before.items() if 'test' in n)
    guard = json.loads(gzip.decompress((E/'candidate-preparation/main-hashes.json.gz').read_bytes()))
    for name,value in guard.items(): assert sha(ROOT/name) == value,name
    expected = read(E.parent/'2026-10-10-artifact-numeric-keys/retention.json')['staged_verification']['preexisting_status']
    status = subprocess.check_output(['git','status','--porcelain'],cwd=ROOT,text=True).splitlines()
    assert [s for s in status if not s[3:].startswith(str(E.relative_to(ROOT)))] == expected
    caps = read(E/'rational-preflight-baseline-caps-v1146.json')
    assert len(caps['tests']) == 10 and caps['source_record_sha256'] == sha(record)
    observations,codes = {},[]
    for phase,row in caps['tests'].items():
        result = read(E/phase/'result.json')
        assert result == row['result'] and result['source_unchanged'] and result['main_unchanged']
        assert sha(E/phase/'source-hashes.json.gz') == sha(record)
        cap = row['cap']
        assert cap['address_space_bytes'] == 192*1024*1024
        assert cap['cpu_seconds_soft_hard'] == [2,3] and cap['wall_seconds_limit'] == 4
        assert cap['helper_sha256'] == sha(E/'run_limited_unit_test.py')
        codes.append(result['exit_code'])
        observations[row['test']] = [s for s in (E/phase/'stderr.log').read_text().splitlines()
            if s.startswith(('rational ','child domain ','literal domain ','public domain '))]
    assert codes == [101]*5+[0]*5
    assert [len(v) for v in observations.values()] == [8,16,32,24,56,0,0,9,0,0]
    preparation = []
    for version in [1134,1135,1136]:
        phase = E/f'typed-api-prototype-v{version}-rational-preflight-baseline-build'
        data = read(phase/'result.json')
        assert data['source_unchanged'] and data['main_unchanged']
        assert data['exit_code'] == (101 if version == 1134 else 0)
        preparation.append({'phase':phase.name,'result':data,'source_record_sha256':sha(phase/'source-hashes.json.gz')})
    initial = E/'typed-api-prototype-v1134-rational-preflight-baseline-build/inputs'/test
    typed = E/'typed-api-prototype-v1135-rational-preflight-baseline-build/inputs'/test
    assert initial.read_text().replace('i32::from(n)','i64::from(n)').replace('                d,\n','                u64::from(d),\n') == typed.read_text()
    assert typed.read_text().replace('let expected = if kind <= 1 {','let expected = if kind == 0 {') == (SOURCE/test).read_text()
    costs = {}
    for family,p in read(E/'algebraic-preflight-paired-cost-v1100/summary.json')['provenance'].items():
        binary = Path(p['binaries']['candidate'])
        assert sha(binary) == p['binary_sha256_at_paired_run']['candidate']
        assert binary.stat().st_size == p['executable_bytes']['candidate']
        assert sha(E/p['phases']['candidate']/'summary.json') == p['original_summary_sha256']['candidate']
        costs[family] = {'phase':p['phases']['candidate'],'binary':str(binary),'sha256':sha(binary),'bytes':binary.stat().st_size}
    out.write_text(json.dumps({
        'status':'Frozen lazy rational/domain preflight regressions before runtime repair; no runtime admission',
        'previous_checkpoint':'typed-api-prototype-v1133-state.json',
        'previous_verification':{'artifacts':len(previous['artifacts_sha256']),'candidate_sources':len(before),
            'main_guarded':len(guard),'original_worktree_entries':len(expected)},
        'source_record':str(record.relative_to(E)),'source_record_sha256':sha(record),
        'test_only_changes':changed,'prior_named_test_bodies_and_assertions_unchanged':True,
        'fixture_preparation':{'builds':preparation,
            'compile_correction':'Rational::fraction requires i64/u64; make the fixture conversions explicit without changing values.',
            'oracle_correction_before_test_execution':'The existing domain validator reports SqrtNegative for a negative square root and NotANumber for a negative NthRoot. Corrected that new test expectation before any regression test was executed.',
            'first_collector_attempt':'validate_rational_preflight_baseline.py stopped at its failed-build prerequisite. It executed no test and created no cap phase.',
            'final_collector':'validate_rational_preflight_baseline_v2.py'},
        'baseline_caps':'rational-preflight-baseline-caps-v1146.json',
        'outcomes':{'assertion_failures':5,'passing_guards':5},'observations':observations,
        'findings':[
            'bounded_rational_value canonicalizes lazy Ratio components in its size guard before meter.read. Each million-bit leaf requests 2500392 Rust allocation bytes before declining at one unit.',
            'Eight child-domain variants reach that recognizer under three units. Their million-bit fixtures request 2500392 to 2500656 bytes before declining; the sign-based fallback cases add bounded allocations after the unsuccessful rational attempt.',
            'Six direct rational-domain variants canonicalize before coefficient work. Their million-bit fixtures request 2500392 bytes and return a successful domain certificate after consuming only the one node unit.',
            'All fourteen public approximation families return Exhausted at their selected low allowance, but million-bit lazy inputs request 2500432 to 2500712 bytes versus 40 to 320 bytes for canonical small-value controls.',
            'Generous exact values at the 4096/4097-bit boundaries, cheap canonical rejection and positive-log domains, every measured recognition cutpoint, 42 public retry/proof schedules and typed domain failures already pass.'
        ],
        'allocation_scope':'Opt-in thread-local Rust allocation requests during the named synchronous call. Fixture setup, assertions, printing, native allocation and other threads are excluded. Private measurements enter BudgetScope before measurement; public measurements include the scope and wrapper work, compared against a fresh canonical control of the same value and shape. Not RSS or peak-live memory.',
        'coverage':{
            'test_functions':10,'stored_factor_bits':[65,4097,65537,1000001],
            'leaf_signs':['positive','negative'],'lazy_boundary_low_work_cases':16,
            'canonical_component_bit_boundaries':[4096,4097],
            'child_domains':['Sqrt','NthRoot(3)','Inverse','LnDeferred','AsinDeferred','AcosPositive','AtanhDirect','AcoshDirect'],
            'literal_domains':['AsinRational','AcosPositiveRational','AcosNegativeRational','AtanhRational','PrescaledLnRational','BinaryScaledLnRational(shift=0)'],
            'low_work_units':{'leaf':1,'child_domain':3,'literal_domain':1},
            'public_allocation_cases':56,'canonical_public_controls':14,
            'recognition_cutpoint_schedules':9,'recognition_factor_bits':[65,256,4097],
            'recognition_shapes':['Ratio','eight Offset(0) wrappers','Inverse'],
            'cutpoints':'Every allowance through measured successful work plus two, preserving node facts and a successful unlimited retry.',
            'public_retry_schedules':42,'public_retry_factor_bits':[65,4097,65537],
            'candidate_calls_per_public_retry_schedule':5,'canonical_reference_calls_per_schedule':1,
            'public_generous_retry_allowance':1000000000,'retained_zero_work_checks_per_schedule':2,
            'invalid_domain_cases':39,'exact_root_identity_cases':6},
        'retained_cost_baselines':costs,
        'limits':[
            'No production runtime source changed in this baseline.',
            'Measured public allocation excess is specific to the frozen shapes, values and allowances; no whole-program bound is established.',
            'Direct Real adapters, constructor arithmetic, other coefficient operations and broader concurrent/reconstructed warming remain separate work.',
            'All 49 original worktree entries, 39 properties, 14 targets and crate future work retain their previous scope and status.'
        ]
    },indent=2)+'\n')
    print(json.dumps({'record':out.name,'source_sha256':sha(record),'artifacts_verified':len(previous['artifacts_sha256']),
        'failing_regressions':5,'passing_guards':5,'public_allocation_cases':56,'public_retry_schedules':42}))


if __name__ == '__main__':
    main()
