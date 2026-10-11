#!/usr/bin/env python3
"""Bind the structural payload repair to unchanged regression and cost inputs."""
import difflib
import gzip
import json
from pathlib import Path
import sys

sys.dont_write_bytecode = True
import record_algebraic_budget_implementation as evidence

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT/'target/scalar-evaluation-candidate/source'
EXPECTED = 'a0d1d543babfb7ae724d0cfdebb6a294c33b59423b94223a0468716413a1b00b'
read, sha, write = evidence.read, evidence.sha, evidence.write


def main():
    prior = read(E/'typed-api-prototype-v1011-state.json')
    baseline = read(E/'structural-comparison-baseline-v1024.json')
    record = E/'typed-api-prototype-v1032-structural-comparison-build/source-hashes.json.gz'
    assert sha(record) == EXPECTED
    source = json.loads(gzip.decompress(record.read_bytes()))
    before = json.loads(gzip.decompress((E/prior['source_record']).read_bytes()))
    assert all(sha(SOURCE/n) == v for n,v in source.items())
    changed = {n: {'before': before.get(n), 'after': v} for n,v in source.items() if before.get(n) != v}
    prefix = 'crates/hyperreal/src/'
    runtimes = [prefix+'computable/node/'+n for n in ['primitive_constructors.rs', 'symbolic_work.rs']]
    runtimes.append(prefix+'rational/arithmetic/queries_conversion.rs')
    test = prefix+'computable/node/structural_comparison_budget_tests.rs'
    assert set(changed) == set(runtimes+[test, prefix+'computable/node.rs'])
    assert all(source[n] == v for n,v in before.items() if 'test' in n)
    initial_record = E/baseline['source_record']
    initial = json.loads(gzip.decompress(initial_record.read_bytes()))
    assert set(n for n,v in source.items() if initial.get(n) != v) == set(runtimes+[test])
    patch = E/'structural-comparison-runtime-diff-v1042.patch'
    assert not patch.exists()
    parts = []
    for name in runtimes:
        old = initial_record.parent/'inputs'/name
        if not old.exists(): old = ROOT/name
        assert sha(old) == initial[name]
        parts.extend(difflib.unified_diff(old.read_text().splitlines(True),
            (SOURCE/name).read_text().splitlines(True), fromfile='before/'+name, tofile='after/'+name))
    patch.write_text(''.join(parts))
    current_test = (SOURCE/test).read_text()
    start = current_test.index('    #[test]\n    fn wide_structural_quantile_precision_rejects_before_payload')
    end = current_test.index('    #[test]\n    fn wide_structural_shared_dags_preserve_equality', start)
    assert current_test[:start]+current_test[end:] == (initial_record.parent/'inputs'/test).read_text()
    passes = {}
    for version,suffix in [(1028,'guard-build'),(1029,'repair-cap'),(1030,'unit-tests'),(1031,'clippy'),(1032,'build')]:
        phase = f'typed-api-prototype-v{version}-structural-comparison-{suffix}'
        r = read(E/phase/'result.json')
        assert r['exit_code'] == 0 and r['source_unchanged'] and r['main_unchanged']
        assert sha(E/phase/'source-hashes.json.gz') == EXPECTED
        passes[phase] = r
    assert '1017 passed; 0 failed;' in (E/'typed-api-prototype-v1030-structural-comparison-unit-tests/stdout.log').read_text()
    assert not any(s.startswith('warning:') for s in (E/'typed-api-prototype-v1031-structural-comparison-clippy/stderr.log').read_text().splitlines())
    cap_phase = E/'typed-api-prototype-v1029-structural-comparison-repair-cap'
    log = (cap_phase/'stdout.log').read_text()
    cap = json.loads(log[log.rfind('{"command":'):])
    assert cap['exit_code'] == 0 and cap['address_space_bytes'] == 192*1024*1024
    assert cap['cpu_seconds_soft_hard'] == [2,3] and cap['wall_seconds_limit'] == 4
    assert '14 passed; 0 failed;' in log
    observations = [s for s in (cap_phase/'stderr.log').read_text().splitlines()
        if s.startswith(('wide ', 'lazy ', 'cross product ', 'shift rejection ', 'comparison cutpoints ', 'public comparison '))]
    assert sum(s.startswith('wide integer ') for s in observations) == 32
    assert sum(s.startswith('wide rational ') for s in observations) == 136
    allocations = [s for s in observations if 'allocations=' in s]
    assert len(allocations) == 16 and all('calls: 0, bytes: 0, largest: 0' in s for s in allocations)
    replays = {}
    for a,b,count in [(971,1033,384),(976,1038,384),(977,1039,288),(978,1040,144),(979,1041,384)]:
        old = next(n for n in prior['final_source_replays'] if n.startswith(f'typed-api-prototype-v{a}-'))
        new = old.replace(f'v{a}-inverse-dag',f'v{b}-structural-comparison')
        data = evidence.check_artifacts(new)
        assert data['cases'] == count and data['incomplete_cases'] == 0 and data['source_hashes_sha256'] == EXPECTED
        for key in ['invalid_samples','invalid_numeric_samples','typed_errors']: assert data.get(key,0) == 0
        for name in ['probe.rs','reproduce.py']: assert sha(E/old/name) == sha(E/new/name)
        old_rows,new_rows = read(E/old/'receipts.json'),read(E/new/'receipts.json')
        assert [evidence.contract(r) for r in old_rows] == [evidence.contract(r) for r in new_rows]
        strip = lambda rows:[{k:v for k,v in r.items() if k != 'command'} for r in rows]
        old_values,new_values = strip(old_rows),strip(new_rows)
        work_changes = []
        if b == 1040:
            for index,(left,right) in enumerate(zip(old_values,new_values)):
                assert len(left['records']) == len(right['records']) == 1
                a_record,b_record = left['records'][0],right['records'][0]
                assert a_record['consumed']+a_record['remaining'] == a_record['allowance']
                assert b_record['consumed']+b_record['remaining'] == b_record['allowance']
                if a_record != b_record:
                    work_changes.append({'case':index,
                        'before':{k:a_record[k] for k in ['consumed','remaining']},
                        'after':{k:b_record[k] for k in ['consumed','remaining']}})
                without_work = lambda row:row | {'records':[
                    {k:v for k,v in row['records'][0].items() if k not in ['consumed','remaining']}]}
                assert without_work(left) == without_work(right)
            assert len(work_changes) == 16
        else:
            assert old_values == new_values
        replays[new] = {'cases':count,'summary_sha256':sha(E/new/'summary.json'),
            'probe_harness_inputs_order_and_caps_preserved':True,
            'answers_numeric_observations_and_retained_proofs_identical':True,
            'receipts_identical_except_commands':old_values == new_values,
            'cooperative_work_count_changes':work_changes}
    evidence.EXPECTED = EXPECTED
    costs,records = {},[]
    for family,a,b,suffix,checksum in [
        ('inverse-trig',972,1034,'inverse-cost','certified'),
        ('normal-form',973,1035,'normal-form-cost','certified'),
        ('default',974,1036,'default-cost','checksums'),
        ('algebraic',975,1037,'algebraic-cost','certified'),
    ]:
        name = 'structural-comparison-'+family+'-cost-comparison-v1042.json'
        costs[family] = evidence.cost(f'typed-api-prototype-v{a}-inverse-dag-{suffix}',
            f'typed-api-prototype-v{b}-structural-comparison-{suffix}',name,checksum)
        records.append(name)
    paired = read(E/'structural-comparison-paired-cost-v1042/summary.json')
    assert paired['source_record_sha256'] == EXPECTED and paired['processes'] == 432 and paired['guards_unchanged']
    for name,value in paired['artifacts_sha256'].items(): assert sha(E/'structural-comparison-paired-cost-v1042'/name) == value
    confirmation = read(E/'structural-comparison-targeted-confirmation-v1042/summary.json')
    assert confirmation['source_record_sha256'] == EXPECTED and confirmation['processes'] == 216 and confirmation['guards_unchanged']
    for name,value in confirmation['artifacts_sha256'].items(): assert sha(E/'structural-comparison-targeted-confirmation-v1042'/name) == value
    failed = E/'typed-api-prototype-v1026-structural-comparison-repair-cap'
    assert read(failed/'result.json')['exit_code'] == 2
    assert "can't open file" in (failed/'stderr.log').read_text()
    write('structural-comparison-implementation-v1042.json', {
        'status':'Scoped wide structural-comparison and lazy rational preflight repair; full checkpoint pending; no runtime admission',
        'previous_checkpoint':'typed-api-prototype-v1011-state.json','baseline_commit':'a8e3c47c0',
        'baseline_record':'structural-comparison-baseline-v1024.json',
        'source_record':str(record.relative_to(E)),'source_record_sha256':EXPECTED,'changed_sources':changed,
        'runtime_diff':patch.name,'prior_named_test_bodies_and_assertions_unchanged':True,
        'all_twelve_baseline_tests_values_caps_and_assertions_unchanged':True,
        'changes':[
            'Debit payload-size work before budgeted structural integer or rational equality; propagate exhaustion as Unknown, preserving the unlimited comparison oracle.',
            'Keep identical node or rational storage and differing signs, integer widths, logarithm shifts and quantile precision as cheap decisions.',
            'Inspect raw stored rational widths through one crate-private accessor, avoiding lazy reduction inside symbolic-work preflight.',
            'Credit the existing node visit as the first payload unit; integer scans use 64-bit word counts and rational equality uses saturating operand-word products.',
            'Preserve the bounded shallow walk, shared-DAG memoization, exact arithmetic, prior certificates and zero-work proof reuse.'
        ],
        'unit_coverage':{'functions':14,'baseline_functions':12,'added_preservation_guards':2,
            'payload_bits':[65,4097,65537,1000001],'rational_literal_variants':17,
            'integer_literal_variants':['Int','IntegralAtan'],'integer_signs':['positive','negative'],
            'comparison_values':['equal','unequal'],'scaled_log_shifts':[7,8],'quantile_precisions':[-16,-17],
            'cutpoints':'Every integer through successful cost plus two for Int, Ratio and lazy Ratio, equal/unequal, at depths zero and eighty.',
            'public_sign_bits':[4097,65537],'public_sign_allowances':[32,1000000000,0],
            'shared_dag_depth':24,'mixed_representations':'Both operand orders; exact reduced/unreduced equality and canonicalization cache warming.'},
        'cap':cap,'observations':observations,'allocation_observations':allocations,
        'measured_work_observations':[s for s in observations if 'needed=' in s or 'successful work=' in s],
        'allocation_scope':baseline['allocation_scope'],
        'runner_path_correction':{'failed_phase':failed.name,
            'reason':'The helper path was relative to the repository root, but validation runs external commands from the isolated source directory. No test ran.',
            'corrected_phase':'typed-api-prototype-v1027-structural-comparison-repair-cap',
            'correction':'Use the absolute helper path; all twelve frozen tests passed before the two additional preservation guards.'},
        'passing_validation':passes,'replays':replays,'cost_records':records,
        'original_trial_percent_changes':costs,'paired_cost':'structural-comparison-paired-cost-v1042/summary.json',
        'paired_percent_changes':{k:[round(r['median_percent_change'],2) for r in v] for k,v in paired['cases'].items()},
        'targeted_confirmation':'structural-comparison-targeted-confirmation-v1042/summary.json',
        'confirmation_percent_changes':{k:[round(r['median_percent_change'],2) for r in v] for k,v in confirmation['cases'].items()},
        'executable_sizes':{k:v['executable_bytes'] for k,v in paired['provenance'].items()},
        'limits':[
            'Payload accounting repairs the frozen comparison and preflight gaps; cooperative units do not bound individual exact-operation CPU, native allocation, elapsed time or memory.',
            'Allocation evidence counts Rust requests in specific synchronous calls, excluding fixture setup and native allocation; it is not RSS or peak-live memory.',
            'Finite tests and replays do not exhaust concurrent, reconstructed or cross-root warming schedules, or every unshared symbolic expression.',
            'Changed runtime remains outside the conditional Verus target; all thirteen whole-crate proof groups remain open.',
            'Main retains the original scalar failure. All 49 original worktree entries, 39 properties, 14 targets and crate future work retain their scope and status.'
        ]})
    print(json.dumps({'source':EXPECTED,'replay_cases':1584,'original_cost_changes':costs,
        'paired_changes':{k:[round(r['median_percent_change'],2) for r in v] for k,v in paired['cases'].items()}}))


if __name__ == '__main__':
    main()
