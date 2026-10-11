#!/usr/bin/env python3
"""Retain depth-aware inverse-trig DAG memoization and its measured tradeoffs."""
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
EXPECTED = '6f594409d486dfc6b4b806139e480c91f385607989ef670ebdf2262140a2132b'
read, sha, write = evidence.read, evidence.sha, evidence.write


def main():
    record = E/'typed-api-prototype-v955-inverse-dag-build/source-hashes.json.gz'
    assert sha(record) == EXPECTED
    source = json.loads(gzip.decompress(record.read_bytes()))
    prior = read(E/'typed-api-prototype-v927-state.json')
    before = json.loads(gzip.decompress((E/prior['source_record']).read_bytes()))
    changed = {n: {'before': before.get(n), 'after': v} for n,v in source.items() if before.get(n) != v}
    prefix = 'crates/hyperreal/src/computable/'
    runtime = prefix+'node/algebra.rs'
    test = prefix+'node/inverse_trig_dag_tests.rs'
    allocator = prefix+'node/algebraic_leaf_tests.rs'
    assert set(changed) == {runtime, test, allocator, prefix+'node.rs'}
    assert all(source[n] == v for n,v in before.items() if 'test' in n and n != allocator)
    assert all(sha(SOURCE/n) == v for n,v in source.items())
    old_allocator = E/Path(prior['source_record']).parent/'inputs'/allocator
    assert (SOURCE/allocator).read_text().replace('pub(super) ', '') == old_allocator.read_text()
    baseline = E/'typed-api-prototype-v945-inverse-dag-isolated-baseline-build'
    baseline_sources = json.loads(gzip.decompress((baseline/'source-hashes.json.gz').read_bytes()))
    assert [n for n,v in source.items() if baseline_sources.get(n) != v] == [runtime]
    patch = E/'inverse-dag-runtime-diff-v965.patch'
    assert not patch.exists()
    patch.write_text(''.join(difflib.unified_diff(
        (baseline/'inputs'/runtime).read_text().splitlines(True),
        (record.parent/'inputs'/runtime).read_text().splitlines(True),
        fromfile='before/'+runtime, tofile='after/'+runtime)))
    original_test = (E/'typed-api-prototype-v940-inverse-dag-allocation-build/inputs'/test).read_text()
    current_test = (SOURCE/test).read_text()
    insertion_start = current_test.index('        // Other tests can fill the global')
    insertion_end = current_test.index('        let mut observations = Vec::new();', insertion_start)
    assert current_test[:insertion_start] + current_test[insertion_end:] == original_test
    test_patch = E/'inverse-dag-allocation-isolation-v965.patch'
    assert not test_patch.exists()
    test_patch.write_text(''.join(difflib.unified_diff(original_test.splitlines(True), current_test.splitlines(True),
        fromfile='before/'+test, tofile='after/'+test)))
    failed = read(E/'typed-api-prototype-v944-inverse-dag-unit-tests/result.json')
    assert failed['exit_code'] == 101 and failed['source_unchanged'] and failed['main_unchanged']
    assert '1002 passed; 1 failed;' in (E/'typed-api-prototype-v944-inverse-dag-unit-tests/stdout.log').read_text()
    isolated_failure = E/'typed-api-prototype-v946-inverse-dag-isolated-baseline-cap'
    assert read(isolated_failure/'result.json')['exit_code'] == 101
    assert 'calls: 4096, bytes: 98304' in (isolated_failure/'stderr.log').read_text()
    passes = {}
    for version, suffix in [(951,'isolated-repair-build'), (952,'repair-cap'), (953,'unit-tests'), (954,'clippy'), (955,'build')]:
        phase = f'typed-api-prototype-v{version}-inverse-dag-{suffix}'
        r = read(E/phase/'result.json')
        assert r['exit_code'] == 0 and r['source_unchanged'] and r['main_unchanged']
        assert sha(E/phase/'source-hashes.json.gz') == EXPECTED
        passes[phase] = r
    assert '1003 passed; 0 failed;' in (E/'typed-api-prototype-v953-inverse-dag-unit-tests/stdout.log').read_text()
    assert not any(s.startswith('warning:') for s in (E/'typed-api-prototype-v954-inverse-dag-clippy/stderr.log').read_text().splitlines())
    observations = (E/'typed-api-prototype-v952-inverse-dag-repair-cap/stderr.log').read_text().splitlines()
    allocation_rows = [s for s in observations if s.startswith('inverse-trig DAG allocation ')]
    assert len(allocation_rows) == 6
    replays = {}
    for a,b,count in [
        ('v888-algebraic-leaf-inverse-replay','v956-inverse-dag-inverse-replay',384),
        ('v893-algebraic-leaf-evaluation-replay','v961-inverse-dag-evaluation-replay',384),
        ('v894-algebraic-leaf-algebraic-replay','v962-inverse-dag-algebraic-replay',288),
        ('v895-algebraic-leaf-wide-replay','v963-inverse-dag-wide-replay',144),
        ('v896-algebraic-leaf-normal-form-replay','v964-inverse-dag-normal-form-replay',384),
    ]:
        a,b = 'typed-api-prototype-'+a, 'typed-api-prototype-'+b
        d = evidence.check_artifacts(b)
        assert d['cases'] == count and d['incomplete_cases'] == 0 and d['source_hashes_sha256'] == EXPECTED
        for key in ['invalid_samples','invalid_numeric_samples','typed_errors']: assert d.get(key,0) == 0
        for name in ['probe.rs','reproduce.py']: assert sha(E/a/name) == sha(E/b/name)
        old_rows,new_rows = read(E/a/'receipts.json'), read(E/b/'receipts.json')
        assert [evidence.contract(r) for r in old_rows] == [evidence.contract(r) for r in new_rows]
        replays[b] = {'cases':count, 'summary_sha256':sha(E/b/'summary.json'),
            'probe_harness_inputs_order_and_caps_preserved':True,
            'receipts_identical_except_commands':[{k:v for k,v in r.items() if k != 'command'} for r in old_rows] == [{k:v for k,v in r.items() if k != 'command'} for r in new_rows]}
    evidence.EXPECTED = EXPECTED
    costs, records = {}, []
    for family,a,b,checksum in [
        ('inverse-trig','v889-algebraic-leaf-inverse-cost','v957-inverse-dag-inverse-cost','certified'),
        ('normal-form','v890-algebraic-leaf-normal-form-cost','v958-inverse-dag-normal-form-cost','certified'),
        ('default','v891-algebraic-leaf-default-cost','v959-inverse-dag-default-cost','checksums'),
        ('algebraic','v892-algebraic-leaf-algebraic-cost','v960-inverse-dag-algebraic-cost','certified'),
    ]:
        name = 'inverse-dag-'+family+'-cost-comparison-v965.json'
        costs[family] = evidence.cost('typed-api-prototype-'+a,'typed-api-prototype-'+b,name,checksum)
        records.append(name)
    paired = read(E/'inverse-dag-paired-cost-v965/summary.json')
    assert paired['source_record_sha256'] == EXPECTED and paired['processes'] == 432 and paired['guards_unchanged']
    for n,h in paired['artifacts_sha256'].items(): assert sha(E/'inverse-dag-paired-cost-v965'/n) == h
    write('inverse-dag-implementation-v965.json', {
        'status':'Scoped inverse-trig shared-DAG repair passes resource regressions, scalar tests, strict Clippy and frozen replay/cost workloads; full checkpoint pending; no runtime admission',
        'previous_checkpoint':'typed-api-prototype-v927-state.json', 'baseline_commit':'bab220336',
        'baseline_record':'inverse-dag-baseline-v941.json',
        'source_record':str(record.relative_to(E)), 'source_record_sha256':EXPECTED, 'changed_sources':changed,
        'prior_named_test_bodies_and_assertions_unchanged':True,
        'regression_values_depths_caps_and_assertions_unchanged':True,
        'allocation_isolation_patch':test_patch.name,
        'allocation_test_correction':{'failed_parallel_suite':'typed-api-prototype-v944-inverse-dag-unit-tests',
            'cause':'Global small-rational arithmetic cache initialization and limited retained slots made concurrent small/large allocation ratios dependent on other tests.',
            'correction':'Run only the allocation test body in a fresh process with one test thread. All original fixtures, measurement windows, exact answers and growth assertions are byte-identical.',
            'old_runtime_recheck':isolated_failure.name,
            'child_accounting':'One additional child test, counted separately from the ten new top-level tests. Capped parent and child inherit the address-space and CPU rlimits, with a single parent wall timeout.'},
        'clippy_correction':'v950 rejected a needless bool::then closure; then_some preserves the exact key and branch semantics.',
        'changes':[
            'Memoize completed inverse-trig forms only for shared compound descendants within one synchronous query.',
            'Key by node identity and remaining depth, preserving path-specific recognition limits and exact form/argument ordering.',
            'Keep memo allocation lazy; ordinary leaves, atoms and root calls avoid table insertion.',
            'Debit form-copy coefficient work before a memo hit or insertion; discard the query-local table on every return, including interruption.',
            'Retain finite-domain guards, all previous certificate paths and numeric-cache publication behavior; no public API or permanent failure cache.'
        ],
        'unit_coverage':{'top_level_functions':10,'allocation_child_tests':1,'parsers':['atan linear','atan terms','acos linear'],
            'doubling_depth':28,'doubling_coefficient':'2^28','different_path_depth':24,'different_path_coefficient':'3^24',
            'path_specific_depths':list(range(8)),'path_orders':['shallow-first','deep-first'],
            'work_allowance':10000,'cutpoints':'Every integer through successful depth-five work plus two, with facts/cache conservation and unlimited retry.',
            'reconstruction_depths':list(range(8)),'undefined_dag_depth':24,'allocation_depths':[6,12],'allocation_growth_limit':8},
        'allocation_observations':allocation_rows,
        'measured_work_observations':[s for s in observations if 'successful work=' in s],
        'allocation_scope':read(E/'inverse-dag-baseline-v941.json')['allocation_scope'],
        'passing_validation':passes, 'replays':replays, 'cost_records':records,
        'original_trial_percent_changes':costs, 'paired_cost':'inverse-dag-paired-cost-v965/summary.json',
        'paired_percent_changes':{k:[round(r['median_percent_change'],2) for r in v] for k,v in paired['cases'].items()},
        'executable_sizes':{k:v['executable_bytes'] for k,v in paired['provenance'].items()},
        'limits':[
            'The memo eliminates repeated expansion in the frozen shared-DAG fixtures. General unshared graph size, coefficient growth, structural comparison cost and individual exact-operation CPU/native allocation remain separate concerns.',
            'Linear parsers allocate additional query-local memo storage. Allocation evidence counts Rust requests in a controlled process, not RSS, peak-live memory or all native allocation.',
            'Node identities are kept live by the borrowed immutable root; the table is never retained or iterated to choose form ordering.',
            'Cooperative work units do not bound CPU instructions, time or allocation inside each exact operation.',
            'Local timings and standalone probe sizes preserve frozen inputs but do not establish a uniform speedup or whole-application admission.',
            'Finite tests/replays do not exhaust interruption, reconstruction, concurrency or cross-root warming schedules.',
            'Changed runtime remains outside the conditional Verus target; all thirteen whole-crate proof groups remain open.',
            'Main retains the original scalar failure. All 49 original worktree entries, 39 properties, 14 targets and crate future work retain their scope and status.'
        ]})
    print(json.dumps({'replay_cases':sum(v['cases'] for v in replays.values()),'original_cost_changes':costs,
        'paired_changes':{k:[round(r['median_percent_change'],2) for r in v] for k,v in paired['cases'].items()}}))


if __name__ == '__main__':
    main()
