#!/usr/bin/env python3
"""Retain borrowed-leaf allocation, work, replay and ordinary-cost evidence."""
import difflib
import gzip
import json
from pathlib import Path
import re
import sys

sys.dont_write_bytecode = True
import record_algebraic_budget_implementation as evidence

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT/'target/scalar-evaluation-candidate/source'
EXPECTED = '87fa94bb1a57d9eacce449f06a8a947d265fe57a9fabe0dc3b80a64bc1c8f6de'
read, sha, write = evidence.read, evidence.sha, evidence.write


def main():
    record = E/'typed-api-prototype-v842-algebraic-leaf-build/source-hashes.json.gz'
    assert sha(record) == EXPECTED
    source = json.loads(gzip.decompress(record.read_bytes()))
    previous = read(E/'typed-api-prototype-v834-state.json')
    old = json.loads(gzip.decompress((E/previous['source_record']).read_bytes()))
    changed = {n: {'before':old.get(n), 'after':v} for n,v in source.items() if old.get(n) != v}
    assert set(changed) == {'crates/hyperreal/src/computable/'+n for n in
                            ['node.rs','node/structural_analysis.rs','node/algebraic_leaf_tests.rs']}
    assert all(source[n] == v for n,v in old.items() if 'test' in n)
    assert all(sha(SOURCE/n) == v for n,v in source.items())
    baseline = read(E/'algebraic-leaf-baseline-v837.json')
    before_record = E/baseline['source_record']
    before = json.loads(gzip.decompress(before_record.read_bytes()))
    runtime = 'crates/hyperreal/src/computable/node/structural_analysis.rs'
    assert [n for n,v in source.items() if before.get(n) != v] == [runtime]
    patch = E/'algebraic-leaf-runtime-diff-v852.patch'
    assert not patch.exists()
    patch.write_text(''.join(difflib.unified_diff(
        (before_record.parent/'inputs'/runtime).read_text().splitlines(True),
        (record.parent/'inputs'/runtime).read_text().splitlines(True),
        fromfile='before/'+runtime, tofile='after/'+runtime)))
    passes = {}
    for phase in ['typed-api-prototype-v838-algebraic-leaf-repair-build',
                  'typed-api-prototype-v839-algebraic-leaf-repair-cap',
                  'typed-api-prototype-v840-algebraic-leaf-unit-tests',
                  'typed-api-prototype-v841-algebraic-leaf-clippy',
                  'typed-api-prototype-v842-algebraic-leaf-build']:
        r = read(E/phase/'result.json')
        assert r['exit_code'] == 0 and r['source_unchanged'] and r['main_unchanged']
        assert sha(E/phase/'source-hashes.json.gz') == EXPECTED
        passes[phase] = r
    assert '993 passed; 0 failed;' in (E/'typed-api-prototype-v840-algebraic-leaf-unit-tests/stdout.log').read_text()
    cap = E/'typed-api-prototype-v839-algebraic-leaf-repair-cap'
    assert '6 passed; 0 failed;' in (cap/'stdout.log').read_text()
    observations = (cap/'stderr.log').read_text().splitlines()
    allocation_rows = [r for r in observations if 'allocations=Allocations' in r]
    assert len(allocation_rows) == 35
    for row in allocation_rows:
        if row.startswith(('leaf rejection ', 'leaf interrupted copy ')):
            assert row.endswith('allocations=Allocations { calls: 0, bytes: 0, largest: 0 }')
        if row.startswith('leaf rejected base '):
            assert row.endswith('allocations=Allocations { calls: 6, bytes: 168, largest: 104 }')
    assert 'leaf dependency successful work=34' in observations
    assert 'leaf metadata ratio=false, successful work=5' in observations
    assert 'leaf metadata ratio=true, successful work=6' in observations
    replays = {}
    for a,b,count in [
        ('typed-api-prototype-v795-inverse-trig-replay','typed-api-prototype-v843-algebraic-leaf-inverse-replay',384),
        ('typed-api-prototype-v800-inverse-trig-evaluation-replay','typed-api-prototype-v848-algebraic-leaf-evaluation-replay',384),
        ('typed-api-prototype-v801-inverse-trig-algebraic-replay','typed-api-prototype-v849-algebraic-leaf-algebraic-replay',288),
        ('typed-api-prototype-v802-inverse-trig-wide-replay','typed-api-prototype-v850-algebraic-leaf-wide-replay',144),
        ('typed-api-prototype-v803-inverse-trig-normal-form-replay','typed-api-prototype-v851-algebraic-leaf-normal-form-replay',384),
    ]:
        data = evidence.check_artifacts(b)
        assert data['cases'] == count and data['incomplete_cases'] == 0 and data['source_hashes_sha256'] == EXPECTED
        for key in ['invalid_samples','invalid_numeric_samples','typed_errors']:
            assert data.get(key,0) == 0
        for name in ['probe.rs','reproduce.py']: assert sha(E/a/name) == sha(E/b/name)
        old_rows,new_rows = read(E/a/'receipts.json'),read(E/b/'receipts.json')
        assert [evidence.contract(r) for r in old_rows] == [evidence.contract(r) for r in new_rows]
        replays[b] = {'cases':count,'summary_sha256':sha(E/b/'summary.json'),
                      'probe_harness_inputs_order_and_caps_preserved':True,
                      'receipts_identical_except_commands':
                          [{k:v for k,v in r.items() if k != 'command'} for r in old_rows] ==
                          [{k:v for k,v in r.items() if k != 'command'} for r in new_rows]}
    evidence.EXPECTED = EXPECTED
    costs,records = {},[]
    for family,a,b,checksum in [
        ('inverse-trig','typed-api-prototype-v796-inverse-trig-cost','typed-api-prototype-v844-algebraic-leaf-inverse-cost','certified'),
        ('normal-form','typed-api-prototype-v797-inverse-trig-normal-form-cost','typed-api-prototype-v845-algebraic-leaf-normal-form-cost','certified'),
        ('default','typed-api-prototype-v798-inverse-trig-default-cost','typed-api-prototype-v846-algebraic-leaf-default-cost','checksums'),
        ('algebraic','typed-api-prototype-v799-inverse-trig-algebraic-cost','typed-api-prototype-v847-algebraic-leaf-algebraic-cost','certified'),
    ]:
        name = 'algebraic-leaf-'+family+'-cost-comparison-v852.json'
        costs[family] = evidence.cost(a,b,name,checksum)
        records.append(name)
    paired = read(E/'algebraic-leaf-paired-cost-v852/summary.json')
    assert paired['source_record_sha256'] == EXPECTED and paired['processes'] == 432 and paired['guards_unchanged']
    for n,h in paired['artifacts_sha256'].items(): assert sha(E/'algebraic-leaf-paired-cost-v852'/n) == h
    write('algebraic-leaf-implementation-v852.json',{
        'status':'Scoped algebraic leaf repair passes allocation regressions, scalar tests, strict Clippy and frozen replay/cost workloads; full checkpoint pending; no runtime admission',
        'previous_checkpoint':'typed-api-prototype-v834-state.json','baseline_commit':'cae2e55f8',
        'baseline_record':'algebraic-leaf-baseline-v837.json',
        'source_record':str(record.relative_to(E)),'source_record_sha256':EXPECTED,
        'changed_sources':changed,'all_prior_named_tests_unchanged':True,
        'regression_test_bytes_unchanged_since_baseline':True,
        'changes':[
            'Check root degree, radicand sign and dependency coefficient widths from borrowed leaf metadata before rational conversion.',
            'Charge supported integer copies and rational reads before extraction. Propagate shared exhaustion separately from a declined candidate/base search.',
            'Compute integer separation metadata from borrowed magnitude without constructing a Rational. Reject impossible logarithm widths before scanning and charge supported integer/rational scans.',
            'Preserve the exact power-of-two boundary and conservative dependency semantics; interrupted searches cannot append a generator or publish numeric/fact state.'
        ],
        'unit_coverage':{'functions':6,'root_rejection_bits':[4097,65537,1000001],
            'root_rejection_kinds':['oversized','negative','different_degree','ratio'],
            'interrupted_integer_copy_bits':[16,256,1024,4096],
            'integer_metadata_bits':[16,4096,131073,131074,1000001],'integer_metadata_signs':['positive','negative'],
            'metadata_nonpower_boundary':'2^131072+1 as integer and reciprocal rational',
            'dependency_allowances':'Every integer from zero through successful cost 34 plus two',
            'metadata_allowances':'Every integer from zero through successful costs 5 (integer) / 6 (ratio) plus two',
            'local_limit_rejection':'One-unit budget rejects million-bit integer/ratio metadata without marking shared exhaustion'},
        'allocation_observations':allocation_rows,'measured_work_observations':[r for r in observations if 'successful work=' in r],
        'allocation_scope':baseline['allocation_scope'],
        'passing_validation':passes,'replays':replays,'cost_records':records,
        'original_trial_percent_changes':costs,'paired_cost':'algebraic-leaf-paired-cost-v852/summary.json',
        'paired_percent_changes':{k:[round(r['median_percent_change'],2) for r in v] for k,v in paired['cases'].items()},
        'executable_sizes':{k:v['executable_bytes'] for k,v in paired['provenance'].items()},
        'limits':[
            'Thread-local allocation requests cover only the synchronous tested query, excluding fixture and budget creation, other threads and native allocation; they are not RSS or peak-live-byte measurements.',
            'Cooperative units do not bound CPU instructions, elapsed time or temporary bytes inside each exact arithmetic operation.',
            'Separate and alternating process timings are local observations for 24 unchanged fixtures. Standalone executable sizes do not establish deployed or whole-application size.',
            'Six regressions and finite replay matrices do not exhaust kernel interruption points, reconstructed graphs, concurrency or cross-root warming.',
            'The changed runtime is outside the conditional Verus target. All thirteen whole-crate proof groups remain open.',
            'Main runtime remains unadmitted; all 39 properties, 14 targets and crate future work retain their previous statuses.'
        ]})
    print(json.dumps({'replay_cases':sum(v['cases'] for v in replays.values()),'original_cost_changes':costs,
        'paired_changes':{k:[round(r['median_percent_change'],2) for r in v] for k,v in paired['cases'].items()}}))


if __name__ == '__main__':
    main()
