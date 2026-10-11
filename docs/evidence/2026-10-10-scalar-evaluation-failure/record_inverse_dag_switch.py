#!/usr/bin/env python3
"""Retain ordinary-node dispatch tuning against unchanged DAG semantics."""
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
EXPECTED = '688d05cdbf832f87f9172da50a21a497134bc7fc0666ec046148fc787d70dacb'
read, sha, write = evidence.read, evidence.sha, evidence.write


def main():
    initial = read(E/'inverse-dag-implementation-v965.json')
    old_record = E/initial['source_record']
    record = E/'typed-api-prototype-v970-inverse-dag-build/source-hashes.json.gz'
    assert sha(record) == EXPECTED
    before = json.loads(gzip.decompress(old_record.read_bytes()))
    source = json.loads(gzip.decompress(record.read_bytes()))
    runtime = 'crates/hyperreal/src/computable/node/algebra.rs'
    assert [n for n,v in source.items() if before.get(n) != v] == [runtime]
    assert all(sha(SOURCE/n) == v for n,v in source.items())
    patch = E/'inverse-dag-switch-diff-v980.patch'
    assert not patch.exists()
    patch.write_text(''.join(difflib.unified_diff(
        (old_record.parent/'inputs'/runtime).read_text().splitlines(True),
        (record.parent/'inputs'/runtime).read_text().splitlines(True),
        fromfile='before/'+runtime,tofile='after/'+runtime)))
    passes = {}
    for v,suffix in [(966,'isolated-repair-build'),(967,'repair-cap'),(968,'unit-tests'),(969,'clippy'),(970,'build')]:
        phase = f'typed-api-prototype-v{v}-inverse-dag-{suffix}'
        r = read(E/phase/'result.json')
        assert r['exit_code'] == 0 and r['source_unchanged'] and r['main_unchanged']
        assert sha(E/phase/'source-hashes.json.gz') == EXPECTED
        passes[phase] = r
    assert '1003 passed; 0 failed;' in (E/'typed-api-prototype-v968-inverse-dag-unit-tests/stdout.log').read_text()
    observations = (E/'typed-api-prototype-v967-inverse-dag-repair-cap/stderr.log').read_text().splitlines()
    allocations = [s for s in observations if s.startswith('inverse-trig DAG allocation ')]
    work = [s for s in observations if 'successful work=' in s]
    assert allocations == initial['allocation_observations'] and work == initial['measured_work_observations']
    replays = {}
    for a,b,count in [('v956','v971',384),('v961','v976',384),('v962','v977',288),('v963','v978',144),('v964','v979',384)]:
        old = next(p for p in initial['replays'] if p.startswith('typed-api-prototype-'+a+'-'))
        new = old.replace(a,b,1)
        data = evidence.check_artifacts(new)
        assert data['cases'] == count and data['incomplete_cases'] == 0 and data['source_hashes_sha256'] == EXPECTED
        for key in ['invalid_samples','invalid_numeric_samples','typed_errors']: assert data.get(key,0) == 0
        for name in ['probe.rs','reproduce.py']: assert sha(E/old/name) == sha(E/new/name)
        old_rows,new_rows = read(E/old/'receipts.json'), read(E/new/'receipts.json')
        assert [evidence.contract(r) for r in old_rows] == [evidence.contract(r) for r in new_rows]
        strip = lambda rows: [{k:v for k,v in r.items() if k != 'command'} for r in rows]
        assert strip(old_rows) == strip(new_rows)
        replays[new] = {'cases':count,'summary_sha256':sha(E/new/'summary.json'),
            'probe_harness_inputs_order_and_caps_preserved':True,'receipts_identical_except_commands':True}
    evidence.EXPECTED = EXPECTED
    costs,records = {},[]
    for family,a,b,checksum in [
        ('inverse-trig','v889-algebraic-leaf-inverse-cost','v972-inverse-dag-inverse-cost','certified'),
        ('normal-form','v890-algebraic-leaf-normal-form-cost','v973-inverse-dag-normal-form-cost','certified'),
        ('default','v891-algebraic-leaf-default-cost','v974-inverse-dag-default-cost','checksums'),
        ('algebraic','v892-algebraic-leaf-algebraic-cost','v975-inverse-dag-algebraic-cost','certified'),
    ]:
        name = 'inverse-dag-'+family+'-cost-comparison-v980.json'
        costs[family] = evidence.cost('typed-api-prototype-'+a,'typed-api-prototype-'+b,name,checksum)
        records.append(name)
    paired = read(E/'inverse-dag-paired-cost-completion-v980/summary.json')
    assert paired['source_record_sha256'] == EXPECTED and paired['processes'] == 432 and paired['guards_unchanged']
    for n,h in paired['artifacts_sha256'].items(): assert sha(E/'inverse-dag-paired-cost-completion-v980'/n) == h
    assert paired['reused_successful_processes'] == 376 and paired['new_processes'] == 56
    assert paired['earlier_failed_processes'] == 1
    confirmation = read(E/'inverse-dag-inverse-confirmation-v980/summary.json')
    assert confirmation['source_record_sha256'] == EXPECTED and confirmation['guards_unchanged']
    assert confirmation['processes'] == 108
    for n,h in confirmation['artifacts_sha256'].items():
        assert sha(E/'inverse-dag-inverse-confirmation-v980'/n) == h
    changed = {n: r | {'after':source[n]} for n,r in initial['changed_sources'].items()}
    write('inverse-dag-switch-v980.json', initial | {
        'status':'Scoped shared-DAG memoization with inlined unshared-node dispatch; full checkpoint pending; no runtime admission',
        'source_record':str(record.relative_to(E)),'source_record_sha256':EXPECTED,'changed_sources':changed,
        'initial_implementation_record':'inverse-dag-implementation-v965.json','initial_implementation_commit':'f9889c1d6',
        'dispatch_diff':patch.name,'changes':initial['changes']+[
            'Inline memo dispatch and return directly for unshared nodes, retaining every memo key, coefficient charge, recognition path and depth limit.'
        ],
        'all_tests_unchanged_since_isolated_baseline':True,
        'focused_replay_answers_work_counts_proofs_and_allocations_identical_to_initial_repair':True,
        'passing_validation':passes,'replays':replays,'cost_records':records,
        'targeted_confirmation':'inverse-dag-inverse-confirmation-v980/summary.json',
        'confirmation_percent_changes':[round(r['median_percent_change'],2) for r in confirmation['cases']['inverse_trig']],
        'interrupted_paired_attempt':'inverse-dag-paired-failure-v980.json',
        'paired_completion_policy':'The unchanged baseline exited nonzero once; its numeric return code was not captured by the initial helper. All 376 saved successful measurements were verified and reused, and the unfinished 56 processes passed the unchanged limits. The failed attempt is retained separately.',
        'original_trial_percent_changes':costs,'paired_cost':'inverse-dag-paired-cost-completion-v980/summary.json',
        'paired_percent_changes':{k:[round(r['median_percent_change'],2) for r in v] for k,v in paired['cases'].items()},
        'executable_sizes':{k:v['executable_bytes'] for k,v in paired['provenance'].items()}
    })
    print(json.dumps({'source':EXPECTED,'identical_focused_replays':1584,'original_cost_changes':costs,
        'paired_changes':{k:[round(r['median_percent_change'],2) for r in v] for k,v in paired['cases'].items()}}))


if __name__ == '__main__':
    main()
