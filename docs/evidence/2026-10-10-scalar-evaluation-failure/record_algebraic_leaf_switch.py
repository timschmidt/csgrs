#!/usr/bin/env python3
"""Verify that parser-state reuse preserves allocation and accounting behavior."""
from copy import deepcopy
import difflib
import gzip
import json
from pathlib import Path
import sys

sys.dont_write_bytecode = True
import record_algebraic_budget_implementation as evidence

E = Path(__file__).resolve().parent
SOURCE = E.parents[2]/'target/scalar-evaluation-candidate/source'
EXPECTED = 'e61f29beff523a6b5f14cec81c10f09e4beb6f1bf70d507d89d9ca63c3a46e01'
read, sha, write = evidence.read, evidence.sha, evidence.write


def main():
    initial = read(E/'algebraic-leaf-implementation-v852.json')
    old_record = E/initial['source_record']
    record = E/'typed-api-prototype-v886-algebraic-leaf-switch-build/source-hashes.json.gz'
    assert sha(record) == EXPECTED
    old = json.loads(gzip.decompress(old_record.read_bytes()))
    source = json.loads(gzip.decompress(record.read_bytes()))
    assert all(sha(SOURCE/n) == v for n,v in source.items())
    runtime = 'crates/hyperreal/src/computable/node/structural_analysis.rs'
    assert [n for n,v in source.items() if old.get(n) != v] == [runtime]
    patch = E/'algebraic-leaf-switch-diff-v897.patch';assert not patch.exists()
    patch.write_text(''.join(difflib.unified_diff(
        (old_record.parent/'inputs'/runtime).read_text().splitlines(True),
        (record.parent/'inputs'/runtime).read_text().splitlines(True),
        fromfile='before/'+runtime,tofile='after/'+runtime)))
    passes = {}
    for phase in ['typed-api-prototype-v884-algebraic-leaf-switch-tests',
                  'typed-api-prototype-v885-algebraic-leaf-switch-clippy',
                  'typed-api-prototype-v886-algebraic-leaf-switch-build',
                  'typed-api-prototype-v887-algebraic-leaf-repair-cap']:
        r = read(E/phase/'result.json')
        assert r['exit_code'] == 0 and r['source_unchanged'] and r['main_unchanged']
        assert sha(E/phase/'source-hashes.json.gz') == EXPECTED
        passes[phase] = r
    assert '993 passed; 0 failed;' in (E/'typed-api-prototype-v884-algebraic-leaf-switch-tests/stdout.log').read_text()
    assert '6 passed; 0 failed;' in (E/'typed-api-prototype-v887-algebraic-leaf-repair-cap/stdout.log').read_text()
    rows = lambda phase:[r for r in (E/phase/'stderr.log').read_text().splitlines() if r.startswith('leaf ')]
    assert rows('typed-api-prototype-v839-algebraic-leaf-repair-cap') == rows('typed-api-prototype-v887-algebraic-leaf-repair-cap')
    replays = {}
    for a,b,suffix,count in [(843,888,'inverse',384),(848,893,'evaluation',384),
                            (849,894,'algebraic',288),(850,895,'wide',144),(851,896,'normal-form',384)]:
        before,after = f'typed-api-prototype-v{a}-algebraic-leaf-{suffix}-replay',f'typed-api-prototype-v{b}-algebraic-leaf-{suffix}-replay'
        data = evidence.check_artifacts(after)
        assert data['cases'] == count and data['source_hashes_sha256'] == EXPECTED and data['incomplete_cases'] == 0
        for n in ['probe.rs','reproduce.py']:assert sha(E/before/n) == sha(E/after/n)
        x,y = read(E/before/'receipts.json'),read(E/after/'receipts.json')
        assert [evidence.contract(r) for r in x] == [evidence.contract(r) for r in y]
        assert [{k:v for k,v in r.items() if k != 'command'} for r in x] == [{k:v for k,v in r.items() if k != 'command'} for r in y]
        replays[after] = {'cases':count,'summary_sha256':sha(E/after/'summary.json'),
            'probe_harness_inputs_order_and_caps_preserved':True,
            'answers_samples_work_and_proofs_match_initial_repair':True}
    evidence.EXPECTED = EXPECTED
    costs,records = {},[]
    for family,a,b,checksum in [
        ('inverse-trig','typed-api-prototype-v796-inverse-trig-cost','typed-api-prototype-v889-algebraic-leaf-inverse-cost','certified'),
        ('normal-form','typed-api-prototype-v797-inverse-trig-normal-form-cost','typed-api-prototype-v890-algebraic-leaf-normal-form-cost','certified'),
        ('default','typed-api-prototype-v798-inverse-trig-default-cost','typed-api-prototype-v891-algebraic-leaf-default-cost','checksums'),
        ('algebraic','typed-api-prototype-v799-inverse-trig-algebraic-cost','typed-api-prototype-v892-algebraic-leaf-algebraic-cost','certified'),
    ]:
        name = 'algebraic-leaf-'+family+'-cost-comparison-v897.json'
        costs[family] = evidence.cost(a,b,name,checksum)
        records.append(name)
    paired = read(E/'algebraic-leaf-paired-cost-v897/summary.json')
    assert paired['source_record_sha256'] == EXPECTED and paired['processes'] == 432 and paired['guards_unchanged']
    for n,h in paired['artifacts_sha256'].items():assert sha(E/'algebraic-leaf-paired-cost-v897'/n) == h
    result = deepcopy(initial)
    result['status'] = 'Parser-state reuse preserves the initial allocation and accounting repair; final-source focused validation passes; full checkpoint pending; no runtime admission'
    result['source_record'] = str(record.relative_to(E))
    result['source_record_sha256'] = EXPECTED
    result['changed_sources'][runtime]['after'] = source[runtime]
    result['changes'].append('Reuse the synchronous metadata parser budget-state switch for integer/rational scans. A test-only adapter preserves existing direct private-constructor tests; word charges and admission rules are unchanged.')
    result['passing_validation'] = passes
    result['replays'] = replays
    result['cost_records'] = records
    result['original_trial_percent_changes'] = costs
    result['paired_cost'] = 'algebraic-leaf-paired-cost-v897/summary.json'
    result['paired_percent_changes'] = {k:[round(r['median_percent_change'],2) for r in v] for k,v in paired['cases'].items()}
    result['executable_sizes'] = {k:v['executable_bytes'] for k,v in paired['provenance'].items()}
    result['initial_implementation_commit'] = 'efe2f1976'
    result['initial_performance_concern'] = 'algebraic-leaf-initial-retention-v852.json'
    result['intermediate_compile_correction'] = {'phase':'typed-api-prototype-v883-algebraic-leaf-switch-tests','change':'Restore the original one-argument private rational constructor as a test-only adapter. Two existing tests and their assertions remain byte-identical.'}
    result['all_focused_answers_samples_work_proofs_and_allocation_observations_preserved'] = True
    write('algebraic-leaf-switch-v897.json',result)
    print(json.dumps({'focused_replays':sum(v['cases'] for v in replays.values()),
        'original_cost_changes':costs,'paired_changes':result['paired_percent_changes'],
        'executable_sizes':result['executable_sizes']}))


if __name__ == '__main__':
    main()
