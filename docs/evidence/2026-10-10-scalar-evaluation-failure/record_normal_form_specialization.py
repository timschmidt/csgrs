#!/usr/bin/env python3
"""Retain specialization only with identical budgeted outcomes and measured costs."""
import difflib
import gzip
import json
from pathlib import Path
import sys

sys.dont_write_bytecode = True
import record_algebraic_budget_implementation as evidence

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / 'target/scalar-evaluation-candidate/source'
read, sha, write = evidence.read, evidence.sha, evidence.write
EXPECTED = 'efe4009ef52bd9669d68ef548b496419a795e1162393eb9b1bbe9e3ebeb6e90d'
SOURCE_RECORD = 'typed-api-prototype-v691-normal-form-scalar-tests/source-hashes.json.gz'


def main():
    assert sha(E / SOURCE_RECORD) == EXPECTED
    old_record = read(E / 'normal-form-budget-implementation-v657.json')['source_record']
    old = json.loads(gzip.decompress((E / old_record).read_bytes()))
    source = json.loads(gzip.decompress((E / SOURCE_RECORD).read_bytes()))
    assert old.keys() == source.keys()
    changed = {n: {'before':old[n], 'after':v} for n,v in source.items() if old[n] != v}
    assert set(changed) == {'crates/hyperreal/src/computable/node/'+n for n in ['algebra.rs','symbolic_work.rs','normal_form_budget_tests.rs']}
    assert all(sha(SOURCE / n) == v for n,v in source.items())
    passes = {}
    for phase in ['typed-api-prototype-v690-normal-form-specialized-cap','typed-api-prototype-v691-normal-form-scalar-tests','typed-api-prototype-v692-normal-form-clippy','typed-api-prototype-v693-normal-form-replay-build']:
        r = read(E/phase/'result.json')
        assert r['exit_code'] == 0 and r['source_unchanged'] and r['main_unchanged']
        assert sha(E/phase/'source-hashes.json.gz') == EXPECTED
        passes[phase] = r
    assert '8 passed; 0 failed;' in (E/'typed-api-prototype-v690-normal-form-specialized-cap/stdout.log').read_text()
    assert '978 passed; 0 failed;' in (E/'typed-api-prototype-v691-normal-form-scalar-tests/stdout.log').read_text()
    assert (E/'typed-api-prototype-v690-normal-form-specialized-cap/stderr.log').read_text() == (E/'typed-api-prototype-v647-normal-form-fast-cap/stderr.log').read_text()
    assert not any(r.startswith('warning:') for r in (E/'typed-api-prototype-v692-normal-form-clippy/stderr.log').read_text().splitlines())
    replays = {}
    for before, after, count in [
        ('typed-api-prototype-v651-normal-form-replay','typed-api-prototype-v694-normal-form-replay',384),
        ('typed-api-prototype-v655-normal-form-budget-replay','typed-api-prototype-v698-normal-form-budget-replay',384),
        ('typed-api-prototype-v656-normal-form-algebraic-replay','typed-api-prototype-v699-normal-form-algebraic-replay',288),
        ('typed-api-prototype-v657-normal-form-algebraic-wide-replay-v2','typed-api-prototype-v700-normal-form-algebraic-wide-replay',144),
    ]:
        a,b = evidence.check_artifacts(before), evidence.check_artifacts(after)
        assert a['cases'] == b['cases'] == count and b['source_hashes_sha256'] == EXPECTED
        assert a['incomplete_cases'] == b['incomplete_cases'] == 0
        for name in ['probe.rs','reproduce.py']:
            assert sha(E/before/name) == sha(E/after/name)
        ac = [evidence.contract({'command':r}) for r in read(E/before/'commands.json') if r['name'] not in ['format','build']]
        bc = [evidence.contract({'command':r}) for r in read(E/after/'commands.json') if r['name'] not in ['format','build']]
        assert ac == bc
        assert [{k:v for k,v in r.items() if k != 'command'} for r in read(E/before/'receipts.json')] == [{k:v for k,v in r.items() if k != 'command'} for r in read(E/after/'receipts.json')]
        replays[after] = {'cases':count, 'summary_sha256':sha(E/after/'summary.json'), 'all_answers_samples_and_work_counts_match':True}
    assert sum(r['cases'] for r in replays.values()) == 1200
    evidence.EXPECTED = EXPECTED
    records, costs = [], {}
    for category, before, after, checksum in [
        ('normal-form','typed-api-prototype-v580-normal-form-cost-baseline','typed-api-prototype-v695-normal-form-cost','certified'),
        ('default','typed-api-prototype-v525-default-evaluation-cost','typed-api-prototype-v696-normal-form-default-cost','checksums'),
        ('algebraic','typed-api-prototype-v524-algebraic-cost','typed-api-prototype-v697-normal-form-algebraic-cost','certified'),
    ]:
        record = 'normal-form-'+category+'-cost-comparison-v700.json'
        costs[category] = evidence.cost(before,after,record,checksum)
        records.append(record)
    paired = read(E/'normal-form-paired-cost-v700/summary.json')
    assert paired['source_record_sha256'] == EXPECTED and paired['guards_unchanged'] and paired['processes'] == 324
    for n,v in paired['artifacts_sha256'].items(): assert sha(E/'normal-form-paired-cost-v700'/n) == v
    percentages = {k:[round(r['median_percent_change'],2) for r in v] for k,v in paired['cases'].items()}
    size = {k:{**v['executable_bytes'], 'percent_change':100*(v['executable_bytes']['candidate']/v['executable_bytes']['baseline']-1)} for k,v in paired['provenance'].items()}
    # Provide a reviewable delta against the last complete checkpoint, not just
    # whole-source copies from the preparation baseline.
    previous = read(E/'typed-api-prototype-v578-state.json')
    previous_phase = (E/previous['source_record']).parent
    previous_source = json.loads(gzip.decompress((E/previous['source_record']).read_bytes()))
    diff = []
    for n,v in sorted(source.items()):
        if previous_source.get(n) == v: continue
        before_text = (previous_phase/'inputs'/n).read_text() if n in previous_source else ''
        diff.extend(difflib.unified_diff(before_text.splitlines(keepends=True),(SOURCE/n).read_text().splitlines(keepends=True),fromfile='a/'+n if n in previous_source else '/dev/null',tofile='b/'+n))
    patch = E/'normal-form-source-diff-v700.patch'; assert not patch.exists(); patch.write_text(''.join(diff))
    write('normal-form-specialization-v700.json',{
        'status':'Isolated specialized accounting passes unchanged regressions, 1200 identical budgeted replay outcomes and scoped cost checks; complete checkpoint pending; no runtime admission',
        'source_record':SOURCE_RECORD,'source_record_sha256':EXPECTED,'prior_record':'normal-form-budget-implementation-v657.json','prior_implementation_commit':'6f60f9545',
        'changed_sources':changed,'source_delta_patch':patch.name,'source_delta_patch_sha256':sha(patch),
        'change':'Dispatch once on the active allowance to private const-generic Laurent/coefficient operations. The ordinary branch omits dynamic work checks; the active branch retains every charge and fallible comparison. Inline the small work adapters so the inactive branches disappear.',
        'source_level_argument':'Temporary synchronous recognition has no external callbacks or API introducing a new caller allowance. The dispatch flag is never stored in a node, memoized fact or public value; active includes exhausted scopes. No public API, arithmetic identity or recognition limit changes.',
        'replays':replays,'all_1200_answers_samples_and_work_counts_equal_to_dynamic_mode':True,
        'cost_records':records,'observed_original_median_percent_changes':costs,
        'paired_cost':'normal-form-paired-cost-v700/summary.json','paired_median_percent_changes':percentages,
        'paired_executable_sizes':size,'paired_comparison_scope':paired['scope'],
        'retention_decision':'Keep the specialized isolated implementation for full checkpoint validation. The prior paired wide-pi/reciprocal regressions were about 17/14 percent; final paired increases across the three matrices stay below 7 percent. Original trials, including their larger outliers, remain archived. No universal speedup is claimed.',
        'passing_final_source_validation':passes,
        'limits':[
            'Size-weighted cooperative units do not bound CPU instructions, elapsed time or temporary bytes inside an exact integer/rational operation.',
            'Original and alternating-order local timing observations vary. Standalone probe executable sizes are a scoped specialization tradeoff, not deployed library or application size measurements.',
            'The paired runs hash retained executable bytes before/after execution; original harness records bind source/library identities but did not retain those executable hashes.',
            'The runtime remains outside the conditional Verus target. All thirteen whole-crate proof groups, all 39 properties, all 14 targets and crate future work retain their existing statuses.'
        ]})
    print(json.dumps({'source':EXPECTED,'replay_cases':1200,'paired_percent_changes':percentages,'probe_sizes':size}))


if __name__ == '__main__':
    main()
