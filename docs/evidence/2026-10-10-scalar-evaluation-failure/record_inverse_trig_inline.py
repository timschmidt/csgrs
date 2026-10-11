#!/usr/bin/env python3
"""Check unchanged inverse-trig accounting and retain all timing observations."""
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
EXPECTED = '4afe53ae2773719d18b0c314fc1a89cc43adeed8a3540cfd9ab0d56f27beb45c'
read, sha, write = evidence.read, evidence.sha, evidence.write


def main():
    record = E/'typed-api-prototype-v793-inverse-trig-inline-build/source-hashes.json.gz'
    assert sha(record) == EXPECTED
    source = json.loads(gzip.decompress(record.read_bytes()))
    old_record = E/'typed-api-prototype-v748-inverse-trig-replay-build/source-hashes.json.gz'
    before = json.loads(gzip.decompress(old_record.read_bytes()))
    changed = [n for n,v in source.items() if before.get(n) != v]
    assert changed == ['crates/hyperreal/src/computable/node/algebra.rs']
    assert all(sha(SOURCE/n) == v for n,v in source.items())
    old = (old_record.parent/'inputs'/changed[0]).read_text()
    new = (record.parent/'inputs'/changed[0]).read_text()
    assert new.count('    #[inline]\n') == 6 and new.replace('    #[inline]\n','') == old
    patch = E/'inverse-trig-inline-diff-v804.patch'
    assert not patch.exists()
    patch.write_text(''.join(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile='before/'+changed[0],tofile='after/'+changed[0])))
    passes = {}
    for phase in ['typed-api-prototype-v791-inverse-trig-inline-tests','typed-api-prototype-v792-inverse-trig-inline-clippy','typed-api-prototype-v793-inverse-trig-inline-build','typed-api-prototype-v794-inverse-trig-repair-cap']:
        r = read(E/phase/'result.json')
        assert r['exit_code'] == 0 and r['source_unchanged'] and r['main_unchanged']
        assert sha(E/phase/'source-hashes.json.gz') == EXPECTED
        passes[phase] = r
    assert '987 passed; 0 failed;' in (E/'typed-api-prototype-v791-inverse-trig-inline-tests/stdout.log').read_text()
    assert '9 passed; 0 failed;' in (E/'typed-api-prototype-v794-inverse-trig-repair-cap/stdout.log').read_text()
    replays = {}
    for prior, current, suffix, count in [(750,795,'inverse-trig-replay',384),(755,800,'inverse-trig-evaluation-replay',384),(756,801,'inverse-trig-algebraic-replay',288),(757,802,'inverse-trig-wide-replay',144),(758,803,'inverse-trig-normal-form-replay',384)]:
        a,b = f'typed-api-prototype-v{prior}-{suffix}',f'typed-api-prototype-v{current}-{suffix}'
        data = evidence.check_artifacts(b)
        assert data['cases'] == count and data['incomplete_cases'] == 0 and data['source_hashes_sha256'] == EXPECTED
        for name in ['probe.rs','reproduce.py']: assert sha(E/a/name) == sha(E/b/name)
        old,new = read(E/a/'receipts.json'),read(E/b/'receipts.json')
        assert [evidence.contract(r) for r in old] == [evidence.contract(r) for r in new]
        assert [{k:v for k,v in r.items() if k != 'command'} for r in old] == [{k:v for k,v in r.items() if k != 'command'} for r in new]
        replays[b] = {'cases':count,'summary_sha256':sha(E/b/'summary.json'),'answers_samples_work_and_proofs_match_initial_repair':True}
    evidence.EXPECTED = EXPECTED
    costs,records = {},[]
    for family,a,b,checksum in [
        ('inverse-trig','typed-api-prototype-v732-inverse-trig-cost-baseline','typed-api-prototype-v796-inverse-trig-cost','certified'),
        ('normal-form','typed-api-prototype-v695-normal-form-cost','typed-api-prototype-v797-inverse-trig-normal-form-cost','certified'),
        ('default','typed-api-prototype-v696-normal-form-default-cost','typed-api-prototype-v798-inverse-trig-default-cost','checksums'),
        ('algebraic','typed-api-prototype-v697-normal-form-algebraic-cost','typed-api-prototype-v799-inverse-trig-algebraic-cost','certified'),
    ]:
        name = 'inverse-trig-'+family+'-cost-comparison-v804.json'
        costs[family] = evidence.cost(a,b,name,checksum)
        records.append(name)
    paired = read(E/'inverse-trig-paired-cost-v804/summary.json')
    assert paired['source_record_sha256'] == EXPECTED and paired['processes'] == 432 and paired['guards_unchanged']
    for n,h in paired['artifacts_sha256'].items(): assert sha(E/'inverse-trig-paired-cost-v804'/n) == h
    write('inverse-trig-inline-v804.json',{
        'status':'Isolated inverse-trig inlining preserves all focused correctness and accounting results; full checkpoint pending; no runtime admission',
        'initial_implementation_commit':'9ed9e30bc','initial_trials':'inverse-trig-initial-trials-v760.json',
        'source_record':str(record.relative_to(E)),'source_record_sha256':EXPECTED,
        'change':'Add six inline annotations to fallible temporary-form add/scale operations so ordinary const-specialized calls can eliminate Option handling; all executable expressions and tests are byte-identical after removing the annotations.',
        'changed_sources':changed,'passing_validation':passes,'replays':replays,'cost_records':records,
        'original_trial_percent_changes':costs,'paired_cost':'inverse-trig-paired-cost-v804/summary.json',
        'paired_percent_changes':{k:[round(r['median_percent_change'],2) for r in v] for k,v in paired['cases'].items()},
        'executable_sizes':{k:v['executable_bytes'] for k,v in paired['provenance'].items()},
        'limits':['Cooperative units do not bound CPU instructions, elapsed time or temporary bytes within an exact operation.',
            'Independent and alternating-order measurements are local timing observations; no uniform speedup, allocation, deployed-size or whole-application claim.',
            'Baseline executable hashes match the retained v732 or v700 records, and paired executables are checked before and after every process. Original cost summaries bind source/library inputs but omit executable hashes.',
            'Nine regressions and finite public matrices do not exhaust concurrency, cross-root warming or every kernel interruption point.',
            'Main runtime remains unadmitted; all 39 properties, 14 targets, crate future work and thirteen whole-crate proof groups retain their existing statuses.']
    })
    print(json.dumps({'replay_cases':sum(v['cases'] for v in replays.values()),'paired_changes':{k:[round(r['median_percent_change'],2) for r in v] for k,v in paired['cases'].items()}}))


if __name__ == '__main__':
    main()
