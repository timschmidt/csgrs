#!/usr/bin/env python3
"""Bind the algebraic preflight repair to frozen regressions and cost inputs."""
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
EXPECTED = '824a7ecd2822e2f06c5ee58d6090ce60aa8e69098d5774ecc0c7113f50caa54a'
read, sha, write = evidence.read, evidence.sha, evidence.write


def main():
    prior = read(E/'typed-api-prototype-v1074-state.json')
    baseline = read(E/'algebraic-preflight-baseline-v1085-corrected.json')
    record = E/'typed-api-prototype-v1090-algebraic-preflight-build/source-hashes.json.gz'
    assert sha(record) == EXPECTED
    source = json.loads(gzip.decompress(record.read_bytes()))
    before = json.loads(gzip.decompress((E/prior['source_record']).read_bytes()))
    assert all(sha(SOURCE/n) == v for n,v in source.items())
    changed = {n: {'before': before.get(n), 'after': v} for n,v in source.items() if before.get(n) != v}
    runtime = 'crates/hyperreal/src/computable/node/structural_analysis.rs'
    test = 'crates/hyperreal/src/computable/node/algebraic_preflight_tests.rs'
    assert set(changed) == {runtime, test, 'crates/hyperreal/src/computable/node.rs'}
    assert all(source[n] == v for n,v in before.items() if 'test' in n)
    initial_record = E/baseline['source_record']
    initial = json.loads(gzip.decompress(initial_record.read_bytes()))
    assert set(n for n,v in source.items() if initial.get(n) != v) == {runtime}
    assert source[test] == initial[test]
    patch = E/'algebraic-preflight-runtime-diff-v1100.patch'
    assert not patch.exists()
    old = initial_record.parent/'inputs'/runtime
    assert sha(old) == initial[runtime]
    patch.write_text(''.join(difflib.unified_diff(old.read_text().splitlines(True),
        (SOURCE/runtime).read_text().splitlines(True), fromfile='before/'+runtime, tofile='after/'+runtime)))
    passes = {}
    for version,suffix in [(1086,'repair-build'),(1087,'repair-cap'),(1088,'unit-tests'),(1089,'clippy'),(1090,'build')]:
        phase = f'typed-api-prototype-v{version}-algebraic-preflight-{suffix}'
        r = read(E/phase/'result.json')
        assert r['exit_code'] == 0 and r['source_unchanged'] and r['main_unchanged']
        assert sha(E/phase/'source-hashes.json.gz') == EXPECTED
        passes[phase] = r
    assert '1027 passed; 0 failed;' in (E/'typed-api-prototype-v1088-algebraic-preflight-unit-tests/stdout.log').read_text()
    assert not any(s.startswith('warning:') for s in (E/'typed-api-prototype-v1089-algebraic-preflight-clippy/stderr.log').read_text().splitlines())
    cap_phase = E/'typed-api-prototype-v1087-algebraic-preflight-repair-cap'
    log = (cap_phase/'stdout.log').read_text()
    cap = json.loads(log[log.rfind('{"command":'):])
    assert cap['exit_code'] == 0 and not cap['wall_timeout']
    assert cap['address_space_bytes'] == 192*1024*1024
    assert cap['cpu_seconds_soft_hard'] == [2,3] and cap['wall_seconds_limit'] == 4
    assert '10 passed; 0 failed;' in log
    observations = [s for s in (cap_phase/'stderr.log').read_text().splitlines()
        if s.startswith(('lazy ', 'empty metadata '))]
    allocations = [s for s in observations if 'allocations=' in s]
    assert len(allocations) == 61 and all('calls: 0, bytes: 0, largest: 0' in s for s in allocations)
    cutpoints = [s for s in observations if 'needed=' in s]
    assert len(cutpoints) == 6
    assert cutpoints == baseline['observations']['algebraic_preflight_every_metadata_cutpoint_preserves_retry']
    replays = {}
    for a,b,count in [(1033,1091,384),(1038,1096,384),(1039,1097,288),(1040,1098,144),(1041,1099,384)]:
        old = next(n for n in prior['final_source_replays'] if n.startswith(f'typed-api-prototype-v{a}-'))
        new = old.replace(f'v{a}-structural-comparison',f'v{b}-algebraic-preflight')
        data = evidence.check_artifacts(new)
        assert data['cases'] == count and data['incomplete_cases'] == 0 and data['source_hashes_sha256'] == EXPECTED
        for key in ['invalid_samples','invalid_numeric_samples','typed_errors']: assert data.get(key,0) == 0
        for name in ['probe.rs','reproduce.py']: assert sha(E/old/name) == sha(E/new/name)
        old_rows,new_rows = read(E/old/'receipts.json'),read(E/new/'receipts.json')
        assert [evidence.contract(r) for r in old_rows] == [evidence.contract(r) for r in new_rows]
        strip = lambda rows:[{k:v for k,v in r.items() if k != 'command'} for r in rows]
        assert strip(old_rows) == strip(new_rows), new
        replays[new] = {'cases':count,'summary_sha256':sha(E/new/'summary.json'),
            'probe_harness_inputs_order_and_caps_preserved':True,
            'answers_numeric_observations_and_retained_proofs_identical':True,
            'receipts_identical_except_commands':True,'cooperative_work_count_changes':[]}
    evidence.EXPECTED = EXPECTED
    costs,records = {},[]
    for family,a,b,suffix,checksum in [
        ('inverse-trig',1034,1092,'inverse-cost','certified'),
        ('normal-form',1035,1093,'normal-form-cost','certified'),
        ('default',1036,1094,'default-cost','checksums'),
        ('algebraic',1037,1095,'algebraic-cost','certified'),
    ]:
        name = 'algebraic-preflight-'+family+'-cost-comparison-v1100.json'
        costs[family] = evidence.cost(f'typed-api-prototype-v{a}-structural-comparison-{suffix}',
            f'typed-api-prototype-v{b}-algebraic-preflight-{suffix}',name,checksum)
        records.append(name)
    paired = read(E/'algebraic-preflight-paired-cost-v1100/summary.json')
    assert paired['source_record_sha256'] == EXPECTED and paired['processes'] == 432 and paired['guards_unchanged']
    for name,value in paired['artifacts_sha256'].items(): assert sha(E/'algebraic-preflight-paired-cost-v1100'/name) == value
    confirmation = read(E/'algebraic-preflight-targeted-confirmation-v1100/summary.json')
    assert confirmation['source_record_sha256'] == EXPECTED and confirmation['processes'] == 108 and confirmation['guards_unchanged']
    for name,value in confirmation['artifacts_sha256'].items(): assert sha(E/'algebraic-preflight-targeted-confirmation-v1100'/name) == value
    assert confirmation['provenance']['default']['binary_sha256_at_paired_run'] == paired['provenance']['default']['binary_sha256_at_paired_run']
    write('algebraic-preflight-implementation-v1100.json', {
        'status':'Scoped algebraic lazy-preflight and initial metadata-allocation repair; full checkpoint pending; no runtime admission',
        'previous_checkpoint':'typed-api-prototype-v1074-state.json','baseline_commit':'51878da00',
        'baseline_record':'algebraic-preflight-baseline-v1085-corrected.json',
        'source_record':str(record.relative_to(E)),'source_record_sha256':EXPECTED,'changed_sources':changed,
        'runtime_diff':patch.name,'prior_named_test_bodies_and_assertions_unchanged':True,
        'all_ten_baseline_tests_values_caps_and_assertions_unchanged':True,
        'changes':[
            'Debit raw stored rational size before lazy canonicalization in algebraic metadata and dependency-candidate/base preflight.',
            'Preserve canonical size limits and cheap sign/degree/oversize rejections; wide unreduced values remain supported when their canonical form fits and sufficient work is available.',
            'Retain the paid canonical rational view for subsequent exact dependency arithmetic without duplicating the preflight debit.',
            'Estimate coefficient work from raw stored widths without reducing operands; preserve canonical operand accounting.',
            'Allocate the initial 16-entry metadata vector only after a successfully charged visit produces metadata; preserve its ordinary initial capacity.',
            'Preserve existing exact answers, local search limits, prior certificates and zero-work proof reuse.'
        ],
        'unit_coverage':baseline['coverage'],
        'cap':cap,'observations':observations,'allocation_observations':allocations,
        'measured_work_observations':cutpoints,'allocation_scope':baseline['allocation_scope'],
        'passing_validation':passes,'replays':replays,'cost_records':records,
        'original_trial_percent_changes':costs,'paired_cost':'algebraic-preflight-paired-cost-v1100/summary.json',
        'paired_percent_changes':{k:[round(r['median_percent_change'],2) for r in v] for k,v in paired['cases'].items()},
        'targeted_confirmation':'algebraic-preflight-targeted-confirmation-v1100/summary.json',
        'confirmation_percent_changes':{k:[round(r['median_percent_change'],2) for r in v] for k,v in confirmation['cases'].items()},
        'executable_sizes':{k:v['executable_bytes'] for k,v in paired['provenance'].items()},
        'limits':[
            'Preflight ordering repairs the frozen allocation gaps; cooperative units do not bound individual exact-operation CPU, native allocation, elapsed time or memory.',
            'Allocation evidence counts Rust requests in specific synchronous calls, excluding fixture setup and native allocation; it is not RSS or peak-live memory.',
            'The 27 public retry schedules check exact answers and retained proofs, not an allocation bound for the complete public call graph.',
            'Finite tests and replays do not exhaust concurrent, reconstructed or cross-root warming schedules, or every unshared symbolic expression.',
            'Changed runtime remains outside the conditional Verus target; all thirteen whole-crate proof groups remain open.',
            'Main retains the original scalar failure. All 49 original worktree entries, 39 properties, 14 targets and crate future work retain their scope and status.'
        ]})
    print(json.dumps({'source':EXPECTED,'replay_cases':1584,'original_cost_changes':costs,
        'paired_changes':{k:[round(r['median_percent_change'],2) for r in v] for k,v in paired['cases'].items()}}))


if __name__ == '__main__':
    main()
