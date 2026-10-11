#!/usr/bin/env python3
"""Bind borrowed integer scaling to unchanged regressions and measured costs."""
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
EXPECTED = '66f8a024a12dd9556c219510df679a988e24c7a8ff4c22f23e59a1cae87ddba0'
read, sha, write = evidence.read, evidence.sha, evidence.write


def allocation_rows(lines):
    rows = {}
    for line in lines:
        if not line.startswith('integer scale allocation '):
            continue
        fixture, detail = line.split(', allocations=')
        match = re.fullmatch(r'Allocations \{ calls: (\d+), bytes: (\d+), largest: (\d+) \}', detail)
        assert match and fixture not in rows
        rows[fixture] = dict(zip(['calls', 'bytes', 'largest'], map(int, match.groups())))
    return rows


def main():
    prior = read(E/'typed-api-prototype-v1283-state.json')
    baseline = read(E/'integer-scale-baseline-v1298.json')
    record = E/'typed-api-prototype-v1303-integer-scale-build/source-hashes.json.gz'
    assert sha(record) == EXPECTED
    source = json.loads(gzip.decompress(record.read_bytes()))
    before = json.loads(gzip.decompress((E/prior['source_record']).read_bytes()))
    initial_record = E/baseline['source_record']
    initial = json.loads(gzip.decompress(initial_record.read_bytes()))
    assert all(sha(SOURCE/n) == v for n, v in source.items())
    prefix = 'crates/hyperreal/src/computable/'
    runtime = {prefix+n for n in ['mod.rs', 'approximation.rs', 'approximation/dispatch.rs',
        'node/scale.rs', 'node/representation.rs', 'node/approximation_queries.rs']}
    assert {n for n, v in source.items() if initial.get(n) != v} == runtime
    assert all(source[n] == v for n, v in before.items() if 'test' in n)
    assert source[prefix+'node/integer_scale_tests.rs'] == initial[prefix+'node/integer_scale_tests.rs']
    changed = {n: {'before': before.get(n), 'after': v} for n, v in source.items() if before.get(n) != v}
    assert len(changed) == 8
    patch = ''
    for name in sorted(runtime):
        archived = initial_record.parent/'inputs'/name
        original = archived if archived.is_file() else ROOT/name
        assert sha(original) == initial[name]
        patch += ''.join(difflib.unified_diff(original.read_text().splitlines(True),
            (SOURCE/name).read_text().splitlines(True), fromfile='before/'+name, tofile='after/'+name))
    passes = {}
    for version, suffix in [(1299, 'repair-build'), (1300, 'repair-cap'), (1301, 'unit-tests'), (1302, 'clippy'), (1303, 'build')]:
        phase = f'typed-api-prototype-v{version}-integer-scale-{suffix}'
        result = read(E/phase/'result.json')
        assert result['exit_code'] == 0 and result['source_unchanged'] and result['main_unchanged']
        assert sha(E/phase/'source-hashes.json.gz') == EXPECTED
        passes[phase] = result
    units = parse_phase(E/'typed-api-prototype-v1301-integer-scale-unit-tests')
    assert units['top_level_passed'] == 1054 and units['child_process_passed'] == 1
    assert units['top_level_failed'] == units['child_process_failed'] == 0
    assert not any(s.startswith('warning:') for s in (E/'typed-api-prototype-v1302-integer-scale-clippy/stderr.log').read_text().splitlines())
    cap_phase = E/'typed-api-prototype-v1300-integer-scale-repair-cap'
    log = (cap_phase/'stdout.log').read_text()
    cap = json.loads(log[log.rfind('{"command":'):])
    assert cap['exit_code'] == 0 and not cap['wall_timeout']
    assert cap['address_space_bytes'] == 192*1024*1024
    assert cap['cpu_seconds_soft_hard'] == [2, 3] and cap['wall_seconds_limit'] == 4
    assert cap['helper_sha256'] == sha(E/'run_limited_unit_test.py')
    assert '5 passed; 0 failed;' in log
    old = allocation_rows(baseline['allocation_observations'])
    new = allocation_rows((cap_phase/'stderr.log').read_text().splitlines())
    assert len(old) == len(new) == 64 and old.keys() == new.keys()
    measured = {fixture: {'baseline': old[fixture], 'candidate': new[fixture]} for fixture in old}
    groups = {}
    for fixture, row in measured.items():
        assert row['candidate']['bytes'] <= row['baseline']['bytes']
        assert row['candidate']['bytes'] <= 56 and row['candidate']['largest'] <= 40
        raw, bits, negative, precision = re.search(r'raw=(\w+), bits=(\d+), negative=(\w+), precision=(\d+)', fixture).groups()
        delta = 'max' if int(precision) == 2**31-1 else int(precision)-int(bits)
        groups.setdefault((raw, negative, delta), []).append(row['candidate'])
    assert len(groups) == 16 and all(len(v) == 4 and all(r == v[0] for r in v) for v in groups.values())
    replays = {}
    for a, b, suffix, count in [(1239, 1309, 'inverse', 384), (1244, 1310, 'evaluation', 384),
        (1245, 1311, 'algebraic', 288), (1246, 1312, 'wide', 144), (1247, 1313, 'normal-form', 384)]:
        old_phase = f'typed-api-prototype-v{a}-inverse-constructor-{suffix}-replay'
        new_phase = f'typed-api-prototype-v{b}-integer-scale-{suffix}-replay'
        data = evidence.check_artifacts(new_phase)
        assert data['cases'] == count and data['incomplete_cases'] == 0 and data['source_hashes_sha256'] == EXPECTED
        for key in ['invalid_samples', 'invalid_numeric_samples', 'typed_errors']:
            assert data.get(key, 0) == 0
        for name in ['probe.rs', 'reproduce.py']:
            assert sha(E/old_phase/name) == sha(E/new_phase/name)
        old_rows, new_rows = read(E/old_phase/'receipts.json'), read(E/new_phase/'receipts.json')
        assert [evidence.contract(r) for r in old_rows] == [evidence.contract(r) for r in new_rows]
        strip = lambda rows: [{k: v for k, v in r.items() if k != 'command'} for r in rows]
        assert strip(old_rows) == strip(new_rows)
        replays[new_phase] = {'cases': count, 'summary_sha256': sha(E/new_phase/'summary.json'),
            'inputs_harness_order_and_caps_preserved': True, 'receipts_identical_except_commands': True,
            'answers_numerics_work_counts_and_retained_proofs_identical': True}
    paired = read(E/'integer-scale-paired-cost-v1314/summary.json')
    assert paired['source_record_sha256'] == EXPECTED and paired['processes'] == 576 and paired['guards_unchanged']
    for name, value in paired['artifacts_sha256'].items():
        assert sha(E/'integer-scale-paired-cost-v1314'/name) == value
    evidence.EXPECTED = EXPECTED
    costs, records = {}, []
    for family, a, b, suffix, checksum in [
        ('inverse-trig', 1240, 1305, 'inverse-cost', 'certified'), ('normal-form', 1241, 1306, 'normal-form-cost', 'certified'),
        ('default', 1242, 1307, 'default-cost', 'checksums'), ('algebraic', 1243, 1308, 'algebraic-cost', 'certified')]:
        name = 'integer-scale-'+family+'-cost-comparison-v1314.json'
        costs[family] = evidence.cost(f'typed-api-prototype-v{a}-inverse-constructor-{suffix}',
            f'typed-api-prototype-v{b}-integer-scale-{suffix}', name, checksum)
        records.append(name)
    a, b = 'typed-api-prototype-v1298-integer-scale-baseline-cost', 'typed-api-prototype-v1304-integer-scale-cost'
    old_cost, new_cost = evidence.check_artifacts(a), evidence.check_artifacts(b)
    assert old_cost['processes'] == new_cost['processes'] == 72
    for name in ['probe.rs', 'reproduce.py']:
        assert sha(E/a/name) == sha(E/b/name)
    cases = {}
    for name, old_case in old_cost['cases'].items():
        new_case = new_cost['cases'][name]
        assert old_case['iterations'] == new_case['iterations'] and old_case['checksums'] == new_case['checksums']
        assert len(old_case['elapsed_ns']) == len(new_case['elapsed_ns']) == 9
        cases[name] = {'before': old_case, 'after': new_case, 'median_ratio': new_case['median_ns']/old_case['median_ns']}
    name = 'integer-scale-targeted-cost-comparison-v1314.json'
    write(name, {'baseline_phase': a, 'candidate_phase': b, 'cases': cases,
        'source_record_sha256': EXPECTED, 'probe_and_harness_identical': True,
        'scope': 'Eight local integer-scaling micro-workloads; bit/sign checksum is not an independent numeric oracle. Exact-value tests are recorded separately.'})
    records.append(name)
    costs['integer'] = {k: round(100*(v['median_ratio']-1), 2) for k, v in cases.items()}
    patch_name = 'integer-scale-runtime-diff-v1314.patch'
    assert not (E/patch_name).exists()
    (E/patch_name).write_text(patch)
    write('integer-scale-implementation-v1314.json', {
        'status': 'Allocation-correct first attempt; ordinary-cost regressions require revision; not retained',
        'retention_decision': 'Revise the extra fine/identity helper call before retaining. Paired small_identity, atan_wide and pi_wide medians increased 9.65%, 13.46% and 14.28%; no broad checkpoint was run for this attempt.',
        'previous_checkpoint': 'typed-api-prototype-v1283-state.json', 'baseline_commit': 'a4e9efe58',
        'baseline_record': 'integer-scale-baseline-v1298.json', 'source_record': str(record.relative_to(E)),
        'source_record_sha256': EXPECTED, 'changed_sources': changed, 'runtime_diff': patch_name,
        'preexisting_tests_unchanged': True, 'all_five_corrected_baseline_tests_and_caps_unchanged': True,
        'changes': [
            'Share the existing cache rounded-right-shift algorithm with borrowed integer-leaf scaling in public and direct-dispatch approximation.',
            'Return exact zero before a borrowed bigint shift can clone every discarded limb; preserve signed halfway rounding toward positive infinity.',
            'Retain owned scaling for already-owned exact results and the existing fine-precision path.',
            'Preserve precision and cancellation precedence, cooperative work charges, and proof/cache publication behavior. No new public API or budget promise.'
        ],
        'cap': cap, 'allocation_observations': measured,
        'coverage': {'allocation_cases': 64, 'widths': [65, 4097, 65537, 1000001],
            'routes': ['public one-unit allowance wrapper', 'direct internal dispatcher'],
            'signs': ['positive', 'negative'], 'precision_offsets_from_bit_width': [-1, 0, 1, 'i32::MAX'],
            'independent_rounding_cases_per_route': 1568, 'allowance_retry_schedules': 60,
            'allowances_per_schedule': [0, 1, 3], 'largest_supported_fractional_precision': -1048576},
        'allocation_scope': 'Synchronous thread-local Rust allocation requests excluding fixture and exact-oracle creation, native allocation, RSS and peak-live memory. EvaluationBudget excludes per-operation byte/time bounds.',
        'passing_validation': passes, 'unit_summary': units, 'replays': replays, 'cost_records': records,
        'original_trial_percent_changes': costs, 'paired_cost': 'integer-scale-paired-cost-v1314/summary.json',
        'paired_percent_changes': {k: [round(r['median_percent_change'], 2) for r in v] for k, v in paired['cases'].items()},
        'executable_sizes': {k: v['executable_bytes'] for k, v in paired['provenance'].items()},
        'limits': [
            'Avoidable integer copying is repaired for the measured coarse-output paths; this is not a new byte/time bound or proof of an EvaluationBudget contract violation.',
            'Negative shifts can still scan discarded bits. Output-sized fine scaling, native allocations and broader exact-operation costs remain outside the allocation result.',
            'The initial new sign test incorrectly required a cold fact to be retained; the correction was frozen before runtime changes. All earlier tests and failures remain unchanged.',
            'The shared cache implementation is excluded from the conditional Verus target. The containing proof-input file changed, so fresh negative guard checks are required for this checkpoint; whole-crate proof remains open.',
            'Main retains the original scalar failure. All 49 original worktree entries, 39 properties, 14 targets and crate future work remain in scope.'
        ]
    })
    print(json.dumps({'record': 'integer-scale-implementation-v1314.json', 'allocations': len(measured),
        'replay_cases': sum(r['cases'] for r in replays.values()), 'separate_cost_percent_changes': costs,
        'paired_percent_changes': {k: [round(r['median_percent_change'], 2) for r in v] for k, v in paired['cases'].items()}}))


if __name__ == '__main__':
    main()
