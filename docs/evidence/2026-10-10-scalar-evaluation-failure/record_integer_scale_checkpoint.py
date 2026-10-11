#!/usr/bin/env python3
"""Record exact integer scaling validation while retaining the full properties plan."""
import gzip
import hashlib
import json
from pathlib import Path
import re
import sys

sys.dont_write_bytecode = True
from parse_test_results_v3 import parse_phase
from record_evaluation_budget_checkpoint import case_commands, command_contract

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT/'target/scalar-evaluation-candidate/source'
EXPECTED = '32b816463eacd2d2cef2f905c0455557c015117690a8a8dd635229a57726d2b2'
read = lambda p:json.loads(p.read_text())
sha = lambda p:hashlib.sha256(p.read_bytes()).hexdigest()


def write(name,data):
    path = E/name
    assert not path.exists(),path
    path.write_text(json.dumps(data,indent=2)+'\n')


def completed(phase):
    path = E/phase
    result = read(path/'result.json')
    assert result['exit_code'] == 0 and result['source_unchanged'] and result['main_unchanged'],(phase,result)
    assert sha(path/'source-hashes.json.gz') == EXPECTED
    return path


def validations(prior,implementation):
    summaries = {}
    for key,version,suffix in [('debug_scalar',1371,'debug-scalar-unit-tests'),('five_crate',1372,'stack-tests'),('default_scalar',1373,'default-tests')]:
        actual = parse_phase(completed(f'typed-api-prototype-v{version}-{suffix}'))
        record = f'typed-api-prototype-v{version}-test-summary.json'
        assert actual == read(E/record)
        old = prior['test_summaries'][key]
        assert actual['top_level_passed'] == old['passed']+5
        assert actual['child_process_passed'] == old['child_passed'] and actual['top_level_ignored'] == old['ignored']
        assert actual['top_level_failed'] == actual['child_process_failed'] == 0
        summaries[key] = {'record':record,'passed':actual['top_level_passed'],'child_passed':actual['child_process_passed'],'ignored':actual['top_level_ignored']}
    caps = {}
    for version,suffix,count in [(1353, 'integer-scale-repair', 5), (1395, 'inverse-constructor', 11), (1394, 'rational-preflight', 10), (1393, 'algebraic-preflight', 10), (1392, 'structural-comparison', 14), (1391, 'inverse-dag', 10), (1390, 'algebraic-leaf', 6), (1377, 'private-kernel', 7), (1378, 'magnitude', 5), (1379, 'publication', 7), (1380, 'fractional', 5), (1381, 'scaled-lower', 1), (1382, 'trig-range', 5), (1383, 'trig-lattice', 5), (1384, 'trig-residual', 6), (1385, 'trig-operand', 6), (1386, 'evaluation-budget', 7), (1387, 'algebraic-budget', 4), (1388, 'normal-form-budget', 8), (1389, 'inverse-trig-budget', 9)]:
        phase = f'typed-api-prototype-v{version}-{suffix}-cap'
        path = completed(phase)
        log = (path/'stdout.log').read_text()
        receipt = json.loads(log[log.rfind('{"command":'):])
        assert receipt['exit_code'] == 0 and not receipt['wall_timeout']
        assert re.findall(r'^test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;',log,re.M) == ([('1','0','0')] if version == 1391 else [])+[(str(count),'0','0')]
        assert receipt['address_space_bytes'] == 192*1024*1024 and receipt['cpu_seconds_soft_hard'] == [2,3] and receipt['wall_seconds_limit'] == 4
        assert receipt['helper_sha256'] == sha(E/'run_limited_unit_test.py')
        caps[phase] = receipt|{'passed':count,'child_passed':int(version == 1391),'phase_result':read(path/'result.json')}
    assert sum(r['passed'] for r in caps.values()) == 141
    replays = {}
    for version,old_version,suffix,count in [(1396, 1275, 'replay', 415), (1397, 1276, 'precision-replay', 60), (1398, 1277, 'private-kernel-replay', 78), (1399, 1278, 'private-trig-replay', 600), (1400, 1279, 'private-trig-wide-replay', 384), (1401, 1280, 'symbolic-trig-scale-replay', 168), (1402, 1281, 'symbolic-trig-residual-replay', 456), (1403, 1282, 'symbolic-trig-nonrational-replay', 384)]:
        phase = f'typed-api-prototype-v{version}-{suffix}'
        path,old = E/phase,E/f'typed-api-prototype-v{old_version}-{suffix}'
        data = read(path/'summary.json')
        assert data['source_hashes_sha256'] == EXPECTED and data['guards_unchanged']
        for name in ['probe.rs','candidate_probe.rs','cache_candidate_probe.rs','reproduce.py','oracles.json']:
            if (old/name).is_file(): assert sha(old/name) == sha(path/name),(phase,name)
        assert [command_contract(r) for r in case_commands(old)] == [command_contract(r) for r in case_commands(path)]
        if version == 1396:
            assert data['ordinary_cases'] == 400 and data['typed_interruption_errors'] == 300
            assert data['invalid_first_contracts'] == data['invalid_retries'] == 0
            assert len(data['extremes']) == 6 and all(r['valid'] for r in data['extremes'])
            assert len(data['cache']) == 9 and all(r['valid'] for r in data['cache'])
            assert all(r['exit_code'] == 0 for r in data['commands'])
        else:
            assert data['cases'] == count
            if version == 1397:
                assert data['valid_cases'] == 60 and not data['nonzero_exit_cases'] and not data['invalid_numeric_cases']
            elif version == 1398:
                assert data['incomplete_cases'] == data['unexpected_result_shapes'] == 0
            elif version == 1401:
                assert data['valid_samples'] == 672 and data['incomplete_cases'] == data['invalid_samples'] == 0
            else:
                assert data['numeric_samples'] == 4*count
                assert data['incomplete_cases'] == data['invalid_numeric_samples'] == data['typed_errors'] == 0
        replays[phase] = {'cases':count,'summary_sha256':sha(path/'summary.json')}
    for phase,value in implementation['replays'].items():
        data = read(E/phase/'summary.json')
        assert data['source_hashes_sha256'] == EXPECTED and data['guards_unchanged'] and data['cases'] == value['cases']
        assert data['incomplete_cases'] == 0 and value['inputs_harness_order_and_caps_preserved']
        assert sha(E/phase/'summary.json') == value['summary_sha256']
        replays[phase] = {'cases':value['cases'],'summary_sha256':value['summary_sha256']}
    assert sum(r['cases'] for r in replays.values()) == 4129
    return summaries,caps,replays


def main():
    assert len(sys.argv) == 2, 'implementation commit required'
    prior = read(E/'typed-api-prototype-v1283-state.json')
    implementation = read(E/'integer-scale-implementation-v1367.json')
    proof = read(E/'integer-scale-proof-validation-v1404.json')
    retention = read(E/'integer-scale-retention-v1367.json')
    assert implementation['source_record_sha256'] == proof['source_record_sha256'] == retention['source_record_sha256'] == EXPECTED
    assert all(r['receipts_identical_except_commands'] for r in implementation['replays'].values())
    assert proof['verified_units'] == 638 and proof['errors'] == 0
    assert len(proof['proof_and_guard_inputs_sha256']) == 44
    assert proof['fresh_rejected_mutants'] == 213 and proof['fresh_passing_guard_baselines'] == 4
    assert proof['new_rejection_invocations'] == 217 and proof['reused_rejected_mutants'] == 0
    assert proof['fresh_acceptance_tests'] == 10 and proof['open_obligation_groups'] == 13
    source_record = 'typed-api-prototype-v1368-budget-verus/source-hashes.json.gz'
    source = json.loads(gzip.decompress((E/source_record).read_bytes()))
    before = json.loads(gzip.decompress((E/prior['source_record']).read_bytes()))
    assert all(sha(SOURCE/n) == v for n, v in source.items())
    changed = {n: {'before': before.get(n), 'after': v} for n, v in source.items() if before.get(n) != v}
    assert changed == implementation['changed_sources'] and len(changed) == 8
    assert all(before[n] == source[n] for n in before if 'test' in n)
    assert implementation['preexisting_tests_unchanged'] and implementation['all_five_corrected_baseline_tests_and_caps_unchanged']
    summaries, caps, replays = validations(prior, implementation)
    formatted = read(completed('typed-api-prototype-v1376-included-fmt')/'command.json')['command']
    formatted = formatted[formatted.index('--check')+1:]
    original = json.loads(gzip.decompress((E/'candidate-preparation/source-hashes.json.gz').read_bytes()))
    assert formatted == sorted(n for n, v in source.items() if n.endswith('.rs') and original.get(n) != v)
    warnings = lambda p: [r for r in p.read_text().splitlines() if r.startswith('warning:')]
    inherited = warnings(completed('typed-api-prototype-v1372-stack-tests')/'stderr.log')
    assert inherited == warnings(E/'typed-api-prototype-v553-stack-tests/stderr.log') and len(inherited) == 14
    for phase in ['typed-api-prototype-v1355-integer-scale-clippy', 'typed-api-prototype-v1374-minimal-clippy']:
        assert not warnings(completed(phase)/'stderr.log')
    orchestration = read(E/'integer-scale-orchestration-v1404.json')
    assert orchestration['exit_code'] == 0 and orchestration['source_record_sha256'] == EXPECTED
    assert orchestration['log_sha256'] == sha(E/'integer-scale-orchestration-v1404.log')
    limits = implementation['limits'] + [
        'The 78 private-kernel replay cases check result shapes. The 64 coarse-integer allocation observations do not establish whole-call-graph resource bounds.',
        'All 1584 focused replay receipts agree except executable commands, including numerical observations, allowance counts and retained proofs.',
        'Both timing attempts and their separate/paired trials remain archived. The final retention record states their practical limits; no uniform speedup or whole-program admission is claimed.',
        'The retained v3 parser and earlier six-test correction remain authoritative; raw logs are unchanged.',
        'Fresh positive and negative proof checks cover the existing conditional target. The changed rounded-scaling/cache implementation itself remains outside that proof target.',
        'Main runtime remains unadmitted; no push was requested or performed.'
    ]
    previous_review = read(E/'inverse-constructor-resource-followup-v1283.json')
    remaining = previous_review['remaining_work'] + [
        'Coarse integer output no longer requires cloning discarded limbs. Fine/output-sized allocation, negative-shift scans and individual exact-operation CPU/native-memory costs remain separate review targets; EvaluationBudget does not promise bytes or elapsed time.'
    ]
    prefix = 'crates/hyperreal/src/computable/'
    reviewed = {
        'node/scale.rs': 'Complete checked precision, machine precision conversion, owned/borrowed scaling and rounded-shift helpers.',
        'node/representation.rs': 'Cached approximation representation and EvaluationCache methods through value_at_precision; conditional exclusion of EvaluationCache from Verus. No whole-file audit credit.',
        'node/approximation_queries.rs': 'approx_signal initial stop/zero/precision guards, direct integer/one branches, cache selection and owned exact-trig result scaling.',
        'approximation/dispatch.rs': 'Lines 1-105, including shared-constant index and direct approximation stop/precision/integer branches. No whole-file audit credit.',
        'budget.rs': 'Complete cooperative allowance and dynamic-scope contract/implementation.',
        'node/evaluation_budget.rs': 'Complete public approximation/sign wrappers and documented exclusions of per-operation byte/time limits.',
        'node/integer_scale_tests.rs': 'Complete final five tests, measurement windows, independent division/remainder oracle and corrected sign-reuse schedule.',
        'node/cache_rescale_tests.rs': 'All five retained cache rescaling tests: signed rounding, limb/tie boundaries, extreme gaps, public roots and concurrent publication.',
        'node/precision_boundary_tests.rs': 'Integer rounding and extreme precision/zero tests; additional symbolic-offset tests retained unchanged.'
    }
    slices = {prefix+n: {'sha256': source[prefix+n], 'scope': scope} for n, scope in reviewed.items()}
    followup = {'status': 'Coarse integer copy optimization validated in isolation; complete properties plan remains open',
        'previous_review': 'inverse-constructor-resource-followup-v1283.json', 'source_record': source_record,
        'source_record_sha256': EXPECTED, 'reviewed_source_slices': slices, 'closed_scoped_gaps': implementation['changes'],
        'allocation_evidence': implementation['coverage'], 'allocation_scope': implementation['allocation_scope'],
        'classification': 'Avoidable-copy optimization, not an allowance-contract violation. The public allowance contract explicitly excludes per-operation byte/time bounds.',
        'next_candidates': previous_review['next_candidates'], 'remaining_work': remaining, 'limits': limits}
    write('integer-scale-resource-followup-v1404.json', followup)
    write('integer-scale-regression-coverage-v1404.json', {
        'status': 'Isolated coarse integer allocation optimization validated; no runtime admission',
        'source_record_sha256': EXPECTED, 'changed_sources': changed, 'contract_decision': 'p4-contract-decision-v173.json',
        'unit_coverage': implementation['coverage'], 'retained_failure_record': 'integer-scale-baseline-v1298.json',
        'prior_unretained_attempt': 'integer-scale-implementation-v1314.json',
        'allocation_observations': implementation['allocation_observations'], 'allocation_scope': implementation['allocation_scope'],
        'final_source_replays': replays, 'prior_tests_unchanged': True, 'all_five_corrected_baseline_tests_and_caps_unchanged': True,
        'focused_replay_comparison': implementation['replays'], 'cost_records': implementation['cost_records'],
        'paired_cost': implementation['paired_cost'], 'retention_record': 'integer-scale-retention-v1367.json', 'limits': limits})
    write('typed-api-prototype-v1404-metadata.json', {
        'previous_checkpoint': 'typed-api-prototype-v1283-state.json', 'implementation_commits': [sys.argv[1]],
        'cost_baseline_commit': 'a4e9efe58', 'prior_attempt_commit': '47a48785e',
        'implementation_records': ['integer-scale-implementation-v1314.json', 'integer-scale-implementation-v1367.json'],
        'status': 'isolated coarse integer allocation optimization; no runtime admission', 'contract_decision': 'p4-contract-decision-v173.json',
        'changes': implementation['changes'], 'retained_regression_record': 'integer-scale-regression-coverage-v1404.json',
        'retained_failure_record': 'integer-scale-baseline-v1298.json', 'test_summaries': summaries,
        'capped_unit_regressions': caps, 'final_source_replays': replays, 'included_rust_fragments_checked': len(formatted),
        'formal_validation_record': 'integer-scale-proof-validation-v1404.json', 'formal_validation': proof,
        'cost_records': implementation['cost_records'], 'paired_cost': implementation['paired_cost'],
        'retention_record': 'integer-scale-retention-v1367.json',
        'inherited_warnings': {'identical_cargo_filename_warning_lines': inherited, 'strict_clippy_warning_lines': 0},
        'coverage_limits': limits, 'remaining_source_review': 'integer-scale-resource-followup-v1404.json',
        'retained_parser_correction': 'algebraic-preflight-parser-correction-v1133.json',
        'completed_orchestration': 'integer-scale-orchestration-v1404.json', 'remaining_requirements': remaining+limits})
    print(json.dumps({'tests': summaries, 'capped_tests': 141, 'allocation_child_tests': 1,
        'replay_cases': 4129, 'formatted_rust_files': len(formatted), 'changed_sources': len(changed)}))


if __name__ == '__main__':
    main()

