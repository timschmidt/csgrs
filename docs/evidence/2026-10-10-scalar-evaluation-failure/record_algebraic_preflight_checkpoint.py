#!/usr/bin/env python3
"""Record algebraic preflight validation and the remaining admission work."""
import gzip
import hashlib
import json
from pathlib import Path
import re
import sys

sys.dont_write_bytecode = True
from parse_test_results_v2 import parse_phase
from record_evaluation_budget_checkpoint import case_commands, command_contract

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT/'target/scalar-evaluation-candidate/source'
EXPECTED = '824a7ecd2822e2f06c5ee58d6090ce60aa8e69098d5774ecc0c7113f50caa54a'
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def write(name,data):
    path = E/name
    assert not path.exists(), path
    path.write_text(json.dumps(data,indent=2)+'\n')


def completed(phase):
    path = E/phase
    result = read(path/'result.json')
    assert result['exit_code'] == 0 and result['source_unchanged'] and result['main_unchanged'], (phase,result)
    assert sha(path/'source-hashes.json.gz') == EXPECTED
    return path


def main():
    prior = read(E/'typed-api-prototype-v1074-state.json')
    implementation = read(E/'algebraic-preflight-implementation-v1100.json')
    fast = implementation
    assert fast['source_record_sha256'] == EXPECTED
    proof = read(E/'algebraic-preflight-proof-reuse-v1133.json')
    assert proof['source_record_sha256'] == EXPECTED and proof['verified_units'] == 638 and proof['errors'] == 0
    assert len(proof['proof_and_guard_inputs_sha256']) == 44 and proof['reused_rejected_mutants'] == 213
    assert proof['fresh_acceptance_tests'] == 10 and proof['open_obligation_groups'] == 13
    source_record = 'typed-api-prototype-v1101-budget-verus/source-hashes.json.gz'
    source = json.loads(gzip.decompress((E/source_record).read_bytes()))
    before = json.loads(gzip.decompress((E/prior['source_record']).read_bytes()))
    assert all(sha(SOURCE/n) == v for n,v in source.items())
    changed = {n:{'before':before.get(n),'after':v} for n,v in source.items() if before.get(n) != v}
    assert set(changed) == set(implementation['changed_sources']) and len(changed) == 3
    assert all(before[n] == source[n] for n in before if 'test' in n)
    assert implementation['prior_named_test_bodies_and_assertions_unchanged']
    summaries = {}
    for key,version,suffix in [('debug_scalar',1103,'debug-scalar-unit-tests'),('five_crate',1104,'stack-tests'),('default_scalar',1105,'default-tests')]:
        actual = parse_phase(completed(f'typed-api-prototype-v{version}-{suffix}'))
        record = f'typed-api-prototype-v{version}-test-summary.json'
        assert actual == read(E/record)
        old = prior['test_summaries'][key]
        assert actual['top_level_passed'] == old['passed'] + 10
        assert actual['child_process_passed'] == old['child_passed'] and actual['top_level_ignored'] == old['ignored']
        assert actual['top_level_failed'] == actual['child_process_failed'] == 0
        summaries[key] = {'record':record,'passed':actual['top_level_passed'],'child_passed':actual['child_process_passed'],'ignored':actual['top_level_ignored']}
    caps = {}
    for version,suffix,count in [(1087,'algebraic-preflight-repair',10),(1124,'structural-comparison',14),(1123,'inverse-dag',10),(1122,'algebraic-leaf',6),(1109,'private-kernel',7),(1110,'magnitude',5),(1111,'publication',7),(1112,'fractional',5),(1113,'scaled-lower',1),(1114,'trig-range',5),(1115,'trig-lattice',5),(1116,'trig-residual',6),(1117,'trig-operand',6),(1118,'evaluation-budget',7),(1119,'algebraic-budget',4),(1120,'normal-form-budget',8),(1121,'inverse-trig-budget',9)]:
        phase = f'typed-api-prototype-v{version}-{suffix}-cap'
        path = completed(phase)
        log = (path/'stdout.log').read_text()
        receipt = json.loads(log[log.rfind('{"command":'):])
        assert receipt['exit_code'] == 0 and not receipt['wall_timeout']
        assert re.findall(r'^test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;',log,re.M) == ([('1','0','0')] if version == 1123 else []) + [(str(count),'0','0')]
        assert receipt['address_space_bytes'] == 192*1024*1024 and receipt['cpu_seconds_soft_hard'] == [2,3] and receipt['wall_seconds_limit'] == 4
        caps[phase] = receipt | {'passed':count,'child_passed':int(version == 1123),'phase_result':read(path/'result.json')}
    assert sum(r['passed'] for r in caps.values()) == 115
    replays = {}
    for version,old_version,suffix,count in [(1125,1066,'replay',415),(1126,1067,'precision-replay',60),(1127,1068,'private-kernel-replay',78),(1128,1069,'private-trig-replay',600),(1129,1070,'private-trig-wide-replay',384),(1130,1071,'symbolic-trig-scale-replay',168),(1131,1072,'symbolic-trig-residual-replay',456),(1132,1073,'symbolic-trig-nonrational-replay',384)]:
        phase = f'typed-api-prototype-v{version}-{suffix}'
        path,old = E/phase,E/f'typed-api-prototype-v{old_version}-{suffix}'
        data = read(path/'summary.json')
        assert data['source_hashes_sha256'] == EXPECTED and data['guards_unchanged']
        for name in ['probe.rs','candidate_probe.rs','cache_candidate_probe.rs','reproduce.py','oracles.json']:
            if (old/name).is_file(): assert sha(old/name) == sha(path/name), (phase,name)
        assert [command_contract(r) for r in case_commands(old)] == [command_contract(r) for r in case_commands(path)]
        if version == 1125:
            assert data['ordinary_cases'] == 400 and data['typed_interruption_errors'] == 300
            assert data['invalid_first_contracts'] == data['invalid_retries'] == 0
            assert len(data['extremes']) == 6 and all(r['valid'] for r in data['extremes'])
            assert len(data['cache']) == 9 and all(r['valid'] for r in data['cache'])
            assert all(r['exit_code'] == 0 for r in data['commands'])
        else:
            assert data['cases'] == count
            if version == 1126:
                assert data['valid_cases'] == 60 and not data['nonzero_exit_cases'] and not data['invalid_numeric_cases']
            elif version == 1127:
                assert data['incomplete_cases'] == data['unexpected_result_shapes'] == 0
            elif version == 1130:
                assert data['valid_samples'] == 672 and data['incomplete_cases'] == data['invalid_samples'] == 0
            else:
                assert data['numeric_samples'] == 4*count
                assert data['incomplete_cases'] == data['invalid_numeric_samples'] == data['typed_errors'] == 0
        replays[phase] = {'cases':count,'summary_sha256':sha(path/'summary.json')}
    for phase,value in fast['replays'].items():
        data = read(E/phase/'summary.json')
        assert data['source_hashes_sha256'] == EXPECTED and data['guards_unchanged'] and data['cases'] == value['cases']
        assert data['incomplete_cases'] == 0 and value['probe_harness_inputs_order_and_caps_preserved']
        assert sha(E/phase/'summary.json') == value['summary_sha256']
        replays[phase] = {'cases':value['cases'],'summary_sha256':value['summary_sha256']}
    assert sum(r['cases'] for r in replays.values()) == 4129
    formatted = read(completed('typed-api-prototype-v1108-included-fmt')/'command.json')['command']
    formatted = formatted[formatted.index('--check')+1:]
    original = json.loads(gzip.decompress((E/'candidate-preparation/source-hashes.json.gz').read_bytes()))
    assert formatted == sorted(n for n,v in source.items() if n.endswith('.rs') and original.get(n) != v)
    warnings = lambda p:[r for r in p.read_text().splitlines() if r.startswith('warning:')]
    inherited = warnings(completed('typed-api-prototype-v1104-stack-tests')/'stderr.log')
    assert inherited == warnings(E/'typed-api-prototype-v553-stack-tests/stderr.log') and len(inherited) == 14
    for phase in ['typed-api-prototype-v1089-algebraic-preflight-clippy','typed-api-prototype-v1106-minimal-clippy']:
        assert not warnings(completed(phase)/'stderr.log')
    limits = implementation['limits'] + [
        'The 78 private-kernel replay cases validate result shapes. Ten new algebraic preflight tests do not establish bounded resources for every symbolic operation.',
        'Local timings and standalone probe sizes are scoped observations. Whole-application performance and runtime admission remain open.',
        'All 1584 focused replay receipts are identical except executable commands. Answers, numerical observations, certificates, work counts, inputs, allowances and resource caps are preserved.',
        'Main runtime remains unadmitted; no push was requested or performed.'
    ]
    previous_review = read(E/'structural-comparison-resource-followup-v1074.json')
    reviewed = {
        'crates/hyperreal/src/computable/node/structural_analysis.rs':
            'Complete AlgebraicSeparation implementation, metadata parser and algebraic_zero_precision, plus the intervening binary64 interval helpers. No whole-file audit.',
        'crates/hyperreal/src/computable/node/symbolic_work.rs':
            'Raw width accounting, integer and rational read/debit helpers through the first 95 lines. No new whole-file audit.',
        'crates/hyperreal/src/computable/node/approximation_queries.rs':
            'zero_status, complete sign_until and adjacent cache dispatch, lines 410-580. No whole-file audit.',
        'crates/hyperreal/src/computable/node/algebraic_preflight_tests.rs':
            'Complete ten regression/preservation tests and fixture helpers.',
        'crates/hyperreal/src/computable/node/algebraic_leaf_tests.rs':
            'Complete six existing tests and the thread-local Rust allocation counter.',
        'crates/hyperreal/src/computable/node/algebraic_budget_tests.rs':
            'Complete four existing algebraic budget tests.'
    }
    next_candidates = previous_review['next_candidates']
    metadata_candidate = next(r for r in next_candidates if r['path'].endswith('/structural_analysis.rs'))
    metadata_candidate['finding'] = (
        'The v1133 candidate now charges raw storage before lazy metadata/dependency reduction, '
        'estimates coefficient work without canonicalization, and defers the initial metadata vector '
        'until successful charged metadata. Subsequent bounded memo copies/vector growth and '
        'CPU/native allocation within exact operations remain outside these allocation assertions.'
    )
    remaining = previous_review['remaining_work']
    remaining[1] = (
        'Review unshared symbolic form growth, construction-time structural comparison, subsequent '
        'algebraic metadata copies/vector growth and individual exact-operation resource use with '
        'unchanged adversarial fixtures. Initial zero-work vector allocation and lazy preflight '
        'reduction are repaired only in the retained scoped cases.'
    )
    followup = {
        'status':'Scoped algebraic preflight repair; remaining admission work stays open',
        'previous_review':'structural-comparison-resource-followup-v1074.json',
        'source_record':source_record,'source_record_sha256':EXPECTED,
        'previous_partial_call_sites_review':'structural-comparison-call-sites-v1074.json',
        'reviewed_source_slices':{n:{'sha256':source[n],'scope':scope} for n,scope in reviewed.items()},
        'closed_scoped_gaps':implementation['changes'],
        'public_reachability_evidence':(
            'sign_until invokes algebraic_zero_precision after ordinary refinement reaches its '
            'metadata threshold; the helper calls the algebraic metadata parser. The 27 public '
            'root-product retry schedules preserve exact answers and completed zero proofs. '
            'Those schedules do not establish that the raw lazy allocation gap survives every '
            'earlier public-call refinement step; the allocation measurements directly invoke '
            'the named private helpers.'),
        'next_candidates':next_candidates,'remaining_work':remaining,'limits':limits
    }
    write('algebraic-preflight-resource-followup-v1133.json',followup)
    write('algebraic-preflight-regression-coverage-v1133.json',{
        'status':'Isolated algebraic preflight repair validated; no runtime admission',
        'source_record_sha256':EXPECTED,'changed_sources':changed,
        'contract_decision':'p4-contract-decision-v173.json','unit_coverage':implementation['unit_coverage'],
        'retained_failure_record':'algebraic-preflight-baseline-v1085-corrected.json',
        'allocation_observations':implementation['allocation_observations'],
        'measured_work_observations':implementation['measured_work_observations'],
        'allocation_scope':implementation['allocation_scope'],
        'final_source_replays':replays,'prior_named_test_bodies_and_assertions_unchanged':True,
        'all_ten_baseline_tests_values_caps_and_assertions_unchanged':True,
        'all_preserved_probe_inputs_caps_and_arguments_unchanged':True,
        'focused_replay_receipts_identical_except_commands':True,
        'cost_records':implementation['cost_records'],'paired_cost':implementation['paired_cost'],'limits':limits})
    assert len(sys.argv) == 2, 'implementation commit required'
    write('typed-api-prototype-v1133-metadata.json',{
        'previous_checkpoint':'typed-api-prototype-v1074-state.json','implementation_commits':[sys.argv[1]],
        'cost_baseline_commit':'51878da00','implementation_records':['algebraic-preflight-implementation-v1100.json'],
        'status':'isolated algebraic lazy-preflight and initial metadata-allocation repair; no runtime admission',
        'contract_decision':'p4-contract-decision-v173.json','changes':followup['closed_scoped_gaps'],
        'retained_regression_record':'algebraic-preflight-regression-coverage-v1133.json',
        'retained_failure_record':'algebraic-preflight-baseline-v1085-corrected.json',
        'test_summaries':summaries,'capped_unit_regressions':caps,'final_source_replays':replays,
        'included_rust_fragments_checked':len(formatted),'formal_reuse_record':'algebraic-preflight-proof-reuse-v1133.json',
        'formal_validation':proof,'cost_records':implementation['cost_records'],'paired_cost':implementation['paired_cost'],
        'targeted_confirmation':implementation['targeted_confirmation'],
        'retention_record':'algebraic-preflight-retention-v1100.json',
        'inherited_warnings':{'identical_cargo_filename_warning_lines':inherited,'strict_clippy_warning_lines':0},
        'coverage_limits':limits,'remaining_source_review':'algebraic-preflight-resource-followup-v1133.json',
        'remaining_requirements':followup['remaining_work']+limits})
    print(json.dumps({'tests':summaries,'capped_tests':115,'allocation_child_tests':1,
        'replay_cases':4129,'formatted_rust_files':len(formatted),'changed_sources':len(changed)}))


if __name__ == '__main__':
    main()
