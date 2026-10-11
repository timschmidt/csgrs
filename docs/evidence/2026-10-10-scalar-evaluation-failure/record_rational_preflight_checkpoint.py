#!/usr/bin/env python3
"""Record rational/domain preflight validation and remaining admission work."""
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
EXPECTED = '2f5184c00b655901e7a89b44ccb63824a7aad7ce3790a11f7e82e7ca5da5192f'
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
    for key,version,suffix in [('debug_scalar',1164,'debug-scalar-unit-tests'),('five_crate',1165,'stack-tests'),('default_scalar',1166,'default-tests')]:
        actual = parse_phase(completed(f'typed-api-prototype-v{version}-{suffix}'))
        record = f'typed-api-prototype-v{version}-test-summary.json'
        assert actual == read(E/record)
        old = prior['test_summaries'][key]
        assert actual['top_level_passed'] == old['passed']+10
        assert actual['child_process_passed'] == old['child_passed'] and actual['top_level_ignored'] == old['ignored']
        assert actual['top_level_failed'] == actual['child_process_failed'] == 0
        summaries[key] = {'record':record,'passed':actual['top_level_passed'],'child_passed':actual['child_process_passed'],'ignored':actual['top_level_ignored']}
    caps = {}
    for version,suffix,count in [(1148,'rational-preflight-repair',10),(1186,'algebraic-preflight',10),(1185,'structural-comparison',14),(1184,'inverse-dag',10),(1183,'algebraic-leaf',6),(1170,'private-kernel',7),(1171,'magnitude',5),(1172,'publication',7),(1173,'fractional',5),(1174,'scaled-lower',1),(1175,'trig-range',5),(1176,'trig-lattice',5),(1177,'trig-residual',6),(1178,'trig-operand',6),(1179,'evaluation-budget',7),(1180,'algebraic-budget',4),(1181,'normal-form-budget',8),(1182,'inverse-trig-budget',9)]:
        phase = f'typed-api-prototype-v{version}-{suffix}-cap'
        path = completed(phase)
        log = (path/'stdout.log').read_text()
        receipt = json.loads(log[log.rfind('{"command":'):])
        assert receipt['exit_code'] == 0 and not receipt['wall_timeout']
        assert re.findall(r'^test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;',log,re.M) == ([('1','0','0')] if version == 1184 else [])+[(str(count),'0','0')]
        assert receipt['address_space_bytes'] == 192*1024*1024 and receipt['cpu_seconds_soft_hard'] == [2,3] and receipt['wall_seconds_limit'] == 4
        assert receipt['helper_sha256'] == sha(E/'run_limited_unit_test.py')
        caps[phase] = receipt|{'passed':count,'child_passed':int(version == 1184),'phase_result':read(path/'result.json')}
    assert sum(r['passed'] for r in caps.values()) == 125
    replays = {}
    for version,old_version,suffix,count in [(1187,1125,'replay',415),(1188,1126,'precision-replay',60),(1189,1127,'private-kernel-replay',78),(1190,1128,'private-trig-replay',600),(1191,1129,'private-trig-wide-replay',384),(1192,1130,'symbolic-trig-scale-replay',168),(1193,1131,'symbolic-trig-residual-replay',456),(1194,1132,'symbolic-trig-nonrational-replay',384)]:
        phase = f'typed-api-prototype-v{version}-{suffix}'
        path,old = E/phase,E/f'typed-api-prototype-v{old_version}-{suffix}'
        data = read(path/'summary.json')
        assert data['source_hashes_sha256'] == EXPECTED and data['guards_unchanged']
        for name in ['probe.rs','candidate_probe.rs','cache_candidate_probe.rs','reproduce.py','oracles.json']:
            if (old/name).is_file(): assert sha(old/name) == sha(path/name),(phase,name)
        assert [command_contract(r) for r in case_commands(old)] == [command_contract(r) for r in case_commands(path)]
        if version == 1187:
            assert data['ordinary_cases'] == 400 and data['typed_interruption_errors'] == 300
            assert data['invalid_first_contracts'] == data['invalid_retries'] == 0
            assert len(data['extremes']) == 6 and all(r['valid'] for r in data['extremes'])
            assert len(data['cache']) == 9 and all(r['valid'] for r in data['cache'])
            assert all(r['exit_code'] == 0 for r in data['commands'])
        else:
            assert data['cases'] == count
            if version == 1188:
                assert data['valid_cases'] == 60 and not data['nonzero_exit_cases'] and not data['invalid_numeric_cases']
            elif version == 1189:
                assert data['incomplete_cases'] == data['unexpected_result_shapes'] == 0
            elif version == 1192:
                assert data['valid_samples'] == 672 and data['incomplete_cases'] == data['invalid_samples'] == 0
            else:
                assert data['numeric_samples'] == 4*count
                assert data['incomplete_cases'] == data['invalid_numeric_samples'] == data['typed_errors'] == 0
        replays[phase] = {'cases':count,'summary_sha256':sha(path/'summary.json')}
    for phase,value in implementation['replays'].items():
        data = read(E/phase/'summary.json')
        assert data['source_hashes_sha256'] == EXPECTED and data['guards_unchanged'] and data['cases'] == value['cases']
        assert data['incomplete_cases'] == 0 and value['probe_harness_inputs_order_and_caps_preserved']
        assert sha(E/phase/'summary.json') == value['summary_sha256']
        replays[phase] = {'cases':value['cases'],'summary_sha256':value['summary_sha256']}
    assert sum(r['cases'] for r in replays.values()) == 4129
    return summaries,caps,replays


def main():
    prior = read(E/'typed-api-prototype-v1133-state.json')
    implementation = read(E/'rational-preflight-implementation-v1161.json')
    assert implementation['source_record_sha256'] == EXPECTED
    proof = read(E/'rational-preflight-proof-reuse-v1195.json')
    assert proof['source_record_sha256'] == EXPECTED and proof['verified_units'] == 638 and proof['errors'] == 0
    assert len(proof['proof_and_guard_inputs_sha256']) == 44 and proof['reused_rejected_mutants'] == 213
    assert proof['fresh_acceptance_tests'] == 10 and proof['open_obligation_groups'] == 13
    source_record = 'typed-api-prototype-v1162-budget-verus/source-hashes.json.gz'
    source = json.loads(gzip.decompress((E/source_record).read_bytes()))
    before = json.loads(gzip.decompress((E/prior['source_record']).read_bytes()))
    assert all(sha(SOURCE/n) == v for n,v in source.items())
    changed = {n:{'before':before.get(n),'after':v} for n,v in source.items() if before.get(n) != v}
    assert changed == implementation['changed_sources'] and len(changed) == 6
    assert all(before[n] == source[n] for n in before if 'test' in n)
    assert implementation['prior_named_test_bodies_and_assertions_unchanged']
    summaries,caps,replays = validations(prior,implementation)
    formatted = read(completed('typed-api-prototype-v1169-included-fmt')/'command.json')['command']
    formatted = formatted[formatted.index('--check')+1:]
    original = json.loads(gzip.decompress((E/'candidate-preparation/source-hashes.json.gz').read_bytes()))
    assert formatted == sorted(n for n,v in source.items() if n.endswith('.rs') and original.get(n) != v)
    warnings = lambda p:[r for r in p.read_text().splitlines() if r.startswith('warning:')]
    inherited = warnings(completed('typed-api-prototype-v1165-stack-tests')/'stderr.log')
    assert inherited == warnings(E/'typed-api-prototype-v553-stack-tests/stderr.log') and len(inherited) == 14
    for phase in ['typed-api-prototype-v1150-rational-preflight-clippy','typed-api-prototype-v1167-minimal-clippy']:
        assert not warnings(completed(phase)/'stderr.log')
    orchestration = read(E/'rational-preflight-orchestration-v1195.json')
    assert orchestration['exit_code'] == 0 and orchestration['source_record_sha256'] == EXPECTED
    assert orchestration['log_sha256'] == sha(E/'rational-preflight-orchestration-v1195.log')
    limits = implementation['limits']+[
        'The 78 private-kernel replay cases validate result shapes. Ten rational/domain preflight tests do not establish bounded resources for every symbolic operation.',
        'Local timings and standalone probe sizes are scoped observations. Whole-application performance and runtime admission remain open.',
        'All 1584 focused replay answers, numerical observations and retained proofs are identical. Eight inverse-trig cases consume six additional units for domain comparison; their remaining allowance decreases by six. Other focused receipts match except executable commands.',
        'The evidence parser is the retained v3 parser with its v1133 six-test correction record; raw Cargo logs remain unchanged.',
        'Main runtime remains unadmitted; no push was requested or performed.'
    ]
    previous_review = read(E/'algebraic-preflight-resource-followup-v1133.json')
    next_candidates = previous_review['next_candidates']
    next_candidates.extend([
        {'path':'crates/hyperreal/src/computable/node/algebra.rs',
         'operations':['inverse-trig argument differences through general constructors'],
         'finding':'The retained v1133 supplemental review traces sign_until_with_budget through exact_sign(Add) and metered inverse-trig recognition to Computable::negate/add. Freeze public fixtures with cheap argument inequality before measuring wide constructor payload copies. This remains a static path, not a measured public resource failure.'},
        {'path':'crates/hyperreal/src/computable/node/algebra.rs',
         'operations':['PiLaurentPolynomial::clone_with_work'],
         'finding':'The retained supplemental review identifies an at-most-eight-term vector allocation after a visit but before coefficient preflight. Measure just-insufficient allowances and successful copies before deciding on a repair; no unbounded allocation or public failure is established.'}
    ])
    remaining = [
        'Measure the retained inverse-trig constructor path with frozen public sign fixtures, including differing integer widths, exact answers and warming schedules.',
        'Measure bounded Laurent coefficient-copy allocation ordering without changing its eight-term recognition limit.'
    ]+previous_review['remaining_work']
    reviewed = {
        'crates/hyperreal/src/computable/node/symbolic_work.rs':'Complete SymbolicWork helper implementation, including the shared raw-width/canonical-value preflight.',
        'crates/hyperreal/src/computable/node/structural_analysis.rs':'AlgebraicSeparation bounded-rational helper and its two callers, complete bounded_rational_value recognizer, and exact_sign root certificate checks through initial traversal allocation. No new whole-file audit.',
        'crates/hyperreal/src/computable/node/domain_certificates.rs':'Complete finite-real certificate visitor and its sign/range helper fallbacks.',
        'crates/hyperreal/src/computable/node/domain_validation.rs':'ensure_finite_real preflight and traversal initialization, and negative square-root/NthRoot error branches. No new whole-file audit.',
        'crates/hyperreal/src/computable/node/approximation_queries.rs':'immediate_sign retained-fact semantics and the public approximation entry/cache dispatch; no new whole-file audit.',
        'crates/hyperreal/src/computable/budget.rs':'EvaluationBudget and BudgetScope setup through active-scope detection; no new whole-file audit.',
        'crates/hyperreal/src/computable/node/rational_preflight_tests.rs':'Complete ten frozen regression/preservation tests and fixture helpers.'
    }
    followup = {
        'status':'Scoped rational/domain preflight repair; remaining admission work stays open',
        'previous_review':'algebraic-preflight-resource-followup-v1133.json',
        'retained_supplemental_review':'algebraic-preflight-supplemental-review-v1133-corrected.json',
        'source_record':source_record,'source_record_sha256':EXPECTED,
        'reviewed_source_slices':{n:{'sha256':source[n],'scope':scope} for n,scope in reviewed.items()},
        'closed_scoped_gaps':implementation['changes'],
        'public_reachability_evidence':'Fifty-six public approx_with_budget cases measure width-dependent Rust allocations at low allowance across fourteen fresh domain shapes. They all return Exhausted both before and after repair; allocation requests change from up to 2500712 bytes to a fixed 40-byte BudgetScope allocation, equal to the fresh canonical control. Forty-two retry schedules preserve subsequent approximations, signs and zero-work retained proofs. This is direct public allocation evidence for those exact fixtures, not the complete call graph.',
        'recognition_work_changes':implementation['measured_work_observations'],
        'next_candidates':next_candidates,'remaining_work':remaining,'limits':limits
    }
    write('rational-preflight-resource-followup-v1195.json',followup)
    write('rational-preflight-regression-coverage-v1195.json',{
        'status':'Isolated rational/domain preflight repair validated; no runtime admission',
        'source_record_sha256':EXPECTED,'changed_sources':changed,
        'contract_decision':'p4-contract-decision-v173.json','unit_coverage':implementation['unit_coverage'],
        'retained_failure_record':'rational-preflight-baseline-v1146.json',
        'allocation_observations':implementation['allocation_observations'],
        'measured_work_observations':implementation['measured_work_observations'],
        'allocation_scope':implementation['allocation_scope'],'final_source_replays':replays,
        'prior_named_test_bodies_and_assertions_unchanged':True,
        'all_ten_baseline_tests_values_caps_and_assertions_unchanged':True,
        'all_preserved_probe_inputs_caps_and_arguments_unchanged':True,
        'focused_replay_comparison':implementation['replays'],
        'cost_records':implementation['cost_records'],'paired_cost':implementation['paired_cost'],
        'targeted_confirmation':implementation['targeted_confirmation'],'limits':limits})
    assert len(sys.argv) == 2,'implementation commit required'
    write('typed-api-prototype-v1195-metadata.json',{
        'previous_checkpoint':'typed-api-prototype-v1133-state.json','implementation_commits':[sys.argv[1]],
        'cost_baseline_commit':'f00bfc5cd','implementation_records':['rational-preflight-implementation-v1161.json'],
        'status':'isolated rational/domain lazy-preflight repair; no runtime admission',
        'contract_decision':'p4-contract-decision-v173.json','changes':followup['closed_scoped_gaps'],
        'retained_regression_record':'rational-preflight-regression-coverage-v1195.json',
        'retained_failure_record':'rational-preflight-baseline-v1146.json',
        'test_summaries':summaries,'capped_unit_regressions':caps,'final_source_replays':replays,
        'included_rust_fragments_checked':len(formatted),'formal_reuse_record':'rational-preflight-proof-reuse-v1195.json',
        'formal_validation':proof,'cost_records':implementation['cost_records'],'paired_cost':implementation['paired_cost'],
        'targeted_confirmation':implementation['targeted_confirmation'],
        'retention_record':'rational-preflight-retention-v1161.json',
        'inherited_warnings':{'identical_cargo_filename_warning_lines':inherited,'strict_clippy_warning_lines':0},
        'coverage_limits':limits,'remaining_source_review':'rational-preflight-resource-followup-v1195.json',
        'retained_parser_correction':'algebraic-preflight-parser-correction-v1133.json',
        'completed_orchestration':'rational-preflight-orchestration-v1195.json',
        'remaining_requirements':remaining+limits})
    print(json.dumps({'tests':summaries,'capped_tests':125,'allocation_child_tests':1,
        'replay_cases':4129,'formatted_rust_files':len(formatted),'changed_sources':len(changed)}))


if __name__ == '__main__':
    main()
