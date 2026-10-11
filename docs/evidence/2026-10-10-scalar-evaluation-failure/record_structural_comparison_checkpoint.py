#!/usr/bin/env python3
"""Record wide structural comparison validation and the remaining admission work."""
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
EXPECTED = 'a0d1d543babfb7ae724d0cfdebb6a294c33b59423b94223a0468716413a1b00b'
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
    prior = read(E/'typed-api-prototype-v1011-state.json')
    implementation = read(E/'structural-comparison-implementation-v1042.json')
    fast = implementation
    assert fast['source_record_sha256'] == EXPECTED
    proof = read(E/'structural-comparison-proof-reuse-v1074.json')
    assert proof['source_record_sha256'] == EXPECTED and proof['verified_units'] == 638 and proof['errors'] == 0
    assert len(proof['proof_and_guard_inputs_sha256']) == 44 and proof['reused_rejected_mutants'] == 213
    assert proof['fresh_acceptance_tests'] == 10 and proof['open_obligation_groups'] == 13
    source_record = 'typed-api-prototype-v1043-budget-verus/source-hashes.json.gz'
    source = json.loads(gzip.decompress((E/source_record).read_bytes()))
    before = json.loads(gzip.decompress((E/prior['source_record']).read_bytes()))
    assert all(sha(SOURCE/n) == v for n,v in source.items())
    changed = {n:{'before':before.get(n),'after':v} for n,v in source.items() if before.get(n) != v}
    assert set(changed) == set(implementation['changed_sources']) and len(changed) == 5
    assert all(before[n] == source[n] for n in before if 'test' in n)
    assert implementation['prior_named_test_bodies_and_assertions_unchanged']
    summaries = {}
    for key,version,suffix in [('debug_scalar',1045,'debug-scalar-unit-tests'),('five_crate',1046,'stack-tests'),('default_scalar',1047,'default-tests')]:
        actual = parse_phase(completed(f'typed-api-prototype-v{version}-{suffix}'))
        record = f'typed-api-prototype-v{version}-test-summary.json'
        assert actual == read(E/record)
        old = prior['test_summaries'][key]
        assert actual['top_level_passed'] == old['passed'] + 14
        assert actual['child_process_passed'] == old['child_passed'] and actual['top_level_ignored'] == old['ignored']
        assert actual['top_level_failed'] == actual['child_process_failed'] == 0
        summaries[key] = {'record':record,'passed':actual['top_level_passed'],'child_passed':actual['child_process_passed'],'ignored':actual['top_level_ignored']}
    caps = {}
    for version,suffix,count in [(1029,'structural-comparison-repair',14),(1065,'inverse-dag',10),(1064,'algebraic-leaf',6),(1051,'private-kernel',7),(1052,'magnitude',5),(1053,'publication',7),(1054,'fractional',5),(1055,'scaled-lower',1),(1056,'trig-range',5),(1057,'trig-lattice',5),(1058,'trig-residual',6),(1059,'trig-operand',6),(1060,'evaluation-budget',7),(1061,'algebraic-budget',4),(1062,'normal-form-budget',8),(1063,'inverse-trig-budget',9)]:
        phase = f'typed-api-prototype-v{version}-{suffix}-cap'
        path = completed(phase)
        log = (path/'stdout.log').read_text()
        receipt = json.loads(log[log.rfind('{"command":'):])
        assert receipt['exit_code'] == 0 and not receipt['wall_timeout']
        assert re.findall(r'^test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;',log,re.M) == ([('1','0','0')] if version == 1065 else []) + [(str(count),'0','0')]
        assert receipt['address_space_bytes'] == 192*1024*1024 and receipt['cpu_seconds_soft_hard'] == [2,3] and receipt['wall_seconds_limit'] == 4
        caps[phase] = receipt | {'passed':count,'child_passed':int(version == 1065),'phase_result':read(path/'result.json')}
    assert sum(r['passed'] for r in caps.values()) == 105
    replays = {}
    for version,old_version,suffix,count in [(1066,1003,'replay',415),(1067,1004,'precision-replay',60),(1068,1005,'private-kernel-replay',78),(1069,1006,'private-trig-replay',600),(1070,1007,'private-trig-wide-replay',384),(1071,1008,'symbolic-trig-scale-replay',168),(1072,1009,'symbolic-trig-residual-replay',456),(1073,1010,'symbolic-trig-nonrational-replay',384)]:
        phase = f'typed-api-prototype-v{version}-{suffix}'
        path,old = E/phase,E/f'typed-api-prototype-v{old_version}-{suffix}'
        data = read(path/'summary.json')
        assert data['source_hashes_sha256'] == EXPECTED and data['guards_unchanged']
        for name in ['probe.rs','candidate_probe.rs','cache_candidate_probe.rs','reproduce.py','oracles.json']:
            if (old/name).is_file(): assert sha(old/name) == sha(path/name), (phase,name)
        assert [command_contract(r) for r in case_commands(old)] == [command_contract(r) for r in case_commands(path)]
        if version == 1066:
            assert data['ordinary_cases'] == 400 and data['typed_interruption_errors'] == 300
            assert data['invalid_first_contracts'] == data['invalid_retries'] == 0
            assert len(data['extremes']) == 6 and all(r['valid'] for r in data['extremes'])
            assert len(data['cache']) == 9 and all(r['valid'] for r in data['cache'])
            assert all(r['exit_code'] == 0 for r in data['commands'])
        else:
            assert data['cases'] == count
            if version == 1067:
                assert data['valid_cases'] == 60 and not data['nonzero_exit_cases'] and not data['invalid_numeric_cases']
            elif version == 1068:
                assert data['incomplete_cases'] == data['unexpected_result_shapes'] == 0
            elif version == 1071:
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
    formatted = read(completed('typed-api-prototype-v1050-included-fmt')/'command.json')['command']
    formatted = formatted[formatted.index('--check')+1:]
    original = json.loads(gzip.decompress((E/'candidate-preparation/source-hashes.json.gz').read_bytes()))
    assert formatted == sorted(n for n,v in source.items() if n.endswith('.rs') and original.get(n) != v)
    warnings = lambda p:[r for r in p.read_text().splitlines() if r.startswith('warning:')]
    inherited = warnings(completed('typed-api-prototype-v1046-stack-tests')/'stderr.log')
    assert inherited == warnings(E/'typed-api-prototype-v553-stack-tests/stderr.log') and len(inherited) == 14
    for phase in ['typed-api-prototype-v1031-structural-comparison-clippy','typed-api-prototype-v1048-minimal-clippy']:
        assert not warnings(completed(phase)/'stderr.log')
    limits = implementation['limits'] + [
        'The 78 private-kernel replay cases validate result shapes. Fourteen new comparison tests do not establish bounded resources for every symbolic operation.',
        'Local timings and standalone probe sizes are scoped observations. Whole-application performance and runtime admission remain open.',
        'Sixteen wide algebraic replay cases change only cooperative consumed/remaining counts. All answers, numerical observations, certificates, inputs, allowances and resource caps remain identical.',
        'Main runtime remains unadmitted; no push was requested or performed.'
    ]
    previous_review = read(E/'inverse-dag-resource-followup-v1011.json')
    call_sites = read(E/'structural-comparison-call-sites-v1074.json')
    assert call_sites['source_record_sha256'] == EXPECTED
    reviewed = {
        'crates/hyperreal/src/computable/node/primitive_constructors.rs':
            'Complete structural comparison implementation, including both literal helpers, bounded walk and memoized fallback. No whole-file audit.',
        'crates/hyperreal/src/computable/node/symbolic_work.rs':
            'Complete symbolic-work preflight helper implementation.',
        'crates/hyperreal/src/rational/arithmetic/queries_conversion.rs':
            'Stored identity, class and new component-width accessor, public numeric component accessors, integer magnitude and stored sign. No whole-file audit.',
        'crates/hyperreal/src/rational/arithmetic/construction.rs':
            'Raw and unreduced construction plus canonicalized_ref; selected call sites. No whole-file audit.',
        'crates/hyperreal/src/rational/arithmetic/comparison.rs':
            'Complete PartialEq implementation and adjacent PartialOrd dispatch. No whole-file audit.',
        'crates/hyperreal/src/computable/node/structural_comparison_budget_tests.rs':
            'Complete fourteen tests and their fixture/measurement helpers.'
    }
    followup = {
        'status':'Scoped structural payload and lazy rational preflight repair; remaining admission work stays open',
        'previous_review':'inverse-dag-resource-followup-v1011.json',
        'source_record':source_record,'source_record_sha256':EXPECTED,
        'partial_reachability_review':'algebraic-leaf-reachability-review-v927.json',
        'supplemental_call_sites_review':'structural-comparison-call-sites-v1074.json',
        'reviewed_source_slices':{n:{'sha256':source[n],'scope':scope} for n,scope in reviewed.items()},
        'closed_scoped_gaps':implementation['changes'],
        'public_reachability_evidence':'The frozen reconstructed-atan public sign test demonstrates that allowance-scoped structural comparison reaches wide rational arguments. Low work stops the cold search; a generous retry certifies zero; the completed certificate remains available at zero work.',
        'next_candidates':previous_review['next_candidates'][:3]+previous_review['next_candidates'][4:],
        'remaining_work':previous_review['remaining_work'],
        'limits':limits
    }
    write('structural-comparison-resource-followup-v1074.json',followup)
    write('structural-comparison-regression-coverage-v1074.json',{
        'status':'Isolated structural comparison repair validated; no runtime admission',
        'source_record_sha256':EXPECTED,'changed_sources':changed,
        'contract_decision':'p4-contract-decision-v173.json','unit_coverage':implementation['unit_coverage'],
        'retained_failure_record':'structural-comparison-baseline-v1024.json',
        'allocation_observations':implementation['allocation_observations'],
        'measured_work_observations':implementation['measured_work_observations'],
        'allocation_scope':implementation['allocation_scope'],
        'final_source_replays':replays,'prior_named_test_bodies_and_assertions_unchanged':True,
        'all_twelve_baseline_tests_values_caps_and_assertions_unchanged':True,
        'all_preserved_probe_inputs_caps_and_arguments_unchanged':True,
        'cooperative_work_count_changes':{k:v['cooperative_work_count_changes'] for k,v in fast['replays'].items()
            if v['cooperative_work_count_changes']},
        'cost_records':implementation['cost_records'],'paired_cost':implementation['paired_cost'],'limits':limits})
    assert len(sys.argv) == 2, 'implementation commit required'
    write('typed-api-prototype-v1074-metadata.json',{
        'previous_checkpoint':'typed-api-prototype-v1011-state.json','implementation_commits':[sys.argv[1]],
        'cost_baseline_commit':'a8e3c47c0','implementation_records':['structural-comparison-implementation-v1042.json'],
        'status':'isolated wide structural-comparison and lazy rational preflight repair; no runtime admission',
        'contract_decision':'p4-contract-decision-v173.json','changes':followup['closed_scoped_gaps'],
        'retained_regression_record':'structural-comparison-regression-coverage-v1074.json',
        'retained_failure_record':'structural-comparison-baseline-v1024.json',
        'test_summaries':summaries,'capped_unit_regressions':caps,'final_source_replays':replays,
        'included_rust_fragments_checked':len(formatted),'formal_reuse_record':'structural-comparison-proof-reuse-v1074.json',
        'formal_validation':proof,'cost_records':implementation['cost_records'],'paired_cost':implementation['paired_cost'],
        'targeted_confirmation':implementation['targeted_confirmation'],
        'retention_record':'structural-comparison-retention-v1042.json',
        'inherited_warnings':{'identical_cargo_filename_warning_lines':inherited,'strict_clippy_warning_lines':0},
        'coverage_limits':limits,'remaining_source_review':'structural-comparison-resource-followup-v1074.json',
        'supplemental_call_sites_review':'structural-comparison-call-sites-v1074.json',
        'remaining_requirements':followup['remaining_work']+limits})
    print(json.dumps({'tests':summaries,'capped_tests':105,'allocation_child_tests':1,
        'replay_cases':4129,'formatted_rust_files':len(formatted),'changed_sources':len(changed)}))


if __name__ == '__main__':
    main()
