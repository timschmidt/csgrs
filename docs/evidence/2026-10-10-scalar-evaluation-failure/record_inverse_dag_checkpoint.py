#!/usr/bin/env python3
"""Record scoped inverse-trig DAG validation and the remaining admission work."""
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
EXPECTED = '688d05cdbf832f87f9172da50a21a497134bc7fc0666ec046148fc787d70dacb'
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
    prior = read(E/'typed-api-prototype-v927-state.json')
    implementation = read(E/'inverse-dag-switch-v980.json')
    fast = implementation
    assert fast['source_record_sha256'] == EXPECTED
    proof = read(E/'inverse-dag-proof-reuse-v1011.json')
    assert proof['source_record_sha256'] == EXPECTED and proof['verified_units'] == 638 and proof['errors'] == 0
    assert len(proof['proof_and_guard_inputs_sha256']) == 44 and proof['reused_rejected_mutants'] == 213
    assert proof['fresh_acceptance_tests'] == 10 and proof['open_obligation_groups'] == 13
    source_record = 'typed-api-prototype-v981-budget-verus/source-hashes.json.gz'
    source = json.loads(gzip.decompress((E/source_record).read_bytes()))
    before = json.loads(gzip.decompress((E/prior['source_record']).read_bytes()))
    assert all(sha(SOURCE/n) == v for n,v in source.items())
    changed = {n:{'before':before.get(n),'after':v} for n,v in source.items() if before.get(n) != v}
    assert set(changed) == set(implementation['changed_sources']) and len(changed) == 4
    assert all(before[n] == source[n] for n in before if 'test' in n and not n.endswith('/algebraic_leaf_tests.rs'))
    assert implementation['prior_named_test_bodies_and_assertions_unchanged']
    summaries = {}
    for key,version,suffix in [('debug_scalar',983,'debug-scalar-unit-tests'),('five_crate',984,'stack-tests'),('default_scalar',985,'default-tests')]:
        actual = parse_phase(completed(f'typed-api-prototype-v{version}-{suffix}'))
        record = f'typed-api-prototype-v{version}-test-summary.json'
        assert actual == read(E/record)
        old = prior['test_summaries'][key]
        assert actual['top_level_passed'] == old['passed'] + 10
        assert actual['child_process_passed'] == old['child_passed'] + 1 and actual['top_level_ignored'] == old['ignored']
        assert actual['top_level_failed'] == actual['child_process_failed'] == 0
        summaries[key] = {'record':record,'passed':actual['top_level_passed'],'child_passed':actual['child_process_passed'],'ignored':actual['top_level_ignored']}
    caps = {}
    for version,suffix,count in [(967,'inverse-dag-repair',10),(1002,'algebraic-leaf',6),(989,'private-kernel',7),(990,'magnitude',5),(991,'publication',7),(992,'fractional',5),(993,'scaled-lower',1),(994,'trig-range',5),(995,'trig-lattice',5),(996,'trig-residual',6),(997,'trig-operand',6),(998,'evaluation-budget',7),(999,'algebraic-budget',4),(1000,'normal-form-budget',8),(1001,'inverse-trig-budget',9)]:
        phase = f'typed-api-prototype-v{version}-{suffix}-cap'
        path = completed(phase)
        log = (path/'stdout.log').read_text()
        receipt = json.loads(log[log.rfind('{"command":'):])
        assert receipt['exit_code'] == 0 and not receipt['wall_timeout']
        assert re.findall(r'^test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;',log,re.M) == ([('1','0','0')] if version == 967 else []) + [(str(count),'0','0')]
        assert receipt['address_space_bytes'] == 192*1024*1024 and receipt['cpu_seconds_soft_hard'] == [2,3] and receipt['wall_seconds_limit'] == 4
        caps[phase] = receipt | {'passed':count,'child_passed':int(version == 967),'phase_result':read(path/'result.json')}
    assert sum(r['passed'] for r in caps.values()) == 91
    replays = {}
    for version,old_version,suffix,count in [(1003,919,'replay',415),(1004,920,'precision-replay',60),(1005,921,'private-kernel-replay',78),(1006,922,'private-trig-replay',600),(1007,923,'private-trig-wide-replay',384),(1008,924,'symbolic-trig-scale-replay',168),(1009,925,'symbolic-trig-residual-replay',456),(1010,926,'symbolic-trig-nonrational-replay',384)]:
        phase = f'typed-api-prototype-v{version}-{suffix}'
        path,old = E/phase,E/f'typed-api-prototype-v{old_version}-{suffix}'
        data = read(path/'summary.json')
        assert data['source_hashes_sha256'] == EXPECTED and data['guards_unchanged']
        for name in ['probe.rs','candidate_probe.rs','cache_candidate_probe.rs','reproduce.py','oracles.json']:
            if (old/name).is_file(): assert sha(old/name) == sha(path/name), (phase,name)
        assert [command_contract(r) for r in case_commands(old)] == [command_contract(r) for r in case_commands(path)]
        if version == 1003:
            assert data['ordinary_cases'] == 400 and data['typed_interruption_errors'] == 300
            assert data['invalid_first_contracts'] == data['invalid_retries'] == 0
            assert len(data['extremes']) == 6 and all(r['valid'] for r in data['extremes'])
            assert len(data['cache']) == 9 and all(r['valid'] for r in data['cache'])
            assert all(r['exit_code'] == 0 for r in data['commands'])
        else:
            assert data['cases'] == count
            if version == 1004:
                assert data['valid_cases'] == 60 and not data['nonzero_exit_cases'] and not data['invalid_numeric_cases']
            elif version == 1005:
                assert data['incomplete_cases'] == data['unexpected_result_shapes'] == 0
            elif version == 1008:
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
    formatted = read(completed('typed-api-prototype-v988-included-fmt')/'command.json')['command']
    formatted = formatted[formatted.index('--check')+1:]
    original = json.loads(gzip.decompress((E/'candidate-preparation/source-hashes.json.gz').read_bytes()))
    assert formatted == sorted(n for n,v in source.items() if n.endswith('.rs') and original.get(n) != v)
    warnings = lambda p:[r for r in p.read_text().splitlines() if r.startswith('warning:')]
    inherited = warnings(completed('typed-api-prototype-v984-stack-tests')/'stderr.log')
    assert inherited == warnings(E/'typed-api-prototype-v553-stack-tests/stderr.log') and len(inherited) == 14
    for phase in ['typed-api-prototype-v969-inverse-dag-clippy','typed-api-prototype-v986-minimal-clippy']:
        assert not warnings(completed(phase)/'stderr.log')
    limits = implementation['limits'] + [
        'Final separate, paired and confirmation costs disagree for some fixtures, including wide inverse-trig terms. Broad performance admission remains open; the scoped retention rationale is in inverse-dag-retention-v980.json.',
        'The 78 private-kernel cases check result shapes. These ten DAG tests cover three inverse-trig parsers; they do not establish bounded resource use for every symbolic operation.',
        'Main runtime remains unadmitted; no push was requested or performed.'
    ]
    previous_review = read(E/'algebraic-leaf-resource-followup-v927.json')
    followup = {
        'status':'Scoped ordinary shared-DAG parsing repair; remaining admission work stays open',
        'previous_review':'algebraic-leaf-resource-followup-v927.json',
        'source_record':source_record,'source_record_sha256':EXPECTED,
        'partial_reachability_review':'algebraic-leaf-reachability-review-v927.json',
        'comparison_followup_review':'inverse-dag-comparison-review-v1011.json',
        'reviewed_source_slices':{
            'crates/hyperreal/src/computable/node/algebra.rs':{
                'sha256':source['crates/hyperreal/src/computable/node/algebra.rs'],
                'scope':'Complete three inverse-trig form types, copy/add/scale helpers and recursive parsers; selected constructor-collapse and sign callers. No whole-file audit.'},
            'crates/hyperreal/src/computable/node/inverse_trig_dag_tests.rs':{
                'sha256':source['crates/hyperreal/src/computable/node/inverse_trig_dag_tests.rs'],
                'scope':'Complete ten tests and allocation-process isolation.'}
        },
        'closed_scoped_gaps':implementation['changes'],
        'next_candidates':previous_review['next_candidates'][:3]+[{
            'path':'crates/hyperreal/src/computable/node/primitive_constructors.rs',
            'operations':['structural_eq_with_work','compare_nodes literal branches'],
            'finding':'Traversal is already bounded then memoized. Literal comparisons debit a node unit without coefficient-width accounting at this layer; reproduce wide reconstructed arguments before editing. See inverse-dag-comparison-review-v1011.json.'},{
            'path':'crates/hyperreal/src/computable/node/algebra.rs',
            'operations':['constructor-time comparison','general unshared form growth','memo coefficient allocation'],
            'finding':'Depth-aware query-local memoization removes ordinary repeated expansion in the frozen shared-DAG fixtures. Unshared growth and individual exact operations require separate resource review.'}],
        'remaining_work':[
            previous_review['remaining_work'][0],
            'Review unshared symbolic form growth, construction-time structural comparison, algebraic metadata vector allocation and individual exact-operation resource use with unchanged adversarial fixtures.',
            previous_review['remaining_work'][2]
        ],'limits':limits
    }
    write('inverse-dag-resource-followup-v1011.json',followup)
    write('inverse-dag-regression-coverage-v1011.json',{
        'status':'Isolated inverse-trig DAG repair validated; no runtime admission',
        'source_record_sha256':EXPECTED,'changed_sources':changed,
        'contract_decision':'p4-contract-decision-v173.json','unit_coverage':implementation['unit_coverage'],
        'retained_failure_record':'inverse-dag-baseline-v941.json',
        'allocation_observations':implementation['allocation_observations'],
        'measured_work_observations':implementation['measured_work_observations'],
        'allocation_scope':implementation['allocation_scope'],
        'final_source_replays':replays,'prior_named_test_bodies_and_assertions_unchanged':True,
        'baseline_values_depths_caps_and_assertions_unchanged':True,
        'allocation_isolation_patch':implementation['allocation_isolation_patch'],
        'allocation_test_correction':implementation['allocation_test_correction'],
        'all_preserved_probe_inputs_caps_and_arguments_unchanged':True,
        'cost_records':implementation['cost_records'],'paired_cost':implementation['paired_cost'],'limits':limits})
    assert len(sys.argv) == 2, 'implementation commit required'
    write('typed-api-prototype-v1011-metadata.json',{
        'previous_checkpoint':'typed-api-prototype-v927-state.json','implementation_commits':['f9889c1d6',sys.argv[1]],
        'cost_baseline_commit':'bab220336','implementation_records':['inverse-dag-implementation-v965.json','inverse-dag-switch-v980.json'],
        'status':'isolated depth-aware inverse-trig DAG memoization; no runtime admission',
        'contract_decision':'p4-contract-decision-v173.json','changes':followup['closed_scoped_gaps'],
        'retained_regression_record':'inverse-dag-regression-coverage-v1011.json',
        'retained_failure_record':'inverse-dag-baseline-v941.json',
        'test_summaries':summaries,'capped_unit_regressions':caps,'final_source_replays':replays,
        'included_rust_fragments_checked':len(formatted),'formal_reuse_record':'inverse-dag-proof-reuse-v1011.json',
        'formal_validation':proof,'cost_records':implementation['cost_records'],'paired_cost':implementation['paired_cost'],
        'targeted_confirmation':implementation['targeted_confirmation'],
        'retention_record':'inverse-dag-retention-v980.json',
        'interrupted_paired_attempt':implementation['interrupted_paired_attempt'],
        'inherited_warnings':{'identical_cargo_filename_warning_lines':inherited,'strict_clippy_warning_lines':0},
        'coverage_limits':limits,'remaining_source_review':'inverse-dag-resource-followup-v1011.json',
        'remaining_requirements':followup['remaining_work']+limits})
    print(json.dumps({'tests':summaries,'capped_tests':91,'allocation_child_tests':1,'replay_cases':4129,'formatted_rust_files':len(formatted),'changed_sources':len(changed)}))


if __name__ == '__main__':
    main()
