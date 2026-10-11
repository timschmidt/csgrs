#!/usr/bin/env python3
"""Record scoped algebraic leaf validation and the remaining admission work."""
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
EXPECTED = 'e61f29beff523a6b5f14cec81c10f09e4beb6f1bf70d507d89d9ca63c3a46e01'
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
    prior = read(E/'typed-api-prototype-v834-state.json')
    implementation = read(E/'algebraic-leaf-switch-v897.json')
    fast = implementation
    assert fast['source_record_sha256'] == EXPECTED
    proof = read(E/'algebraic-leaf-proof-reuse-v927.json')
    assert proof['source_record_sha256'] == EXPECTED and proof['verified_units'] == 638 and proof['errors'] == 0
    assert len(proof['proof_and_guard_inputs_sha256']) == 44 and proof['reused_rejected_mutants'] == 213
    assert proof['fresh_acceptance_tests'] == 10 and proof['open_obligation_groups'] == 13
    source_record = 'typed-api-prototype-v898-budget-verus/source-hashes.json.gz'
    source = json.loads(gzip.decompress((E/source_record).read_bytes()))
    before = json.loads(gzip.decompress((E/prior['source_record']).read_bytes()))
    assert all(sha(SOURCE/n) == v for n,v in source.items())
    changed = {n:{'before':before.get(n),'after':v} for n,v in source.items() if before.get(n) != v}
    assert set(changed) == set(implementation['changed_sources']) and len(changed) == 3
    assert all(before[n] == source[n] for n in before if 'test' in n)
    summaries = {}
    for key,version,suffix in [('debug_scalar',900,'debug-scalar-unit-tests'),('five_crate',901,'stack-tests'),('default_scalar',902,'default-tests')]:
        actual = parse_phase(completed(f'typed-api-prototype-v{version}-{suffix}'))
        record = f'typed-api-prototype-v{version}-test-summary.json'
        assert actual == read(E/record)
        old = prior['test_summaries'][key]
        assert actual['top_level_passed'] == old['passed'] + 6
        assert actual['child_process_passed'] == old['child_passed'] and actual['top_level_ignored'] == old['ignored']
        assert actual['top_level_failed'] == actual['child_process_failed'] == 0
        summaries[key] = {'record':record,'passed':actual['top_level_passed'],'child_passed':actual['child_process_passed'],'ignored':actual['top_level_ignored']}
    caps = {}
    for version,suffix,count in [(887,'algebraic-leaf-repair',6),(906,'private-kernel',7),(907,'magnitude',5),(908,'publication',7),(909,'fractional',5),(910,'scaled-lower',1),(911,'trig-range',5),(912,'trig-lattice',5),(913,'trig-residual',6),(914,'trig-operand',6),(915,'evaluation-budget',7),(916,'algebraic-budget',4),(917,'normal-form-budget',8),(918,'inverse-trig-budget',9)]:
        phase = f'typed-api-prototype-v{version}-{suffix}-cap'
        path = completed(phase)
        log = (path/'stdout.log').read_text()
        receipt = json.loads(log.splitlines()[-1])
        assert receipt['exit_code'] == 0 and not receipt['wall_timeout']
        assert re.findall(r'^test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;',log,re.M) == [(str(count),'0','0')]
        assert receipt['address_space_bytes'] == 192*1024*1024 and receipt['cpu_seconds_soft_hard'] == [2,3] and receipt['wall_seconds_limit'] == 4
        caps[phase] = receipt | {'passed':count,'phase_result':read(path/'result.json')}
    assert sum(r['passed'] for r in caps.values()) == 81
    replays = {}
    for version,old_version,suffix,count in [(919,721,'replay',415),(920,722,'precision-replay',60),(921,723,'private-kernel-replay',78),(922,724,'private-trig-replay',600),(923,725,'private-trig-wide-replay',384),(924,726,'symbolic-trig-scale-replay',168),(925,727,'symbolic-trig-residual-replay',456),(926,728,'symbolic-trig-nonrational-replay',384)]:
        phase = f'typed-api-prototype-v{version}-{suffix}'
        path,old = E/phase,E/f'typed-api-prototype-v{old_version}-{suffix}'
        data = read(path/'summary.json')
        assert data['source_hashes_sha256'] == EXPECTED and data['guards_unchanged']
        for name in ['probe.rs','candidate_probe.rs','cache_candidate_probe.rs','reproduce.py','oracles.json']:
            if (old/name).is_file(): assert sha(old/name) == sha(path/name), (phase,name)
        assert [command_contract(r) for r in case_commands(old)] == [command_contract(r) for r in case_commands(path)]
        if version == 919:
            assert data['ordinary_cases'] == 400 and data['typed_interruption_errors'] == 300
            assert data['invalid_first_contracts'] == data['invalid_retries'] == 0
            assert len(data['extremes']) == 6 and all(r['valid'] for r in data['extremes'])
            assert len(data['cache']) == 9 and all(r['valid'] for r in data['cache'])
            assert all(r['exit_code'] == 0 for r in data['commands'])
        else:
            assert data['cases'] == count
            if version == 920:
                assert data['valid_cases'] == 60 and not data['nonzero_exit_cases'] and not data['invalid_numeric_cases']
            elif version == 921:
                assert data['incomplete_cases'] == data['unexpected_result_shapes'] == 0
            elif version == 924:
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
    formatted = read(completed('typed-api-prototype-v905-included-fmt')/'command.json')['command']
    formatted = formatted[formatted.index('--check')+1:]
    original = json.loads(gzip.decompress((E/'candidate-preparation/source-hashes.json.gz').read_bytes()))
    assert formatted == sorted(n for n,v in source.items() if n.endswith('.rs') and original.get(n) != v)
    warnings = lambda p:[r for r in p.read_text().splitlines() if r.startswith('warning:')]
    inherited = warnings(completed('typed-api-prototype-v901-stack-tests')/'stderr.log')
    assert inherited == warnings(E/'typed-api-prototype-v553-stack-tests/stderr.log') and len(inherited) == 14
    for phase in ['typed-api-prototype-v885-algebraic-leaf-switch-clippy','typed-api-prototype-v903-minimal-clippy']:
        assert not warnings(completed(phase)/'stderr.log')
    limits = implementation['limits'] + [
        'The 78 private-kernel cases check result shapes. Existing shared-DAG caps establish termination under active allowances only; ordinary unbudgeted expansion remains open.',
        'Algebraic metadata memo/generator vectors, other direct Real helpers and individual exact arithmetic operations require separate allocation and reachability review.',
        'Main retains the original scalar failure. All 49 original worktree entries and complete plan scope remain preserved; no push was requested or performed.'
    ]
    followup = {
        'status':'Scoped algebraic leaf copying and scan repair; remaining admission work is open',
        'previous_review':'inverse-trig-resource-followup-v834.json',
        'source_record':source_record,'source_record_sha256':EXPECTED,
        'partial_reachability_review':'algebraic-leaf-reachability-review-v927.json',
        'reviewed_source_slices':{
            n:{'sha256':source[n],'scope':scope} for n,scope in {
                'crates/hyperreal/src/computable/node/structural_analysis.rs':'Complete AlgebraicSeparation implementation, algebraic metadata parser and exact_rational leaf accessor; no whole-file audit.',
                'crates/hyperreal/src/computable/node/symbolic_work.rs':'Complete borrowed-work helper, including fallible rational_leaf extraction; unchanged in this repair.',
                'crates/hyperreal/src/computable/node/algebraic_leaf_tests.rs':'Complete six-test file and test-only forwarding allocator.',
                'crates/hyperreal/src/computable/node/evaluation_budget.rs':'Complete two public Computable allowance wrappers and interruption helpers.',
                'crates/hyperreal/src/computable/node/exp_trig.rs':'atan_rational_argument and adjacent argument accessors only.',
                'crates/hyperreal/src/real/arithmetic/elementary_functions.rs':'retained_atan_argument and retained_integer_pi_plus_or_minus_atan_argument adapters only.',
                'crates/hyperreal/src/real/arithmetic/quadratic_tower_sign.rs':'Production Real methods before the test modules, including compact/sign/branch/polynomial adapters.'
            }.items()
        },
        'closed_scoped_gaps':implementation['changes'],
        'next_candidates':[
            {'path':'crates/hyperreal/src/computable/node/exp_trig.rs','operations':['atan_rational_argument'],
             'finding':'Recursive Negate recognition remains unmetered. Its inspected Real caller also constructs exact radicals. The reviewed public caller-allowance scopes live in Computable approximation/sign wrappers; trace any path back into this Real helper before asserting active-budget reachability.'},
            {'path':'crates/hyperreal/src/real/arithmetic/quadratic_tower_sign.rs','operations':['compact_quadratic_tower','quadratic_tower_sign','radical_tower_sign','sign_polynomial_at_rational_square_root'],
             'finding':'These direct Real APIs expose no allowance argument and construct/scale exact payloads before private recognizers. They require separate coverage of default resource behavior, retained proofs and any nested Computable allowance reachability.'},
            {'path':'crates/hyperreal/src/computable/node/structural_analysis.rs','operations':['AlgebraicSeparation memo clones','generator vectors','exact dependency arithmetic'],
             'finding':'The leaf repair avoids unsupported coefficient copies and charges supported scans/copies. Metadata vector allocation and the CPU/native allocation within individual exact operations are outside its allocation assertions.'},
            {'path':'crates/hyperreal/src/computable/node/algebra.rs','operations':['ordinary inverse-trig parsers','construction-time cancellation'],
             'finding':'Existing active-budget DAG tests do not bound ordinary recursive expansion or construction-time comparison costs.'}
        ],
        'remaining_work':[
            'Trace direct Real inverse-trig/radical/quadratic-tower helper reachability from the two public Computable allowance wrappers, preserving retained facts and explicit API scope.',
            'Review default shared-DAG expansion, algebraic metadata vector allocation and individual exact-operation resource use with unchanged adversarial fixtures.',
            'Continue kernel-specific interruption cutpoints, public concurrent/reconstructed and cross-root warming schedules, implementation-connected proofs, composed ln/exp/inverse-hyperbolic planning, native-prototype combination and unchanged full-board admission.'
        ],'limits':limits
    }
    write('algebraic-leaf-resource-followup-v927.json',followup)
    write('algebraic-leaf-regression-coverage-v927.json',{
        'status':'Isolated algebraic leaf repair validated; no runtime admission',
        'source_record_sha256':EXPECTED,'changed_sources':changed,
        'contract_decision':'p4-contract-decision-v173.json','unit_coverage':implementation['unit_coverage'],
        'retained_failure_record':'algebraic-leaf-baseline-v837.json',
        'allocation_observations':implementation['allocation_observations'],
        'measured_work_observations':implementation['measured_work_observations'],
        'allocation_scope':implementation['allocation_scope'],
        'final_source_replays':replays,'all_prior_named_tests_unchanged':True,
        'baseline_regression_bytes_unchanged':True,
        'all_preserved_probe_inputs_caps_and_arguments_unchanged':True,
        'cost_records':implementation['cost_records'],'paired_cost':implementation['paired_cost'],'limits':limits})
    assert len(sys.argv) == 2, 'implementation commit required'
    write('typed-api-prototype-v927-metadata.json',{
        'previous_checkpoint':'typed-api-prototype-v834-state.json','implementation_commits':['efe2f1976',sys.argv[1]],
        'cost_baseline_commit':'cae2e55f8','implementation_records':['algebraic-leaf-implementation-v852.json','algebraic-leaf-switch-v897.json'],
        'status':'isolated borrowed algebraic leaf rejection and metered coefficient scans; no runtime admission',
        'contract_decision':'p4-contract-decision-v173.json','changes':followup['closed_scoped_gaps'],
        'retained_regression_record':'algebraic-leaf-regression-coverage-v927.json',
        'retained_failure_record':'algebraic-leaf-baseline-v837.json',
        'test_summaries':summaries,'capped_unit_regressions':caps,'final_source_replays':replays,
        'included_rust_fragments_checked':len(formatted),'formal_reuse_record':'algebraic-leaf-proof-reuse-v927.json',
        'formal_validation':proof,'cost_records':implementation['cost_records'],'paired_cost':implementation['paired_cost'],
        'inherited_warnings':{'identical_cargo_filename_warning_lines':inherited,'strict_clippy_warning_lines':0},
        'coverage_limits':limits,'remaining_source_review':'algebraic-leaf-resource-followup-v927.json',
        'remaining_requirements':followup['remaining_work']+limits})
    print(json.dumps({'tests':summaries,'capped_tests':81,'replay_cases':4129,'formatted_rust_files':len(formatted),'changed_sources':len(changed)}))


if __name__ == '__main__':
    main()
