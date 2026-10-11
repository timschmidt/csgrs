#!/usr/bin/env python3
"""Record scoped inverse-trig validation and the remaining admission work."""
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
EXPECTED = '4afe53ae2773719d18b0c314fc1a89cc43adeed8a3540cfd9ab0d56f27beb45c'
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
    prior = read(E/'typed-api-prototype-v729-state.json')
    implementation = read(E/'inverse-trig-initial-trials-v760.json')
    fast = read(E/'inverse-trig-inline-v804.json')
    assert fast['source_record_sha256'] == EXPECTED
    proof = read(E/'inverse-trig-budget-proof-reuse-v834.json')
    assert proof['source_record_sha256'] == EXPECTED and proof['verified_units'] == 638 and proof['errors'] == 0
    assert len(proof['proof_and_guard_inputs_sha256']) == 44 and proof['reused_rejected_mutants'] == 213
    assert proof['fresh_acceptance_tests'] == 10 and proof['open_obligation_groups'] == 13
    source_record = 'typed-api-prototype-v805-budget-verus/source-hashes.json.gz'
    source = json.loads(gzip.decompress((E/source_record).read_bytes()))
    before = json.loads(gzip.decompress((E/prior['source_record']).read_bytes()))
    assert all(sha(SOURCE/n) == v for n,v in source.items())
    changed = {n:{'before':before.get(n),'after':v} for n,v in source.items() if before.get(n) != v}
    assert set(changed) == set(implementation['changed_sources']) and len(changed) == 5
    assert all(before[n] == source[n] for n in before if 'test' in n)
    summaries = {}
    for key,version,suffix in [('debug_scalar',807,'debug-scalar-unit-tests'),('five_crate',808,'stack-tests'),('default_scalar',809,'default-tests')]:
        actual = parse_phase(completed(f'typed-api-prototype-v{version}-{suffix}'))
        record = f'typed-api-prototype-v{version}-test-summary.json'
        assert actual == read(E/record)
        old = prior['test_summaries'][key]
        assert actual['top_level_passed'] == old['passed'] + 9
        assert actual['child_process_passed'] == old['child_passed'] and actual['top_level_ignored'] == old['ignored']
        assert actual['top_level_failed'] == actual['child_process_failed'] == 0
        summaries[key] = {'record':record,'passed':actual['top_level_passed'],'child_passed':actual['child_process_passed'],'ignored':actual['top_level_ignored']}
    caps = {}
    for version,suffix,count in [(813,'private-kernel',7),(814,'magnitude',5),(815,'publication',7),(816,'fractional',5),(817,'scaled-lower',1),(818,'trig-range',5),(819,'trig-lattice',5),(820,'trig-residual',6),(821,'trig-operand',6),(822,'evaluation-budget',7),(823,'algebraic-budget',4),(824,'normal-form-budget',8),(825,'inverse-trig-budget',9)]:
        phase = f'typed-api-prototype-v{version}-{suffix}-cap'
        path = completed(phase)
        log = (path/'stdout.log').read_text()
        receipt = json.loads(log.splitlines()[-1])
        assert receipt['exit_code'] == 0 and not receipt['wall_timeout']
        assert re.findall(r'^test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;',log,re.M) == [(str(count),'0','0')]
        assert receipt['address_space_bytes'] == 192*1024*1024 and receipt['cpu_seconds_soft_hard'] == [2,3] and receipt['wall_seconds_limit'] == 4
        caps[phase] = receipt | {'passed':count,'phase_result':read(path/'result.json')}
    assert sum(r['passed'] for r in caps.values()) == 75
    replays = {}
    for version,old_version,suffix,count in [(826,721,'replay',415),(827,722,'precision-replay',60),(828,723,'private-kernel-replay',78),(829,724,'private-trig-replay',600),(830,725,'private-trig-wide-replay',384),(831,726,'symbolic-trig-scale-replay',168),(832,727,'symbolic-trig-residual-replay',456),(833,728,'symbolic-trig-nonrational-replay',384)]:
        phase = f'typed-api-prototype-v{version}-{suffix}'
        path,old = E/phase,E/f'typed-api-prototype-v{old_version}-{suffix}'
        data = read(path/'summary.json')
        assert data['source_hashes_sha256'] == EXPECTED and data['guards_unchanged']
        for name in ['probe.rs','candidate_probe.rs','cache_candidate_probe.rs','reproduce.py','oracles.json']:
            if (old/name).is_file(): assert sha(old/name) == sha(path/name), (phase,name)
        assert [command_contract(r) for r in case_commands(old)] == [command_contract(r) for r in case_commands(path)]
        if version == 826:
            assert data['ordinary_cases'] == 400 and data['typed_interruption_errors'] == 300
            assert data['invalid_first_contracts'] == data['invalid_retries'] == 0
            assert len(data['extremes']) == 6 and all(r['valid'] for r in data['extremes'])
            assert len(data['cache']) == 9 and all(r['valid'] for r in data['cache'])
            assert all(r['exit_code'] == 0 for r in data['commands'])
        else:
            assert data['cases'] == count
            if version == 827:
                assert data['valid_cases'] == 60 and not data['nonzero_exit_cases'] and not data['invalid_numeric_cases']
            elif version == 828:
                assert data['incomplete_cases'] == data['unexpected_result_shapes'] == 0
            elif version == 831:
                assert data['valid_samples'] == 672 and data['incomplete_cases'] == data['invalid_samples'] == 0
            else:
                assert data['numeric_samples'] == 4*count
                assert data['incomplete_cases'] == data['invalid_numeric_samples'] == data['typed_errors'] == 0
        replays[phase] = {'cases':count,'summary_sha256':sha(path/'summary.json')}
    for phase,value in fast['replays'].items():
        data = read(E/phase/'summary.json')
        assert data['source_hashes_sha256'] == EXPECTED and data['guards_unchanged'] and data['cases'] == value['cases']
        assert data['incomplete_cases'] == 0 and value['answers_samples_work_and_proofs_match_initial_repair']
        assert sha(E/phase/'summary.json') == value['summary_sha256']
        replays[phase] = {'cases':value['cases'],'summary_sha256':value['summary_sha256']}
    assert sum(r['cases'] for r in replays.values()) == 4129
    formatted = read(completed('typed-api-prototype-v812-included-fmt')/'command.json')['command']
    formatted = formatted[formatted.index('--check')+1:]
    original = json.loads(gzip.decompress((E/'candidate-preparation/source-hashes.json.gz').read_bytes()))
    assert formatted == sorted(n for n,v in source.items() if n.endswith('.rs') and original.get(n) != v)
    warnings = lambda p:[r for r in p.read_text().splitlines() if r.startswith('warning:')]
    inherited = warnings(completed('typed-api-prototype-v808-stack-tests')/'stderr.log')
    assert inherited == warnings(E/'typed-api-prototype-v553-stack-tests/stderr.log') and len(inherited) == 14
    for phase in ['typed-api-prototype-v792-inverse-trig-inline-clippy','typed-api-prototype-v810-minimal-clippy']:
        assert not warnings(completed(phase)/'stderr.log')
    limits = fast['limits'] + [
        'The 78 private-kernel cases check result shapes. Ordinary unbudgeted recognition retains its original depth limits and shared-DAG expansion behavior; the new capped DAG test establishes active-allowance termination only.',
        'The changed runtime is outside the conditional Verus target. Algebraic leaf copying, direct Real radical/quadratic-tower paths and individual exact arithmetic operations still require separate resource review.',
        'Main retains the original scalar failure. All 49 original worktree entries and complete plan scope remain preserved; no push was requested or performed.'
    ]
    # Source-review and metadata records are completed below after all validation.
    followup = {
        'status':'Scoped inverse-trig source review and remaining work; no runtime admission',
        'previous_review':'normal-form-resource-followup-v729.json','source_record':source_record,'source_record_sha256':EXPECTED,
        'reviewed_source_slices':{
            n:{'sha256':source[n],'scope':scope} for n,scope in {
                'crates/hyperreal/src/computable/node/symbolic_work.rs':'Complete helper, including new shift preflight; no change to its existing rational word weights.',
                'crates/hyperreal/src/computable/node/inverse_trig_budget_tests.rs':'Complete nine-test file.',
                'crates/hyperreal/src/computable/node/algebra.rs':'Inverse-trig temporary forms, recursive recognition, linear sign search, collapse and its independent finite-domain caller guards; retained pi and canonical sine helpers; selected Laurent argument/projection call sites. No whole-file audit.',
                'crates/hyperreal/src/computable/node/exp_trig.rs':'atan/asin/acos argument extraction and its call sites; the separate recursive atan_rational_argument remains unmetered. No whole-file audit.',
                'crates/hyperreal/src/computable/node/structural_analysis.rs':'exact_sign publication path, quadratic-surd projection, rational_root and immediate dependency caller; no whole-file audit.',
                'crates/hyperreal/src/real/arithmetic/elementary_functions.rs':'retained_atan_argument and immediate retained integer-pi argument adapter only; distinguishes Real call paths from the changed Computable budget paths.'
            }.items()
        },
        'closed_scoped_gaps':implementation['changes']+[fast['change']],
        'next_candidates':[
            {'path':'crates/hyperreal/src/computable/node/structural_analysis.rs','operations':['AlgebraicSeparation::rational_root','rational_root_depends_on_existing'],'finding':'rational_root still clones exact rational leaves before the dependency-size restriction and without the new argument-copy debit. Both candidate and existing-base callers treat unsupported recognition as a declined dependency search; a fallible replacement must propagate shared exhaustion separately. Preserve cheap unsupported-size rejection before allocation work.'},
            {'path':'crates/hyperreal/src/computable/node/exp_trig.rs','operations':['atan_rational_argument'],'finding':'This separate helper recursively follows Negate and clones its rational leaf without shared accounting. The inspected caller is Real::retained_atan_argument; review actual active-budget reachability before treating it as the same Computable query path.'},
            {'path':'crates/hyperreal/src/computable/node/algebra.rs','operations':['ordinary inverse-trig parsers','construction-time cancellation'],'finding':'The new shared-DAG cap establishes termination under an active allowance. Ordinary unbudgeted parsing still has the old recursive expansion behavior. Selected constructor comparisons and direct Real radical/quadratic-tower work remain separate accounting/reuse reviews.'}
        ],
        'remaining_work':[
            'Review algebraic leaf copying before size rejection and preserve conservative dependency results on interruption.',
            'Distinguish direct Real helper reachability, default shared-DAG expansion, and individual exact-operation time/allocation from the scoped cooperative Computable repair.',
            'Continue kernel-specific interruption cutpoints, public concurrent/reconstructed and cross-root warming schedules, implementation-connected proofs, composed ln/exp/inverse-hyperbolic planning, native-prototype combination and unchanged full-board admission.'
        ],'limits':limits
    }
    write('inverse-trig-resource-followup-v834.json',followup)
    write('inverse-trig-regression-coverage-v834.json',{
        'status':'Isolated inverse-trig repair validated; no runtime admission','source_record_sha256':EXPECTED,'changed_sources':changed,
        'contract_decision':'p4-contract-decision-v173.json','unit_coverage':implementation['regression_coverage'],
        'retained_failures':implementation['retained_initial_regressions'],'source_corrections':implementation['source_corrections'],
        'final_source_replays':replays,'all_prior_named_tests_unchanged':True,
        'all_preserved_probe_inputs_caps_and_arguments_unchanged':True,
        'all_1584_focused_answers_samples_work_and_proofs_unchanged_by_inlining':True,
        'cost_records':fast['cost_records'],'paired_cost':fast['paired_cost'],'limits':limits})
    write('typed-api-prototype-v834-metadata.json',{
        'previous_checkpoint':'typed-api-prototype-v729-state.json','implementation_commits':['9ed9e30bc','9054b07a2'],'cost_baseline_commit':'11b2d466b',
        'implementation_records':['inverse-trig-initial-trials-v760.json','inverse-trig-inline-v804.json'],
        'status':'isolated inverse-trig and retained-pi work accounting with retryable partial searches; no runtime admission',
        'contract_decision':'p4-contract-decision-v173.json','changes':followup['closed_scoped_gaps'],
        'retained_regression_record':'inverse-trig-regression-coverage-v834.json',
        'retained_intermediate_failures':implementation['retained_initial_regressions'],
        'test_summaries':summaries,'capped_unit_regressions':caps,'final_source_replays':replays,
        'included_rust_fragments_checked':len(formatted),'formal_reuse_record':'inverse-trig-budget-proof-reuse-v834.json',
        'formal_validation':proof,'cost_records':fast['cost_records'],'paired_cost':fast['paired_cost'],
        'inherited_warnings':{'identical_cargo_filename_warning_lines':inherited,'strict_clippy_warning_lines':0},
        'coverage_limits':limits,'remaining_source_review':'inverse-trig-resource-followup-v834.json',
        'remaining_requirements':followup['remaining_work']+limits})
    print(json.dumps({'tests':summaries,'capped_tests':75,'replay_cases':4129,'formatted_rust_files':len(formatted),'changed_sources':len(changed)}))


if __name__ == '__main__':
    main()
