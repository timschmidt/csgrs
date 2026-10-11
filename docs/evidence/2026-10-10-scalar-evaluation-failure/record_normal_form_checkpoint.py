#!/usr/bin/env python3
"""Record the complete normal-form checkpoint without expanding admission claims."""
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
SOURCE = ROOT / 'target/scalar-evaluation-candidate/source'
EXPECTED = 'efe4009ef52bd9669d68ef548b496419a795e1162393eb9b1bbe9e3ebeb6e90d'
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def write(name, data):
    p = E / name
    assert not p.exists(), p
    p.write_text(json.dumps(data, indent=2) + '\n')


def completed(phase):
    p = E / phase
    r = read(p / 'result.json')
    assert r['exit_code'] == 0 and r['source_unchanged'] and r['main_unchanged'], (phase, r)
    assert sha(p / 'source-hashes.json.gz') == EXPECTED
    return p


def main():
    assert len(sys.argv) == 2, 'specialization commit required'
    prior = read(E / 'typed-api-prototype-v578-state.json')
    implementation = read(E / 'normal-form-budget-implementation-v657.json')
    fast = read(E / 'normal-form-specialization-v700.json')
    assert fast['source_record_sha256'] == EXPECTED
    proof = read(E / 'normal-form-budget-proof-reuse-v729.json')
    assert proof['source_record_sha256'] == EXPECTED and proof['verified_units'] == 638 and proof['errors'] == 0
    assert len(proof['proof_and_guard_inputs_sha256']) == 44 and proof['reused_rejected_mutants'] == 213
    assert proof['fresh_acceptance_tests'] == 10 and proof['open_obligation_groups'] == 13
    source_record = 'typed-api-prototype-v707-budget-verus/source-hashes.json.gz'
    source = json.loads(gzip.decompress((E / source_record).read_bytes()))
    before = json.loads(gzip.decompress((E / prior['source_record']).read_bytes()))
    assert all(sha(SOURCE / n) == v for n, v in source.items())
    changed = {n: {'before': before.get(n), 'after': v} for n, v in source.items() if before.get(n) != v}
    assert set(changed) == set(implementation['changed_sources']) and len(changed) == 6
    assert all(before[n] == source[n] for n in before if 'test' in n)
    summaries = {}
    for key, version, suffix in [('debug_scalar',701,'debug-scalar-unit-tests'), ('five_crate',702,'stack-tests'), ('default_scalar',703,'default-tests')]:
        actual = parse_phase(completed(f'typed-api-prototype-v{version}-{suffix}'))
        record = f'typed-api-prototype-v{version}-test-summary.json'
        assert actual == read(E / record)
        old = prior['test_summaries'][key]
        assert actual['top_level_passed'] == old['passed'] + 8
        assert actual['child_process_passed'] == old['child_passed'] and actual['top_level_ignored'] == old['ignored']
        assert actual['top_level_failed'] == actual['child_process_failed'] == 0
        summaries[key] = {'record': record, 'passed': actual['top_level_passed'], 'child_passed': actual['child_process_passed'], 'ignored': actual['top_level_ignored']}
    caps = {}
    for version, suffix, count in [(709,'private-kernel',7),(710,'magnitude',5),(711,'publication',7),(712,'fractional',5),(713,'scaled-lower',1),(714,'trig-range',5),(715,'trig-lattice',5),(716,'trig-residual',6),(717,'trig-operand',6),(718,'evaluation-budget',7),(719,'algebraic-budget',4),(720,'normal-form-budget',8)]:
        phase = f'typed-api-prototype-v{version}-{suffix}-cap'
        p = completed(phase)
        log = (p / 'stdout.log').read_text()
        receipt = json.loads(log.splitlines()[-1])
        assert receipt['exit_code'] == 0 and not receipt['wall_timeout']
        assert re.findall(r'^test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;',log,re.M) == [(str(count),'0','0')]
        assert receipt['address_space_bytes'] == 192*1024*1024 and receipt['cpu_seconds_soft_hard'] == [2,3] and receipt['wall_seconds_limit'] == 4
        caps[phase] = receipt | {'passed':count, 'phase_result':read(p/'result.json')}
    assert sum(r['passed'] for r in caps.values()) == 66
    replays = {}
    for version, old_version, suffix, count in [(721,569,'replay',415),(722,570,'precision-replay',60),(723,571,'private-kernel-replay',78),(724,572,'private-trig-replay',600),(725,573,'private-trig-wide-replay',384),(726,574,'symbolic-trig-scale-replay',168),(727,575,'symbolic-trig-residual-replay',456),(728,576,'symbolic-trig-nonrational-replay',384)]:
        phase = f'typed-api-prototype-v{version}-{suffix}'
        p, old = E / phase, E / f'typed-api-prototype-v{old_version}-{suffix}'
        data = read(p/'summary.json')
        assert data['source_hashes_sha256'] == EXPECTED and data['guards_unchanged']
        for name in ['probe.rs','candidate_probe.rs','cache_candidate_probe.rs','reproduce.py','oracles.json']:
            if (old/name).is_file(): assert sha(old/name) == sha(p/name), (phase,name)
        assert [command_contract(r) for r in case_commands(old)] == [command_contract(r) for r in case_commands(p)]
        if version == 721:
            assert data['ordinary_cases'] == 400 and data['typed_interruption_errors'] == 300
            assert data['invalid_first_contracts'] == data['invalid_retries'] == 0
            assert len(data['extremes']) == 6 and all(r['valid'] for r in data['extremes'])
            assert len(data['cache']) == 9 and all(r['valid'] for r in data['cache'])
            assert all(r['exit_code'] == 0 for r in data['commands'])
        else:
            assert data['cases'] == count
            if version == 722:
                assert data['valid_cases'] == 60 and not data['nonzero_exit_cases'] and not data['invalid_numeric_cases']
            elif version == 723:
                assert data['incomplete_cases'] == data['unexpected_result_shapes'] == 0
            elif version == 726:
                assert data['valid_samples'] == 672 and data['incomplete_cases'] == data['invalid_samples'] == 0
            else:
                assert data['numeric_samples'] == 4*count
                assert data['incomplete_cases'] == data['invalid_numeric_samples'] == data['typed_errors'] == 0
        replays[phase] = {'cases':count,'summary_sha256':sha(p/'summary.json')}
    for phase, count in [('typed-api-prototype-v694-normal-form-replay',384),('typed-api-prototype-v698-normal-form-budget-replay',384),('typed-api-prototype-v699-normal-form-algebraic-replay',288),('typed-api-prototype-v700-normal-form-algebraic-wide-replay',144)]:
        data = read(E/phase/'summary.json')
        assert data['source_hashes_sha256'] == EXPECTED and data['guards_unchanged'] and data['cases'] == count
        assert data['incomplete_cases'] == 0
        assert phase in fast['replays']
        replays[phase] = {'cases':count,'summary_sha256':sha(E/phase/'summary.json')}
    assert sum(r['cases'] for r in replays.values()) == 3745
    formatted = read(completed('typed-api-prototype-v706-included-fmt')/'command.json')['command']
    formatted = formatted[formatted.index('--check')+1:]
    original = json.loads(gzip.decompress((E/'candidate-preparation/source-hashes.json.gz').read_bytes()))
    assert formatted == sorted(n for n,v in source.items() if n.endswith('.rs') and original.get(n) != v)
    warnings = lambda p: [r for r in p.read_text().splitlines() if r.startswith('warning:')]
    inherited = warnings(completed('typed-api-prototype-v702-stack-tests')/'stderr.log')
    assert inherited == warnings(E/'typed-api-prototype-v553-stack-tests/stderr.log') and len(inherited) == 14
    for phase in ['typed-api-prototype-v692-normal-form-clippy','typed-api-prototype-v704-minimal-clippy']:
        assert not warnings(completed(phase)/'stderr.log')
    limits = fast['limits'] + [
        'The 78 private-kernel cases check result shapes. The earlier budget corpus numerically warms inputs where recorded; this is finite coverage, not exhaustive cutpoints or concurrent schedules.',
        'Main runtime remains unadmitted with the original failure. All 49 original worktree entries, 39 properties, 14 targets and crate future work remain preserved; no push was requested or performed.'
    ]
    followup = {
        'status':'Scoped source review and next work; no runtime admission', 'previous_review':'algebraic-budget-resource-followup-v578.json',
        'source_record':source_record, 'source_record_sha256':EXPECTED,
        'reviewed_source_slices':{
            n:{'sha256':source[n],'scope':scope} for n,scope in {
                'crates/hyperreal/src/computable/node/symbolic_work.rs':'Complete new cooperative work helper.',
                'crates/hyperreal/src/computable/node/normal_form_budget_tests.rs':'Complete eight-test file; fixtures and assertions retained through both accounting-path refactors.',
                'crates/hyperreal/src/computable/budget.rs':'Complete shared allowance implementation and updated unit documentation; no algorithm change here.',
                'crates/hyperreal/src/computable/node/algebra.rs':'RadicalCoefficient and PiLaurentPolynomial implementations, Laurent parser, rational projection and compaction; inverse-trig forms, inverse_trig_linear_sign, retained pi-multiple and canonical sine-term helpers; the add method inverse-trig collapse call and finite-domain guards. No whole-file audit.',
                'crates/hyperreal/src/computable/node/structural_analysis.rs':'Exact sign traversal/publication, quadratic-surd projections/sign, bounded rational visitor and prior algebraic dependency implementation. No whole-file audit.',
                'crates/hyperreal/src/computable/node/evaluation_budget.rs':'Complete public allowance wrappers and immediate proof reuse.',
                'crates/hyperreal/src/computable/node/approximation_queries.rs':'sign_until proof/refinement cascade; no wholesale comparator audit.'
            }.items()
        },
        'closed_scoped_gaps':implementation['changes']+[fast['change']],
        'next_candidates':[
            {'path':'crates/hyperreal/src/computable/node/algebra.rs','operations':['pi_atan_terms_form','pi_atan_linear_form','pi_acos_linear_form','inverse_trig_linear_sign'], 'finding':'These depth-bounded parsers, coefficient arithmetic and exact atom comparisons still do not debit shared work. exact_sign calls inverse_trig_linear_sign before its metered iterative walk. A retained finite-domain proof can avoid that entry check\'s traversal. The add constructor also runs linear collapse before independently requiring both finite-domain certificates; preserve those checks while completing the sign/constructor accounting map.'},
            {'path':'crates/hyperreal/src/computable/node/algebra.rs','operations':['retained_pi_rational_multiple','pi_laurent_rational_multiple','canonical_sin_pi_term'], 'finding':'Selected coefficient clones, arithmetic and recursive recognition remain unmetered outside the repaired Laurent/rational projections. Preserve cheap representation-limit rejection before any allocation debit.'},
            {'path':'crates/hyperreal/src/computable/node/structural_analysis.rs','operations':['AlgebraicSeparation::rational_root','Computable::exact_sign'], 'finding':'Algebraic leaf cloning still precedes dependency-size rejection. Ordinary negative sign memoization remains as before; broader cross-root warming and public concurrency schedules are not exhaustively covered by the eight new tests.'}
        ],
        'remaining_work':[
            'Continue shared accounting at reachable inverse-trig and retained pi-recognition helpers, with finite-domain and fallible-comparison rules preserved.',
            'Retain radical/quadratic-tower gaps and distinguish direct Real API reachability from Computable evaluation. Individual exact arithmetic operations still need separate time/allocation analysis.',
            'Continue kernel-specific interruption cutpoints, public concurrent/reconstructed and cross-root warming schedules, implementation-connected proofs, composed ln/exp/inverse-hyperbolic planning, native-prototype combination and unchanged full-board admission.'
        ], 'limits':limits
    }
    write('normal-form-resource-followup-v729.json',followup)
    write('normal-form-regression-coverage-v729.json',{
        'status':'Isolated normal-form repair validated; no runtime admission','source_record_sha256':EXPECTED,'changed_sources':changed,
        'contract_decision':'p4-contract-decision-v173.json','unit_coverage':implementation['unit_coverage'],'retained_failures':implementation['retained_failures'],
        'final_source_replays':replays,'all_prior_named_tests_unchanged':True,'all_preserved_probe_inputs_caps_and_arguments_unchanged':True,
        'cost_records':fast['cost_records'],'paired_cost':fast['paired_cost'],'limits':limits})
    write('typed-api-prototype-v729-metadata.json',{
        'previous_checkpoint':'typed-api-prototype-v578-state.json','implementation_commits':['e69370b45','6f60f9545',sys.argv[1]],'cost_baseline_commit':'4786a7f82',
        'implementation_records':['normal-form-budget-implementation-v657.json','normal-form-specialization-v700.json'],
        'status':'isolated Laurent and bounded-rational work accounting with retry-safe sign publication; no runtime admission',
        'contract_decision':'p4-contract-decision-v173.json','changes':followup['closed_scoped_gaps'],
        'retained_regression_record':'normal-form-regression-coverage-v729.json','retained_intermediate_failures':implementation['retained_failures'],
        'test_summaries':summaries,'capped_unit_regressions':caps,'final_source_replays':replays,'included_rust_fragments_checked':len(formatted),
        'formal_reuse_record':'normal-form-budget-proof-reuse-v729.json','formal_validation':proof,
        'cost_records':fast['cost_records'],'paired_cost':fast['paired_cost'],
        'inherited_warnings':{'identical_cargo_filename_warning_lines':inherited,'strict_clippy_warning_lines':0},
        'coverage_limits':limits,'remaining_source_review':'normal-form-resource-followup-v729.json','remaining_requirements':followup['remaining_work']+limits})
    print(json.dumps({'tests':summaries,'capped_tests':66,'replay_cases':3745,'formatted_rust_files':len(formatted),'changed_sources':len(changed)}))


if __name__ == '__main__':
    main()
