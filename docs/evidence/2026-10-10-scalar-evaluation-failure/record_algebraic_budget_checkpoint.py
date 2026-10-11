#!/usr/bin/env python3
"""Record the final algebraic-budget candidate without broadening proof or admission claims."""
import gzip
import hashlib
import json
from pathlib import Path
import re
import sys

sys.dont_write_bytecode = True
from record_evaluation_budget_checkpoint import case_commands, command_contract
from parse_test_results_v2 import parse_phase

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / 'target/scalar-evaluation-candidate/source'
SOURCE_RECORD = 'typed-api-prototype-v550-budget-verus/source-hashes.json.gz'
EXPECTED = 'd6527953cefc76fb9c5f914434459b74e5949b08a22c755a5b83684a1b4b7deb'
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
    assert len(sys.argv) == 2, 'fast-path implementation commit required'
    prior = read(E / 'typed-api-prototype-v504-state.json')
    implementation = read(E / 'algebraic-budget-implementation-v517.json')
    fast = read(E / 'algebraic-budget-fast-path-v525.json')
    assert fast['source_record_sha256'] == EXPECTED
    orchestration = read(E / 'algebraic-budget-orchestration-v578.json')
    assert orchestration['exit_code'] == 0 and orchestration['source_record_sha256'] == EXPECTED
    assert orchestration['helper_sha256'] == sha(E / 'validate_algebraic_budget_checkpoint_v2.py')
    assert orchestration['log_sha256'] == sha(E / 'algebraic-budget-orchestration-v578.log')
    proof = read(E / 'algebraic-budget-proof-reuse-v578.json')
    assert proof['source_record_sha256'] == EXPECTED
    assert proof['verified_units'] == 638 and proof['errors'] == 0
    assert len(proof['proof_and_guard_inputs_sha256']) == 44 and proof['reused_rejected_mutants'] == 213
    assert proof['fresh_acceptance_tests'] == 10 and proof['open_obligation_groups'] == 13
    sources = json.loads(gzip.decompress((E / SOURCE_RECORD).read_bytes()))
    before = json.loads(gzip.decompress((E / prior['source_record']).read_bytes()))
    assert set(before) <= set(sources)
    changed = {k: {'before': before.get(k), 'after': v} for k, v in sources.items() if before.get(k) != v}
    assert set(changed) == set(implementation['changed_sources']) and len(changed) == 4
    assert all(before[n] == sources[n] for n in before if 'test' in n)
    assert all(sha(SOURCE / n) == v for n, v in sources.items())
    summaries = {}
    for key, version, suffix in [('debug_scalar', 552, 'debug-scalar-unit-tests'), ('five_crate', 553, 'stack-tests'), ('default_scalar', 554, 'default-tests')]:
        phase = f'typed-api-prototype-v{version}-{suffix}'
        actual = parse_phase(completed(phase))
        record = f'typed-api-prototype-v{version}-test-summary.json'
        assert actual == read(E / record)
        old = prior['test_summaries'][key]
        assert actual['top_level_passed'] == old['passed'] + 4
        assert actual['child_process_passed'] == old['child_passed'] and actual['top_level_ignored'] == old['ignored']
        assert actual['top_level_failed'] == actual['child_process_failed'] == 0
        summaries[key] = {'record': record, 'passed': actual['top_level_passed'], 'child_passed': actual['child_process_passed'], 'ignored': actual['top_level_ignored']}
    caps = {}
    for version, suffix, count in [(558, 'private-kernel', 7), (559, 'magnitude', 5), (560, 'publication', 7), (561, 'fractional', 5), (562, 'scaled-lower', 1), (563, 'trig-range', 5), (564, 'trig-lattice', 5), (565, 'trig-residual', 6), (566, 'trig-operand', 6), (567, 'evaluation-budget', 7), (568, 'algebraic-budget', 4)]:
        phase = f'typed-api-prototype-v{version}-{suffix}-cap'
        p = completed(phase)
        log = (p / 'stdout.log').read_text()
        receipt = json.loads(log.splitlines()[-1])
        assert receipt['exit_code'] == 0 and not receipt['wall_timeout']
        assert re.findall(r'^test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;', log, re.M) == [(str(count), '0', '0')]
        assert receipt['address_space_bytes'] == 192 * 1024 * 1024 and receipt['cpu_seconds_soft_hard'] == [2, 3] and receipt['wall_seconds_limit'] == 4
        caps[phase] = receipt | {'passed': count, 'phase_result': read(p / 'result.json')}
    assert sum(r['passed'] for r in caps.values()) == 58
    replays = {}
    for version, old_version, suffix, count in [(569,495,'replay',415), (570,496,'precision-replay',60), (571,497,'private-kernel-replay',78), (572,498,'private-trig-replay',600), (573,499,'private-trig-wide-replay',384), (574,500,'symbolic-trig-scale-replay',168), (575,501,'symbolic-trig-residual-replay',456), (576,502,'symbolic-trig-nonrational-replay',384), (577,503,'evaluation-budget-replay',384)]:
        phase = f'typed-api-prototype-v{version}-{suffix}'
        p, old = E / phase, E / f'typed-api-prototype-v{old_version}-{suffix}'
        data = read(p / 'summary.json')
        assert data['source_hashes_sha256'] == EXPECTED and data['guards_unchanged']
        for name in ['probe.rs','candidate_probe.rs','cache_candidate_probe.rs','reproduce.py','oracles.json']:
            if (old / name).is_file():
                assert sha(old / name) == sha(p / name), (phase,name)
        assert [command_contract(r) for r in case_commands(old)] == [command_contract(r) for r in case_commands(p)]
        if version == 569:
            assert data['ordinary_cases'] == 400 and data['typed_interruption_errors'] == 300
            assert data['invalid_first_contracts'] == data['invalid_retries'] == 0
            assert len(data['extremes']) == 6 and all(r['valid'] for r in data['extremes'])
            assert len(data['cache']) == 9 and all(r['valid'] for r in data['cache'])
            assert all(r['exit_code'] == 0 for r in data['commands'])
        else:
            assert data['cases'] == count
            if version == 570:
                assert data['valid_cases'] == 60 and not data['nonzero_exit_cases'] and not data['invalid_numeric_cases']
            elif version == 571:
                assert data['incomplete_cases'] == data['unexpected_result_shapes'] == 0
            elif version == 574:
                assert data['valid_samples'] == 672 and data['incomplete_cases'] == data['invalid_samples'] == 0
            else:
                assert data['numeric_samples'] == 4 * count
                assert data['incomplete_cases'] == data['invalid_numeric_samples'] == data['typed_errors'] == 0
                if version == 577:
                    assert data['warm_zero_budget_checks'] == 384
        replays[phase] = {'cases':count, 'summary_sha256':sha(p/'summary.json')}
    phase = 'typed-api-prototype-v523-algebraic-budget-replay'
    data = read(E / phase / 'summary.json')
    assert data['source_hashes_sha256'] == EXPECTED and data['guards_unchanged']
    assert data['cases'] == data['zero_budget_retained_proofs'] == 288 and data['numeric_observations'] == 576
    assert data['incomplete_cases'] == data['invalid_samples'] == 0
    replays[phase] = {'cases':288, 'summary_sha256':sha(E/phase/'summary.json')}
    assert sum(r['cases'] for r in replays.values()) == 3217
    formatted = read(completed('typed-api-prototype-v557-included-fmt')/'command.json')['command']
    formatted = formatted[formatted.index('--check')+1:]
    original = json.loads(gzip.decompress((E/'candidate-preparation/source-hashes.json.gz').read_bytes()))
    assert formatted == sorted(n for n,v in sources.items() if n.endswith('.rs') and original.get(n) != v)
    warnings = lambda p: [line for line in p.read_text().splitlines() if line.startswith('warning:')]
    inherited = warnings(completed('typed-api-prototype-v553-stack-tests')/'stderr.log')
    assert inherited == warnings(E/'typed-api-prototype-v479-stack-tests/stderr.log') and len(inherited)==14
    for phase in ['typed-api-prototype-v521-stack-clippy', 'typed-api-prototype-v555-minimal-clippy']:
        assert not warnings(completed(phase)/'stderr.log')
    for record in fast['cost_records']:
        c = read(E/record)
        assert c['source_record_sha256'] == EXPECTED and c['probe_and_harness_identical'] and c['inputs_order_caps_checksums_and_iterations_preserved']
        for which in ['baseline','candidate']:
            assert sha(E/c[which+'_phase']/'summary.json') == c[which+'_summary_sha256']
    limits = fast['limits'] + [
        'The new allowance sweep covers one exact root-product metadata identity; the 180 public schedules and 288 external cases are finite coverage, not every kernel cutpoint or public concurrency schedule.',
        'The 78 private-kernel cases check result shapes, not full numeric accuracy. The earlier 384 budget cases use numeric warmup/cache reuse as recorded in v504. No full-board, allocation or binary-size claim follows from local micro-costs.',
        'Main runtime remains unadmitted and retains the original failure. The 49 original worktree entries, all 39 properties, 14 targets and crate future work remain preserved. No push was requested or performed.'
    ]
    followup = {
        'status':'Scoped source review and next work; no runtime admission',
        'previous_review':'evaluation-budget-resource-followup-v504.json', 'source_record':SOURCE_RECORD, 'source_record_sha256':EXPECTED,
        'reviewed_source_slices': {
            'crates/hyperreal/src/computable/budget.rs': {'sha256':sources['crates/hyperreal/src/computable/budget.rs'], 'scope':'Complete shared allowance/TLS implementation, with new private active-scope query and work-unit documentation.'},
            'crates/hyperreal/src/computable/node/structural_analysis.rs': {'sha256':sources['crates/hyperreal/src/computable/node/structural_analysis.rs'], 'scope':'AlgebraicSeparation implementation, metadata parser, exact_quadratic_surd_sign/parts, exact_rational and bounded_rational_value; no whole-file audit.'},
            'crates/hyperreal/src/computable/node/algebraic_budget_tests.rs': {'sha256':sources['crates/hyperreal/src/computable/node/algebraic_budget_tests.rs'], 'scope':'Complete four-test file; metadata sweep, wide dependency charges, deep generator comparison and 180 public sign/cache schedules.'},
            'crates/hyperreal/src/computable/node/approximation_queries.rs': {'sha256':sources['crates/hyperreal/src/computable/node/approximation_queries.rs'], 'scope':'Public sign_until path through algebraic metadata and retained proof publication; surrounding public comparator/cache APIs not re-audited wholesale.'},
            'crates/hyperreal/src/computable/node/algebra.rs': {'sha256':sources['crates/hyperreal/src/computable/node/algebra.rs'], 'scope':'RadicalCoefficient setup, insert/add/scale and polynomial parser through bounded_laurent_rational; other methods located by search are not credited as read.'},
        },
        'source_level_argument':fast['source_level_argument'],
        'closed_scoped_gap':'Algebraic separation metadata parsing, generator comparison, dependency combinations and selected rational coefficient operations now share the active allowance; stopped searches cannot append a generator based on interrupted equality/dependency reasoning.',
        'next_candidates':[
            {'path':'crates/hyperreal/src/computable/node/algebra.rs', 'operations':['Computable::pi_laurent_polynomial_with_memo','RadicalCoefficient::insert_atom','RadicalCoefficient::add','RadicalCoefficient::scale_in_place'], 'finding':'The local normal-form node limit does not debit the active allowance. Opaque-atom equality remains exact/unmetered, and coefficient arithmetic plus retained square reduction can do work inside one visit. exact_sign reaches this path through exact_quadratic_surd_sign; this is directly relevant to the public budgeted sign API.'},
            {'path':'crates/hyperreal/src/computable/node/structural_analysis.rs', 'operations':['Computable::bounded_rational_value::visit','Computable::exact_quadratic_surd_sign'], 'finding':'The rational recognizer has local depth 32/work 64/coefficient 4096-bit guards, but only its square-root-radicand helper currently charges shared visits. Generic recursion and arithmetic remain unmetered. Quadratic-surd sign squares/orderings need coefficient-size accounting.'},
            {'path':'crates/hyperreal/src/computable/node/structural_analysis.rs', 'operations':['AlgebraicSeparation::rational_root','AlgebraicSeparation::charge_coefficient_work'], 'finding':'Exact leaf cloning still precedes dependency bit-limit rejection. The new coefficient charge is a cooperative weight; it does not interrupt one GMP/BigInt operation or bound all temporary bytes, duration or prior leaf allocations.'},
        ],
        'remaining_work':[
            'Prioritize public-budget paths in Laurent and bounded rational recognition. Preserve the original domain checks, use fallible comparisons, and never publish resource-interrupted negative caches or weaker facts.',
            'Retain radical/quadratic tower arithmetic gaps from v504. Their Real API reachability differs from this Computable sign path; complete the call-graph/accounting map before broadening claims.',
            'Continue kernel-specific budget cutpoints, reconstructed/public concurrent schedules, implementation-connected proofs, composed ln/exp/inverse-hyperbolic planning, full allocation/time accounting, native-prototype combination and full-board admission.'
        ], 'limits':limits,
    }
    coverage = {'status':'Isolated algebraic-budget repair validated; no runtime admission', 'source_record_sha256':EXPECTED, 'changed_sources':changed,
                'contract_decision':'p4-contract-decision-v173.json', 'unit_coverage':implementation['unit_coverage'], 'replay':fast['replay'],
                'all_prior_named_test_sources_unchanged':True, 'all_2929_prior_replay_fixtures_helpers_arguments_order_and_caps_preserved':True,
                'initial_regression':implementation['retained_failures'], 'initial_timing_record':'algebraic-budget-cost-comparison-v517.json',
                'fast_path_record':'algebraic-budget-fast-path-v525.json', 'cost_records':fast['cost_records'], 'limits':limits}
    metadata = {'previous_checkpoint':'typed-api-prototype-v504-state.json', 'implementation_commits':['6cd8b3990',sys.argv[1]], 'cost_baseline_commit':'ee49f52aa',
                'implementation_records':['algebraic-budget-implementation-v517.json','algebraic-budget-fast-path-v525.json'],
                'status':'isolated algebraic metadata and dependency work accounting; no runtime admission', 'contract_decision':'p4-contract-decision-v173.json',
                'changes':implementation['changes']+[fast['change']], 'retained_regression_record':'algebraic-budget-regression-coverage-v578.json',
                'retained_intermediate_failures':implementation['retained_failures'], 'test_summaries':summaries, 'capped_unit_regressions':caps, 'final_source_replays':replays,
                'new_matrix_coverage':implementation['unit_coverage'], 'external_algebraic_replay':fast['replay'], 'included_rust_fragments_checked':len(formatted),
                'formal_reuse_record':'algebraic-budget-proof-reuse-v578.json', 'formal_validation':{k:proof[k] for k in ['fresh_verification_phase','fresh_acceptance_phase','verified_units','errors','listed_production_contracts','listed_model_theorems','open_obligation_groups','reused_rejection_phase','reused_rejected_mutants','fresh_acceptance_tests','limits']},
                'cost_records':fast['cost_records'], 'inherited_warnings':{'identical_cargo_filename_warning_lines':inherited,'strict_clippy_warning_lines':0},
                'coverage_limits':limits, 'remaining_source_review':'algebraic-budget-resource-followup-v578.json', 'remaining_requirements':followup['remaining_work']+limits}
    write('algebraic-budget-resource-followup-v578.json', followup)
    write('algebraic-budget-regression-coverage-v578.json', coverage)
    write('typed-api-prototype-v578-metadata.json', metadata)
    print(json.dumps({'tests':summaries,'capped_tests':58,'replay_cases':3217,'formatted_rust_files':len(formatted),'changed_sources':len(changed)}))


if __name__ == '__main__':
    main()
