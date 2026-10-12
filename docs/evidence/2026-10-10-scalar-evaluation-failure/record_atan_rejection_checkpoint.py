"""Close the rejected atan experiments without narrowing the full properties plan."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
previous = E/'typed-api-prototype-v1500-state.json'
assert sha(previous) == 'adadeb0ee94ced7edb7ec9757a84b417a7e0cfb75441cdfa47730a4a7bf89d56'
prior = read(previous)
validation = read(E/'atan-restoration-validation-v1576.json')
assert validation['source_record_sha256'] == prior['source_record_sha256']
assert validation['restored_libraries_match_retained']
assert read(E/'atan-ledger-update-v1576.json')['after_sha256'] == sha(ROOT/'docs/properties-status.json')
metadata = {
    'previous_checkpoint': previous.name,
    'status': 'Both atan zero-term optimizations rejected; retained v1404/v1500 source restored exactly; no runtime admission',
    'contract_decision': 'p4-contract-decision-v173.json',
    'accepted_cache_rule': 'Warming may strengthen Unknown into Certified and reuse higher-precision proofs at zero work; no contradiction or weakening of an available certificate.',
    'evidence_commits': ['71c245225', 'b57a0546d', 'bb2ace2ab', '8c70eab73'],
    'retained_runtime_checkpoint': 'typed-api-prototype-v1404-state.json',
    'retained_runtime_source_sha256': prior['source_record_sha256'],
    'baseline_record': 'atan-zero-term-baseline-v1515.json',
    'rejected_experiments': ['atan-zero-term-implementation-v1540.json', 'atan-magnitude-guard-rejection-v1541.json',
                             'atan-quotient-implementation-v1568.json', 'atan-quotient-rejection-v1571.json'],
    'restoration_record': 'atan-source-restoration-v1572.json',
    'restoration_validation_record': 'atan-restoration-validation-v1576.json',
    'fresh_scalar_test_summary': validation['tests'],
    'fresh_included_rustfmt_files': validation['included_rust_files'],
    'restored_release_libraries_match_retained': True,
    'measurements': {
        'survey_schedules': 320, 'allocation_rows': 24, 'independent_rational_enclosures': 164,
        'new_test_functions_archived': 9, 'baseline_failed_allocation_guards': 2,
        'scalar_units_per_candidate': 1063, 'child_tests_per_candidate': 1,
        'baseline_targeted_cost_processes': 108, 'original_cost_processes_per_candidate': 468,
        'magnitude_guard_paired_processes': 936, 'magnitude_guard_controlled_processes': 936,
        'quotient_guard_controlled_processes': 936, 'quotient_guard_focused_controlled_processes': 216,
        'all_cost_processes_including_baseline': 4068, 'separate_fixed_cpu_preparation_processes': 2088,
        'million_bit_direct_rust_bytes_baseline_magnitude_quotient': [4801400, 16, 24],
        'magnitude_guard_cached_reciprocal_percent_change': [7.560844936298072, 9.823076028790357],
        'quotient_guard_wide_coarse_integer_percent_change': [9.448756500209887, 11.176297915018885],
    },
    'decision': 'Neither scoped allocation shortcut passes the performance gate. Preserve every trial and restore the retained source; no causal explanation for the unrelated-control cost increases is established.',
    'reused_prior_validation': {
        'checkpoint': 'typed-api-prototype-v1404-state.json',
        'checkpoint_sha256': sha(E/'typed-api-prototype-v1404-state.json'), 'source_identical': True,
        'scope': 'The prior complete stack/default/debug/Clippy/format/cap/replay/proof evidence applies to identical restored source bytes. Only scalar units, the release build and formatting were freshly rerun for this restoration.'
    },
    'coverage_limits': [
        'EvaluationBudget counts cooperative work units and excludes per-operation byte/time bounds. The measured allocations are optimization opportunities, not demonstrated budget or cache-value contract violations.',
        'Public rational atan uses AtanRational. Raw IntegralAtan and AtanDeferred integer cases are private diagnostics; public opaque atan is separately measured.',
        'All 160 coarse warmups in the surveys, including privately constructed diagnostic roots, complete as Ok(0) through the public query API. Successful zero-work/unlimited cache-hit retries are checked; exhausted uncached queries are not claimed to have independent unlimited retries.',
        'Fixtures, warmups and oracle construction are excluded from allocation/timing windows. Rust allocation counters exclude native allocation, RSS and peak-live memory.',
        'The quotient guard adds one eight-byte allocation in 140 measured rows relative to the magnitude guard; all 508 structured non-allocation observations agree.',
        'The quotient guard reverses the order of independent quotient/square calculations for nonzero terms. Existing dispatch/domain/precision/endpoint guards and work charges are unchanged.',
        'The first variant has original and fixed-CPU alternating-order comparisons. The second uses the same fixed-CPU control directly, then a predeclared four-fixture, 27-repetition follow-up. All original measurements remain retained.',
        'The prepared unconditioned quotient comparison v1562 was never executed. No result is claimed for it.',
        'Neither rejected candidate received a fresh broad-stack/replay/formal-proof run. Conditional Verus does not prove these inverse-trig kernels; all thirteen whole-crate proof groups remain open.',
        'All earlier retained tests and full-size hard cases remain unchanged; all nine added test bodies and both variants remain archived.',
        'Main runtime retains the original scalar failure and all 49 original worktree entries. No push or runtime admission occurred. All 39 properties, 14 targets and crate future work remain in scope.'
    ],
    'remaining_source_review': 'integer-scale-deferred-review-v1404.json',
    'remaining_requirements': prior['remaining_requirements'],
    'next_measurements': [
        'Measured deferred atan construction: establish a public path to the raw-integer double-copy observation before proposing a constructor repair.',
        'Log-range fallback: distinguish necessary operand sampling from avoidable constructors and establish explicit public EvaluationBudget reachability.',
        'Retained quadratic towers: establish public proof warming before measuring later fine-cache misses and coefficient copies.',
        'Further acos/atan allocation shortcuts must address retained cost regressions while preserving all frozen fixtures and outputs; do not infer causality from code layout alone.',
        'Continue general kernel/cache/concurrency, implementation-connected proof and unchanged native/full-board admission requirements.'
    ]
}
path = E/'typed-api-prototype-v1576-metadata.json'; assert not path.exists()
path.write_text(json.dumps(metadata, indent=2)+'\n')
subprocess.run([sys.executable, str(E/'record_checkpoint_v4.py'), path.name, 'typed-api-prototype-v1576-state.json',
    'typed-api-prototype-v1573-restored-build', *validation['phases']], cwd=ROOT, check=True)
state = read(E/'typed-api-prototype-v1576-state.json')
assert state['changed_since_previous_checkpoint'] == {}
assert state['source_files'] == 24743 and state['archived_changed_sources_verified'] == 120
print(json.dumps({'checkpoint': 'typed-api-prototype-v1576-state.json',
    'sha256': sha(E/'typed-api-prototype-v1576-state.json'), 'artifacts': len(state['artifacts_sha256']),
    'restored_source_identical': True}), flush=True)
