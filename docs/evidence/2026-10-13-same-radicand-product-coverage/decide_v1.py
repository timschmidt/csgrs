"""Use observed product coverage to scope a distinct, still unprepared kernel."""
import hashlib
import json
from pathlib import Path

from diagnose_v2 import verify

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    destination = E / 'decision-v1.json'
    assert not destination.exists()
    preparation = json.loads((E / 'preparation-v2/protocol.json').read_text())
    protocol = json.loads((E / 'diagnostic-v2/protocol.json').read_text())
    binary = json.loads((E / 'diagnostic-v2/binary.json').read_text())
    verify(preparation, protocol, binary)
    assessment = json.loads((E / 'assessment-v2.json').read_text())
    review = json.loads((E / 'next-kernel-review-v1.json').read_text())
    assert assessment['format_strict_clippy_and_docs_passed']
    assert assessment['source_parent_main_and_binary_unchanged']
    for name, digest in assessment['artifacts_sha256'].items():
        assert sha(E / name) == digest, name
    for row in review['source_files']:
        assert sha(ROOT / row['path']) == row['sha256']
    counts = assessment['summary']
    assert counts['coefficient_multiplications'] == 21954159
    assert counts['single_radical_pairs_without_atoms'] == 6986607
    assert counts['same_radicand_pairs'] == 6379817
    assert counts['same_radicand_outputs_without_radicals'] == 3778231
    assert counts['same_radicand_outputs_with_one_radical'] == 2601586
    assert not review['runtime_candidate_prepared']
    decision = {
        'status': 'prepare-distinct-direct-same-radicand-product-experiment',
        'decision_runner_sha256': sha(Path(__file__)),
        'inputs_sha256': {name: sha(E / name) for name in [
            'preparation-v2/protocol.json', 'assessment-v2.json',
            'diagnostic-v2/result.json', 'next-kernel-review-v1.json',
        ]},
        'summary': counts,
        'rationale': 'The complete boards execute 6,379,817 one-radical/no-atom products with equal radicands, about 29.06 percent of all coefficient multiplications. The unchanged operation leaves either zero or one radical in every observed equal-radicand case. This repeated coverage and the reviewed local arithmetic justify a distinct direct-kernel experiment. They do not establish its non-unit eligibility count, allocation savings, time share or a speedup.',
        'qualification': {
            'tests': assessment['tests'],
            'format_strict_four_crate_clippy_and_warning_denied_scalar_docs_passed': True,
            'qualification_commands_passed': 11,
            'counts_overlap': True,
            'full_workflow_matrix_covered': False,
        },
        'instrumented_gate': {
            'passed': assessment['original_gate_passed'],
            'worker_cpu_seconds': assessment['instrumented_worker_cpu_seconds'],
            'original_worker_cpu_limit_seconds': 120,
            'finding_counts': [12, 197],
            'complete_report_identity_established': False,
            'performance_comparison': False,
        },
        'next_experiment': {
            'status': 'reviewed-not-prepared',
            'baseline': 'Unchanged endpoint/P4 parent with the previously shared regression controls; no diagnostic counters or rejected runtime changes.',
            'runtime_scope': 'A private direct path for exactly one equal non-unit radical per coefficient after the original unsupported opaque-product guard. Preserve the original general fallback and all other normalizer/cache/retry behavior.',
            'source_review': 'next-kernel-review-v1.json',
            'requirements': [
                'Preserve arithmetic operand order and zero-skipping behavior; retain the same left/right radicand payload selected by ordinary insertion.',
                'Review the lifetime of inputs and result payloads before reusing the consumed vector. Do not infer numeric-cache equivalence from a polynomial identity.',
                'Cover independent exact products, original-general differential results, cancellation, payload/buffer ownership, unreduced/wide rationals, unit-radicand fallback, radical/term/power frontiers, zero budgets, opaque release and mandatory original retry.',
                'Qualify shared tests on both frozen variants, scalar default/all-feature libraries and doctests, formatting, strict Clippy and warning-denied docs before measuring.',
                'Complete fresh fixed scalar costs first. Native correctness, original fixture gates, matched costs/receipts, complete reports, production size and remaining workflow obligations stay separate admission requirements.',
            ],
            'fixed_scalar_costs_if_qualified': {
                'suites': ['rational', 'quadratic', 'integer'],
                'modes': ['time', 'memory'],
                'blocks': [['baseline', 'candidate'], ['candidate', 'baseline'],
                           ['baseline', 'candidate'], ['candidate', 'baseline'],
                           ['baseline', 'candidate']],
                'measurement_processes': 60, 'preflight_processes': 12,
                'timing_inputs_per_phase': 4096, 'allocation_inputs_per_phase': 1000,
                'all_builds_before_probes': True, 'sample_extension_planned': False,
            },
            'candidate_prepared': False,
        },
        'limits': assessment['limits'] + review['risks'] + [
            'The diagnostic counts vector shapes and the original equality branches. It does not independently test non-unit or square-reduced radicands and does not estimate how many allocations a future implementation removes.',
            'The instrumented board assertion fails at 139.129132266 worker-CPU seconds against 120. It must not be compared with historical uninstrumented or differently instrumented durations as a matched benchmark.',
            'No new runtime source is admitted, no closed sample set is extended and no previous rejection is reversed. The original native changes and all 39 properties, 14 targets and crate future work remain open as previously recorded.',
        ],
        'source_parent_main_and_pending_native_unchanged': True,
        'runtime_admitted': False, 'plan_complete': False,
    }
    assert not decision['instrumented_gate']['passed']
    assert decision['instrumented_gate']['worker_cpu_seconds'] == 139.129132266
    destination.write_text(json.dumps(decision, indent=2) + '\n')
    print(json.dumps({'status': decision['status'], 'summary': counts,
                      'candidate_prepared': False, 'runtime_admitted': False}))


if __name__ == '__main__':
    main()
