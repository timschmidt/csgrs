"""Retain the observed coverage and preregister separate scaling variants."""
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    assert not (E / 'decision-v1.json').exists()
    observation = json.loads((E / 'assessment-v1.json').read_text())
    rows = observation['rows']
    total = observation['total_multiplication_calls']
    left_scale_vectors = sum(row['calls'] for row in rows if row['left'] == 'rational' and row['right'] != 'rational')
    right_scale = observation['right_rational_calls']
    decision = {
        'decision': 'advance-two-isolated-scaling-variants-with-separate-cost-results',
        'runner_sha256': sha(Path(__file__)), 'assessment_sha256': sha(E / 'assessment-v1.json'),
        'total_calls': total, 'right_rational_calls': right_scale,
        'right_rational_with_owned_left_vectors_calls': observation['right_rational_with_owned_left_vectors_calls'],
        'left_rational_with_borrowed_right_vectors_calls': left_scale_vectors,
        'left_rational_with_borrowed_right_vectors_percent': 100 * left_scale_vectors / total,
        'either_scaling_direction_calls': right_scale + left_scale_vectors,
        'either_scaling_direction_percent': 100 * (right_scale + left_scale_vectors) / total,
        'variants': {
            'right': 'When the right coefficient has no radical or opaque vectors, consume the owned left coefficient through the existing scale_in_place helper; preserve the original general multiplication body otherwise.',
            'both': 'Include the right-scalar branch; additionally, when only the left coefficient is rational, clone the borrowed right coefficient once and scale its vectors through the existing helper.',
        },
        'why_two_variants': 'Right scaling can retain already owned vectors; left scaling still clones vectors and can change operand/cache work. Their costs must remain separately visible, including the more frequent unmodified shapes.',
        'required_qualification': [
            'Fork both variants from the frozen uninstrumented endpoint candidate into separate source and Cargo targets; never inherit diagnostic counters.',
            'Exact component/zero/negative-scale cases in both directions; opaque owner release and unchanged unsupported products; existing eight/nine-term cache frontiers and cold/warm/cloned facts.',
            'Full scalar library/doctests, strict Clippy, formatting and warning-denied docs before scalar cost admission.',
            'Three original scalar probe suites, time and requested-allocation modes, construction/first/warm/cloned/opaque and mixed-field declines, five fixed sample blocks with all variants and separate targets. No extension to remove adverse rows.',
            'Only supported candidates advance to native consumer qualification, original 120-second worker and separate 60-second copper-neck assertions, broad endpoint controls and complete unnormalized report identity.',
        ],
        'limits': 'Operand counts are not time shares or guaranteed allocation savings. The instrumented test still fails at 134.127171383 worker-CPU seconds / 120. Original release and endpoint cost tradeoffs remain unchanged; this decision admits experiments only.',
        'runtime_admitted': False, 'plan_complete': False,
    }
    assert left_scale_vectors == 8594476
    (E / 'decision-v1.json').write_text(json.dumps(decision, indent=2) + '\n')
    status_path = ROOT / 'docs/properties-status.json'
    status = json.loads(status_path.read_text())
    status['current_work']['updated_on'] = '2026-10-13'
    status['current_work']['isolated_coefficient_shapes'] = {
        'status': decision['decision'], 'evidence': str((E / 'decision-v1.json').relative_to(ROOT)),
        'result': '21954159 observed multiplications. Right-rational calls: 3067533 (13.972%); 1348450 have owned left vectors (6.142% of all calls). Rational-left with borrowed right vectors occurs 8594476 times. No scalar queries or owners were added by the counters.',
        'qualification': observation['tests'], 'instrumented_worker_cpu_seconds': observation['instrumented_worker_cpu_seconds'],
        'original_worker_cpu_limit_seconds': 120, 'next_step': decision['variants'], 'runtime_admitted': False}
    next_step = 'Evaluate the preregistered right-scalar and bidirectional coefficient-scaling variants separately, using the frozen uninstrumented endpoint parent. Preserve original arithmetic fallbacks and ownership guarantees; complete scalar correctness and fixed three-suite cost controls before native qualification. Full requirements: ' + str((E / 'decision-v1.json').relative_to(ROOT))
    for key in ['pending_native_geometry', 'isolated_endpoint_coordinate_reuse', 'isolated_endpoint_candidate_profile']:
        status['current_work'][key]['next_step'] = next_step
    assert len(status['properties']) == 39 and len(status['targets']) == 14
    assert status['current_work']['plan_complete'] is False
    status_path.write_text(json.dumps(status, indent=2) + '\n')
    print(json.dumps({key: decision[key] for key in ['decision', 'left_rational_with_borrowed_right_vectors_calls', 'either_scaling_direction_percent']}))


if __name__ == '__main__':
    main()
