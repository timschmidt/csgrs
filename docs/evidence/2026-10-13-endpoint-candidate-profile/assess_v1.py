"""Verify the full-board profile and select a bounded operand-shape diagnostic."""
import gzip
import hashlib
import json
from pathlib import Path
import sys

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
OUT = E / 'profile-v1'
PARENT = ROOT / 'docs/evidence/2026-10-12-endpoint-coordinate-reuse'
SOURCE = ROOT / 'target/endpoint-coordinate-reuse/source'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources


def main():
    assert not (E / 'assessment-v1.json').exists()
    manifest = json.loads((OUT / 'manifest.json').read_text())
    protocol = json.loads((OUT / 'protocol.json').read_text())
    observed = json.loads((OUT / 'observations.json').read_text())
    attribution = json.loads((OUT / 'attribution.json').read_text())
    multiply = json.loads((OUT / 'multiply-costs.json').read_text())
    assert manifest['complete'] and manifest['record_exit_code'] == 101
    assert sha(Path(manifest['binary'])) == manifest['binary_sha256']
    assert sha(E / 'profile_v1.py') == protocol['runner_sha256']
    for name, digest in manifest['artifacts_sha256'].items():
        assert sha(OUT / name) == digest, name
    for result, script in [(observed, 'analyze_v1.py'), (attribution, 'attribute_v1.py'),
                           (multiply, 'multiply_costs_v1.py')]:
        assert sha(E / script) == result['runner_sha256']
    for command in json.loads((OUT / 'commands.json').read_text()):
        assert command['exit_code'] == (101 if command['name'] == 'record' else 0)
        for stream in ['stdout', 'stderr']:
            path = OUT / (command['name'] + '.' + stream + '.gz')
            raw = gzip.decompress(path.read_bytes())
            assert len(raw) == command[stream + '_bytes']
            assert hashlib.sha256(raw).hexdigest() == command[stream + '_sha256']
            assert sha(path) == command[stream + '_gzip_sha256']
    expected = json.loads(gzip.decompress((OUT / 'source-hashes.json.gz').read_bytes()))
    assert len(expected) == 10130 and observed['samples'] == 6786 and observed['lost_samples'] == 0
    assert all(sha(SOURCE / name) == digest for name, digest in expected.items())
    assert sources() == json.loads((PARENT / 'preparation.json').read_text())['main_source_sha256']
    pending = json.loads((ROOT / 'docs/evidence/2026-10-12-existing-worktree/geometry-validation-closure/result.json').read_text())['pending_native_sha256']
    assert len(pending) == 35 and all(sha(ROOT / name) == digest for name, digest in pending.items())
    reviews = []

    def review(path, start, end, finding):
        text = (SOURCE / path).read_text()
        begin = text.index(start)
        body = text[begin:text.index(end, begin)]
        reviews.append({'path': path, 'begin': start, 'end_before': end,
                        'body_sha256': hashlib.sha256(body.encode()).hexdigest(), 'finding': finding})

    algebra = 'crates/hyperreal/src/computable/node/algebra.rs'
    review(algebra, '    fn multiply(self, other: &Self)', '    fn inverse(self)',
           'Coefficient multiplication consumes the left coefficient, borrows the right, rejects unsupported opaque cross products, and builds new result vectors. Even a purely rational right coefficient follows this reconstruction path. Actual operand-shape frequencies are not established by the profile.')
    review(algebra, '    fn scale_in_place(', '    fn multiply(self, other: &Self)',
           'The existing scaling helper updates the rational part and radical/opaque coefficients in order and removes zero coefficients. Reusing this helper in multiplication is only a hypothesis: owner lifetimes, cache work, boundary behavior and costs still require qualification.')
    review(algebra, '    fn multiply(self, other: Self)', '    fn inverse_monomial(',
           'Polynomial multiplication already consumes each left coefficient directly when the right polynomial has one term; other cases clone it per right term. The proposed shape diagnostic does not alter that loop or term order.')
    review(algebra, '    fn project_pi_laurent_polynomial<T>(', '    fn pi_laurent_polynomial(',
           'A failed retained projection that reused a certificate drops its memo before one original recognition retry. This mandatory fallback remains unchanged; no negative cache or retry omission is proposed.')
    review(algebra, '    fn pi_laurent_polynomial_retained<const RETAIN: bool>(', '    pub(crate) fn bounded_laurent_rational(',
           'Shared compound nodes retain successful closed forms and per-pass memo results; expansion and recursion bounds remain. The diagnostic must preserve these operations and the original query sequence.')
    review('crates/hyperreal/src/computable/node/structural_analysis.rs',
           '    pub(crate) fn exact_quadratic_surd_parts(', '    #[cfg(test)]\n    pub(super) fn planning_msd(',
           'Quadratic recognition projects the shared normal form after cancellation. The profile attributes substantial work here, but does not show that an early restricted traversal would preserve its completeness.')
    merge = ROOT / 'docs/evidence/2026-10-12-laurent-coefficient-merge'
    narrow = ROOT / 'docs/evidence/2026-10-12-quadratic-certificate-consumer'
    prior = []
    for path, distinction in [
        (merge / 'native-assessment-v1.json', 'That rejected trial changed coefficient addition/merge ownership. The proposed diagnostic observes coefficient multiplication shapes and changes no arithmetic.'),
        (narrow / 'cost-assessment-v2.json', 'That rejected trial added an eight-node narrow traversal and cache publication before the normalizer. The proposed diagnostic adds no recognition pass, query, cache write or projection.'),
    ]:
        prior.append({'path': str(path.relative_to(ROOT)), 'sha256': sha(path),
                      'decision': json.loads(path.read_text())['decision'], 'distinction': distinction})
    next_step = (
        'In a separate copy of the frozen endpoint candidate, count the existing left/right coefficient shapes at RadicalCoefficient::multiply: '
        'rational-only, radical-only, opaque-only and mixed. Use fixed integer counters, inspect only existing vector presence/length, and retain no scalar owners. '
        'Keep every original multiplication, early rejection, insertion order, normal-form pass, cache publication and full-board fixture. '
        'Only if actual shape coverage supports it, evaluate consuming the owned left vectors for a purely rational right scale through the existing scaling helper. '
        'Do not inherit the rejected coefficient merge or eight-node prepass. Preserve the eight/nine-term cache frontier and mandatory original fallback, '
        'opaque ownership, abort/budget receipts, cold/warm/cloned facts, fixed scalar/native timing and memory controls, complete reports, and both original native limits. '
        'All prior endpoint timing tradeoffs and the broader properties/future-work plan remain open.')
    assessment = {
        'decision': 'diagnose-rational-scaling-shapes-inside-normal-form-multiplication',
        'assessor_sha256': sha(Path(__file__)),
        'scope': 'Fresh frame-pointer/line-table profile of the frozen uninstrumented endpoint candidate on both original complete boards.',
        'quality': {key: observed[key] for key in ['samples', 'total_sample_period', 'lost_samples',
                                                 'empty_stack_period_percent', 'unknown_leaf_period_percent']},
        'profiled_worker_cpu_seconds': manifest['profiled_worker_cpu_seconds'],
        'original_worker_cpu_limit_seconds': 120, 'profiled_gate_passed': False,
        'inclusive_groups': observed['groups'],
        'normal_form_callers': observed['nearest_recovered_non_scalar_normal_form_callers'][:20],
        'coefficient_multiplication': multiply,
        'allocator_leaf_subset_percent': attribution['allocator_leaf_total_percent'],
        'point_equality_callers': attribution['point_equality_nearest_recovered_project_callers'][:8],
        'source_review': reviews, 'prior_rejections': prior, 'next_step': next_step,
        'limits': [
            'Cycle-period percentages overlap and are not call counts, wall time or predicted savings. No inline frames were decoded.',
            'Zero recovered cache-read symbol weight is not evidence of zero cache cost; inlined work can appear under other symbols.',
            'The 619 samples containing coefficient multiplication establish sampled cost, not which operand shapes produced it.',
            'The earlier metadata-parent profile is a different build and run. Differences between its percentages and these do not establish a matched speedup.',
            'The independently recorded release gate remains failed at 138.312559051 / 120 seconds. This profiling result does not replace it.',
            'The 35 pending main runtime entries, all properties/targets/future-work obligations, and prior endpoint adverse cost rows remain in scope.',
        ],
        'artifacts_sha256': {str(p.relative_to(E)): sha(p) for p in sorted(OUT.iterdir()) if p.is_file()},
        'main_runtime_unchanged': True, 'runtime_admitted': False, 'plan_complete': False,
    }
    (E / 'assessment-v1.json').write_text(json.dumps(assessment, indent=2) + '\n')
    status_path = ROOT / 'docs/properties-status.json'
    status = json.loads(status_path.read_text())
    original_scope = {key: status[key] for key in ['properties', 'targets', 'future_work', 'next_actions']}
    assert len(status['properties']) == 39 and len(status['targets']) == 14
    work = status['current_work']
    work['updated_on'] = '2026-10-13'
    work['isolated_endpoint_candidate_profile'] = {
        'status': assessment['decision'], 'evidence': str((E / 'assessment-v1.json').relative_to(ROOT)),
        'result': '6786 samples, zero lost. Normal-form work has 27.404% inclusive sampled weight; coefficient multiplication has 9.227%, including a declared allocator-leaf subset at 1.947% of total profile weight. The profile does not identify operand shapes or predict savings.',
        'profiled_worker_cpu_seconds': manifest['profiled_worker_cpu_seconds'],
        'original_worker_cpu_limit_seconds': 120, 'next_step': next_step, 'runtime_admitted': False}
    for key in ['pending_native_geometry', 'isolated_endpoint_coordinate_reuse', 'isolated_endpoint_cost_attribution']:
        work[key]['next_step'] = next_step
    assert all(status[key] == value for key, value in original_scope.items())
    assert work['plan_complete'] is False
    status_path.write_text(json.dumps(status, indent=2) + '\n')
    print(json.dumps({'decision': assessment['decision'], 'samples': observed['samples'],
                      'profiled_worker_cpu_seconds': manifest['profiled_worker_cpu_seconds']}))


if __name__ == '__main__':
    main()
