"""Verify the fresh profile and select a diagnostic distinct from rejected trials."""
import gzip
import hashlib
import json
from pathlib import Path
import sys

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
OUT = E / 'profile-v1'
SOURCE = ROOT / 'target/region-metadata-first/source'
PARENT = ROOT / 'docs/evidence/2026-10-12-region-metadata-first'
HISTORY = ROOT / 'docs/evidence/2026-10-09-hyperdrc-native-linear-strokes'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources


def main():
    manifest = json.loads((OUT / 'manifest.json').read_text())
    protocol = json.loads((OUT / 'protocol.json').read_text())
    observed = json.loads((OUT / 'observations.json').read_text())
    attribution = json.loads((OUT / 'attribution.json').read_text())
    subcurve = json.loads((OUT / 'subcurve-callers.json').read_text())
    assert manifest['complete'] and manifest['record_exit_code'] == 101
    assert sha(Path(manifest['binary'])) == manifest['binary_sha256']
    assert sha(E / 'profile_v1.py') == protocol['runner_sha256']
    for name, digest in manifest['artifacts_sha256'].items():
        assert sha(OUT / name) == digest
    for command in json.loads((OUT / 'commands.json').read_text()):
        assert command['exit_code'] == (101 if command['name'] == 'record' else 0)
        for stream in ['stdout', 'stderr']:
            path = OUT / (command['name'] + '.' + stream + '.gz')
            raw = gzip.decompress(path.read_bytes())
            assert len(raw) == command[stream + '_bytes']
            assert hashlib.sha256(raw).hexdigest() == command[stream + '_sha256']
            assert sha(path) == command[stream + '_gzip_sha256']
    expected = json.loads(gzip.decompress((OUT / 'source-hashes.json.gz').read_bytes()))
    assert len(expected) == 10129 and observed['samples'] == 7326 and observed['lost_samples'] == 0
    assert all(sha(SOURCE / name) == digest for name, digest in expected.items())
    assert sources() == json.loads((PARENT / 'preparation.json').read_text())['main_source_sha256']
    reviews = []

    def body(path, begin, end, finding):
        text = (SOURCE / path).read_text()
        start = text.index(begin)
        excerpt = text[start:text.index(end, start)]
        reviews.append({'path': path, 'begin': begin, 'end_before': end,
                        'body_sha256': hashlib.sha256(excerpt.encode()).hexdigest(), 'finding': finding})

    body('crates/hypercurve/src/point.rs', 'impl PartialEq for Point2 {', '#[cfg(test)]',
         'Point equality first accepts shared storage, then requests x-difference zero knowledge and only requests y when x is proved zero. A false bool discards the distinction between NonZero and Unknown.')
    body('crates/hypercurve/src/segment.rs', '    pub(crate) fn contains_endpoint(', '    /// Returns whether this arc travels clockwise',
         'The initial start/end point comparisons precede policy-based point equality. Both stages retain early success for either endpoint.')
    body('crates/hypercurve/src/segment.rs', 'fn points_equal(', 'fn directed_radial_angle(',
         'Policy-based point equality repeats structural point equality before requesting zero knowledge for the squared distance.')
    body('crates/hypercurve/src/classify.rs', 'pub(crate) fn is_zero(', '#[track_caller]',
         'A known zero or nonzero fact returns immediately; Unknown proceeds through real_sign. Warming can strengthen a later query, so repeated calls cannot simply be omitted.')
    body('crates/hyperreal/src/computable/node/algebra.rs', '    fn project_pi_laurent_polynomial<T>(', '    fn pi_laurent_polynomial(',
         'The retained pass has 512 expansions, then drops its memo before one original 512-expansion retry if a retained certificate was reused and projection failed. Public budget documentation still needs to describe both passes before admission.')
    merge = ROOT / 'docs/evidence/2026-10-12-laurent-coefficient-merge'
    merge_delta = json.loads((merge / 'validation-v1/protocol.json').read_text())['source_changes_sha256']
    rational = 'crates/hyperreal/src/rational/arithmetic/ops.rs'
    assert rational not in merge_delta
    assert sha(SOURCE / rational) == sha(ROOT / 'target/laurent-coefficient-merge/source' / rational)
    rejection = json.loads((HISTORY / 'diagnostics-84.json').read_text())
    next_step = ('Instrument the initial and repeated point-equality stages inside contains_endpoint in a separate source and target. '
                 'Observe the Zero/NonZero/Unknown coordinate facts that the existing comparisons already request, final endpoint outcomes and policy categories. '
                 'Keep every original query, short circuit, temporary lifetime and validation result; counters must retain no scalar or geometry owners. '
                 'Only evaluate reuse of an already computed negative coordinate proof if the diagnostic supports it. Preserve preview/approximation receipts, shared budgets, aborts, cache strengthening, full reports and the original limits.')
    assessment = {
        'decision': 'diagnose-existing-negative-coordinate-proofs-at-endpoint-comparison',
        'previous_goal_turn_classification': 'progress',
        'scope': 'Fresh frame-pointer/line-table profile of the frozen metadata-first candidate, with both complete original boards. No rejected coefficient merge or arc retry-removal trial is inherited.',
        'quality': {key: observed[key] for key in ['samples', 'total_sample_period', 'lost_samples', 'empty_stack_period_percent', 'unknown_leaf_period_percent']},
        'profiled_worker_cpu_seconds': manifest['profiled_worker_cpu_seconds'],
        'original_worker_cpu_limit_seconds': 120,
        'inclusive_groups': observed['groups'],
        'normal_form_callers': observed['nearest_recovered_non_scalar_normal_form_callers'][:20],
        'allocator_leaf_subset_percent': attribution['allocator_leaf_total_percent'],
        'allocator_nearest_recovered_callers': attribution['allocator_nearest_recovered_project_callers'][:12],
        'point_equality_profile_percent': attribution['point_equality_total_percent'],
        'point_equality_callers': attribution['point_equality_nearest_recovered_project_callers'][:8],
        'subcurve_equality_callers': subcurve,
        'source_review': reviews,
        'prior_rejections': {
            'endpoint_retry': {'path': str((HISTORY / 'diagnostics-84.json').relative_to(ROOT)), 'sha256': sha(HISTORY / 'diagnostics-84.json'),
                               'decision': rejection['decision'],
                               'distinction': 'That trial omitted a later complete endpoint-membership call using an earlier exact negative result. The proposed diagnostic observes coordinate proof facts within the first call and removes no query.'},
            'coefficient_merge': {'patch_sha256': sha(merge / 'validation-v1/candidate.patch'),
                                  'rational_ops_unchanged_sha256': sha(SOURCE / rational),
                                  'finding': 'The actual runtime delta is in coefficient handling. Rational addition operators were unchanged; removing a purported rational-addition delta would not be a new trial.'},
        },
        'next_step': next_step,
        'limits': [
            'Sampled cycle periods are not wall seconds, call counts, allocations or predicted savings. Inclusive groups overlap.',
            'The prior parent profile is a different run and source. A lower equality share here does not by itself establish a matched speedup.',
            'No inline frames are decoded. Recovered callers are ancestors, not necessarily exact allocation sites. No recovered cache-read symbol weight does not mean zero cache cost.',
            'The profile still fails its original worker CPU assertion. The release candidate also remains unadmitted at its independently recorded 146.02494178 / 120-second result.',
            'Only the identified bodies and named rejection records were reviewed. This is not a full-crate or universal cache-frontier proof.',
        ],
        'artifacts_sha256': {str(p.relative_to(E)): sha(p) for p in sorted(OUT.iterdir()) if p.is_file()},
        'main_runtime_unchanged': True, 'runtime_admitted': False, 'plan_complete': False,
    }
    (E / 'assessment-v1.json').write_text(json.dumps(assessment, indent=2) + '\n')
    status_path = ROOT / 'docs/properties-status.json'
    status = json.loads(status_path.read_text())
    assert len(status['properties']) == 39 and len(status['targets']) == 14
    status['current_work']['isolated_region_metadata_profile'] = {
        'status': assessment['decision'], 'evidence': str((E / 'assessment-v1.json').relative_to(ROOT)),
        'result': '7326 samples, zero lost. Normal-form work has 29.031% inclusive sampled weight. Point equality has 7.569%; contains_endpoint accounts for 4.114% of total weight through point equality and 12.800% inclusive overall. Subcurve equality under CurveRegion2::new has 0.056%. Groups overlap; these are not speedup estimates.',
        'profiled_worker_cpu_seconds': manifest['profiled_worker_cpu_seconds'],
        'original_worker_cpu_limit_seconds': 120,
        'next_step': next_step, 'runtime_admitted': False,
    }
    status['current_work']['pending_native_geometry']['next_step'] = next_step
    status_path.write_text(json.dumps(status, indent=2) + '\n')
    print(json.dumps({'decision': assessment['decision'], 'samples': observed['samples'],
                      'profiled_worker_cpu_seconds': manifest['profiled_worker_cpu_seconds']}))


if __name__ == '__main__':
    main()
