"""Assess the completed profile and record a bounded next investigation."""
import gzip
import hashlib
import json
from pathlib import Path
import subprocess
import sys

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
OUT = E / 'profile-v1'
SOURCE = ROOT / 'target/small-integer-certificates/source'
PARENT = ROOT / 'docs/evidence/2026-10-12-small-integer-certificates'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources


def main():
    manifest = json.loads((OUT / 'manifest.json').read_text())
    protocol = json.loads((OUT / 'protocol.json').read_text())
    observed = json.loads((OUT / 'observations.json').read_text())
    allocation = json.loads((OUT / 'attribution.json').read_text())
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
    assert len(expected) == 10128
    assert all(sha(SOURCE / name) == digest for name, digest in expected.items())
    assert sources() == json.loads((PARENT / 'preparation.json').read_text())['main_source_sha256']
    assert observed['samples'] == 8362 and observed['lost_samples'] == 0
    review = []

    def body(path, begin, end, finding):
        text = (SOURCE / path).read_text()
        start = text.index(begin)
        excerpt = text[start:text.index(end, start)]
        review.append({'path': path, 'begin': begin, 'end_before': end,
                       'body_sha256': hashlib.sha256(excerpt.encode()).hexdigest(), 'finding': finding})

    body('crates/hypercurve/src/point.rs', 'impl PartialEq for Point2 {', '#[cfg(test)]',
         'Shared storage proves equality immediately. Other pairs construct exact coordinate differences and request zero knowledge, short-circuiting x before y.')
    body('crates/hypercurve/src/bezier_split.rs', 'pub(crate) enum BezierSubcurve2 {', '/// One exact analytic Bezier-parallel image',
         'Derived equality dispatches across four curve-family variants. The enum itself has no parameter or source-index fields to reorder.')
    body('crates/hypercurve/src/bezier_region.rs', 'pub(crate) struct CurveRegionFragmentSource2 {', 'impl CurveRegionFragmentSource2 {',
         'Source metadata consists of three usize indices with derived exact equality.')
    body('crates/hypercurve/src/bezier_region.rs', 'fn validate_retained_region_loops(', 'fn validate_retained_region_arrangement_sources(',
         'Duplicate-loop validation compares fragments before arrangement_sources in a conjunction, then performs the separate cross-loop arrangement-source uniqueness check.')
    body('crates/hypercurve/src/bezier_region.rs', '    pub(crate) fn new(boundary_loops: Vec<CurveRegionBoundaryLoop2>)', '    /// Attaches authored role, fill, and interior-side hints',
         'Nonempty region construction invokes duplicate-loop validation before constructing the certified internal carrier.')
    body('crates/hypercurve/src/classify.rs', 'pub(crate) fn real_sign(', 'pub(crate) fn is_zero(',
         'real_sign first requests zero_status, then preserves the preview or certified-predicate cascade.')
    body('crates/hypercurve/src/classify.rs', 'pub(crate) fn is_zero(', '#[track_caller]',
         'An Unknown zero_status result calls real_sign, which requests zero_status again. Removing that request is not justified here: warming may strengthen a later fact.')
    assessment = {
        'decision': 'diagnostic-retained-next-investigation-is-region-validation-metadata',
        'goal_turn_classification': 'progress',
        'scope': 'Fresh frame-pointer/line-table all-feature profile of the frozen integer-leaf parent, using the complete original two-board smoke. The rejected coefficient merge is absent.',
        'quality': {name: observed[name] for name in ['samples', 'total_sample_period', 'lost_samples', 'empty_stack_period_percent', 'unknown_leaf_period_percent']},
        'board_fixture_sample_percent': observed['groups']['board_fixture']['inclusive_percent'],
        'profiled_worker_cpu_seconds': manifest['profiled_worker_cpu_seconds'],
        'original_worker_limit_seconds': 120,
        'profile_runtime_is_release_admission': False,
        'selected_inclusive_groups': {name: observed['groups'][name] for name in [
            'polynomial_normal_form', 'quadratic_projection', 'exact_sign', 'structural_facts',
            'regularized_fragment_classification', 'regularized_fragment_actions']},
        'normal_form_callers': observed['nearest_recovered_non_scalar_normal_form_callers'][:12],
        'allocation_attribution': {key: allocation[key] for key in [
            'allocator_leaf_total_percent', 'allocator_leaf_with_normal_form_total_percent',
            'allocator_nearest_recovered_project_callers', 'limits']},
        'point_equality_total_profile_percent': allocation['point_equality_total_percent'],
        'point_equality_callers': allocation['point_equality_nearest_recovered_project_callers'][:8],
        'subcurve_equality_callers': subcurve,
        'source_review': review,
        'next_step': {
            'question': 'How much geometry-comparison work occurs between loops whose integer source metadata already differs?',
            'diagnostic': 'In a separate source and target, classify loop comparisons by metadata presence/equality and retain the original geometry comparison, validation order, errors and final results. Counters must not retain expression owners. Quantify geometry time and outcomes on both full original boards.',
            'possible_trial_only_after_diagnostic': 'Compare source-index metadata first in the existing conjunction if evidence supports it. Preserve the separate arrangement-source uniqueness validation and all duplicate-loop behavior.',
            'required_controls': 'Absent, equal and unequal provenance; shared and separately constructed equal geometry; duplicate and reused arrangement indices; unchanged error messages; cold/warm later facts, complete reports, original runtime limits and measured costs.',
        },
        'limits': [
            'Sampled cycle periods are not call counts, allocated bytes or predicted savings. Inclusive groups overlap and must not be added.',
            'No inline frames are decoded; recovered callers need not be exact allocation sites. The cache-read symbol group has zero recovered weight, which does not imply zero cache cost.',
            'This profiling build changes code generation. Its time and percentages are not a matched comparison with earlier profiles or a release-speedup claim.',
            'Only the named source sections were reviewed here. This is not a full-file, full-crate or universal monotonicity proof.',
        ],
        'artifacts_sha256': {str(p.relative_to(E)): sha(p) for p in sorted(OUT.iterdir()) if p.is_file()},
        'analysis_scripts_sha256': {name: sha(E / name) for name in ['profile_v1.py', 'analyze_v1.py', 'attribute_v1.py', 'subcurve_callers_v1.py']},
        'main_runtime_unchanged': True, 'runtime_admitted': False, 'plan_complete': False,
    }
    (E / 'assessment-v1.json').write_text(json.dumps(assessment, indent=2) + '\n')
    print(json.dumps({'decision': assessment['decision'], 'samples': observed['samples'],
                      'normal_form_percent': observed['groups']['polynomial_normal_form']['inclusive_percent'],
                      'subcurve_equality_percent': subcurve['selected_total_profile_percent']}))


if __name__ == '__main__':
    main()
