"""Close this isolated trial without waiving its native or broader cost failures."""
import gzip
import hashlib
import json
from pathlib import Path
import sys

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
BASE = ROOT / 'target/region-metadata-first/source'
SOURCE = ROOT / 'target/endpoint-coordinate-reuse/source'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources


def main():
    preparation = json.loads((E / 'preparation.json').read_text())
    validation = json.loads((E / 'validation-assessment-v1.json').read_text())
    controls = json.loads((E / 'controls-assessment-v1.json').read_text())
    reports = json.loads((E / 'report-assessment-v1.json').read_text())
    baseline = json.loads(gzip.decompress((E / 'baseline-source-hashes.json.gz').read_bytes()))
    current = baseline | preparation['source_changes_sha256']
    assert all(sha(BASE / name) == digest for name, digest in baseline.items())
    assert all(sha(SOURCE / name) == digest for name, digest in current.items())
    assert sources() == preparation['main_source_sha256']
    pending_record = ROOT / 'docs/evidence/2026-10-12-existing-worktree/geometry-validation-closure/result.json'
    pending = json.loads(pending_record.read_text())['pending_native_sha256']
    assert len(pending) == 35 and all(sha(ROOT / name) == digest for name, digest in pending.items())
    for assessment in [validation, controls, reports]:
        assert all(sha(E / name) == digest for name, digest in assessment['artifacts_sha256'].items())
    # Confirm the exact measured executable/library identities remain available
    # after the separate report build with its own recorded feature selection.
    libraries = json.loads((E / 'controls-v1/libraries.json').read_text())
    binaries = json.loads((E / 'controls-v1/binaries.json').read_text())
    assert all(sha(Path(row['path'])) == row['sha256'] for values in libraries.values() for row in values.values())
    assert all(sha(Path(row['path'])) == row['sha256'] for row in binaries.values())
    assert all(sha(Path(row['path'])) == row['sha256'] for row in reports['production_binaries'].values())
    assert validation['test_counts']['curve-tests']['passed'] == 2494
    assert validation['test_counts']['drc-regressions']['passed'] == 26
    assert not validation['all_original_native_gates_passed']
    assert controls['historical_comparison']['all_231_historical_witnesses_identical']
    assert reports['complete_reports_byte_identical']
    exact_families = ['rational_non_endpoint', 'radical_non_endpoint', 'pi_scaled_non_endpoint',
                      'opaque_translated_non_endpoint', 'off_circle', 'other_sweep', 'near_endpoint_off_circle']
    exact_rows = [row for row in controls['arc_timing'] if row['name'] in [family + '_' + phase for family in exact_families for phase in ['first', 'warm', 'cloned']]]
    assert len(exact_rows) == 21 and sum(row['faster_pairs'] == 5 for row in exact_rows) == 20
    source_review = []

    def unchanged_body(path, begin, end, after='', finding=''):
        def extract(tree):
            text = (tree / path).read_text()
            start = text.index(begin, text.index(after) if after else 0)
            return text[start:text.index(end, start)]
        original, candidate = extract(BASE), extract(SOURCE)
        assert original == candidate
        source_review.append({'path': path, 'begin': begin, 'end_before': end,
                              'body_sha256': hashlib.sha256(candidate.encode()).hexdigest(),
                              'finding': finding})

    unchanged_body('crates/hypercurve/src/point.rs', 'impl PartialEq for Point2 {', '#[cfg(test)]',
                   finding='Global boolean point equality is byte-for-byte unchanged.')
    unchanged_body('crates/hypercurve/src/segment.rs', '    pub(crate) fn contains_sweep_point_with_policy(', '    fn contains_non_endpoint_sweep_point(',
                   finding='Sweep containment still executes the later complete endpoint query; the earlier rejected retry-omission trial is not inherited.')
    unchanged_body('crates/hypercurve/src/segment.rs', '    pub(crate) fn contains_point_with_policy(', '    /// Returns conservative structural facts for this arc.', after='impl CircularArc2 {',
                   finding='Point-on-arc classification still checks endpoints, radius equality and sweep in the original order.')
    unchanged_body('crates/hypercurve/src/segment.rs', 'fn points_equal(', 'fn directed_radial_angle(',
                   finding='Every retained policy retry still repeats boolean point equality and then its original squared-distance zero query.')
    unchanged_body('crates/hyperreal/src/computable/node/structural_analysis.rs', '    pub(crate) fn exact_quadratic_surd_parts(', '    #[cfg(test)]',
                   finding='The quadratic projection calls the shared two-pass Laurent normalizer directly; the corrected budget docs describe this unchanged implementation.')
    unchanged_body('crates/hyperreal/src/computable/node/algebra.rs', '    fn project_pi_laurent_polynomial<T>(', '    fn pi_laurent_polynomial(',
                   finding='The retained pass and optional original retry retain their 512-expansion limits and the drop of the first memo before fallback.')
    next_step = (
        'Attribute the remaining cost tradeoffs before extending this candidate. In separate diagnostic copies of both frozen trees, observe whether '
        'the mixed-quadratic aggregation, warm polygon area and unit-rectangle controls execute endpoint classification, and which earlier stages warm the shared facts they use. '
        'Also trace the extra aborted-unknown first-query and approximate non-endpoint first-query allocations without changing original query order or retaining owners. '
        'Do not extend the fixed five-pair samples to remove adverse rows. Resolve the changed caller/cache-work behavior or reject this trial as a parent; '
        'then select the next concrete full-board cost from a fresh profile. The original 120-second worker CPU gate, broader native/workflow qualification and the entire properties/future-work plan remain open.'
    )
    decision = {
        'decision': 'retain-isolated-endpoint-proof-trial-for-cost-attribution-no-runtime-admission',
        'validation': validation['test_counts'], 'native_gates': validation['native_gates'],
        'intended_exact_arc_rows': len(exact_rows),
        'intended_arc_median_paired_ratio_range': [min(row['median_paired_ratio'] for row in exact_rows), max(row['median_paired_ratio'] for row in exact_rows)],
        'intended_arc_all_five_faster_rows': 20,
        'historical_witnesses_preserved': 231,
        'effective_final_coordinate_abort_receipts_identical': controls['final_coordinate_abort'],
        'adverse_arc_rows': [row for row in controls['arc_timing'] if row['median_paired_ratio'] > 1],
        'broad_all_five_slower_rows': controls['broad_slower_in_all_five_pairs'],
        'phase_memory_increases': controls['memory_increases'],
        'complete_reports': reports['comparisons'],
        'production_binary_byte_delta': reports['production_binary_byte_delta'],
        'report_process_observations': reports['process_observations'],
        'source_review': source_review,
        'mathematical_basis': 'For real coordinates, a certified nonzero coordinate difference excludes point equality independently of the other coordinate. The new helper preserves the distinction between that certificate and an unknown boolean comparison. Exact non-preview endpoint aggregation reuses only this sufficient negative evidence; either-endpoint success remains first.',
        'limits': [
            'The original full-board assertion still fails at 138.312559051 worker CPU seconds / 120. The separate copper-neck assertion passes, but its check-body duration is not printed; 128.075790529 whole-process seconds must not be relabelled as that duration.',
            'All original five-pair observations remain, including nine broad rows slower in every pair, phase allocation/retention increases, and a 1.117 median approximate-endpoint cloned-query ratio with four slower pairs. Their causes are not established by these timings.',
            'Complete report identity and lower standalone report RSS do not establish a matched application speedup, a universal memory improvement or graph reclamation.',
            'The source argument establishes sufficient inequality evidence, not universal equivalence of cache-warming side effects after skipped work. The named controls qualify observed behavior; the broader cache/completeness audit remains open.',
            'Five new regression functions overlap the full curve total. The 231 historical rows and repeated abort receipts are not new universal proof counts. No complete DRC suite or all-workflow validation is claimed for this isolated trial.',
            'No rejected global point-equality shortcut, later whole endpoint omission, coefficient merge or eight-node quadratic prepass is inherited.',
        ],
        'next_step': next_step,
        'assessment_sha256': {name: sha(E / name) for name in ['preparation.json', 'preflight-assessment-v1.json', 'validation-assessment-v1.json', 'controls-assessment-v1.json', 'report-assessment-v1.json']},
        'pending_native_guard_sha256': sha(pending_record), 'pending_native_entries_unchanged': 35,
        'main_runtime_unchanged': True, 'runtime_admitted': False, 'plan_complete': False,
    }
    path = E / 'decision-v1.json'
    assert not path.exists()
    path.write_text(json.dumps(decision, indent=2) + '\n')
    ledger_path = ROOT / 'docs/properties-status.json'
    ledger = json.loads(ledger_path.read_text())
    assert len(ledger['properties']) == 39 and len(ledger['targets']) == 14
    work = ledger['current_work']['isolated_endpoint_coordinate_reuse']
    work['status'] = decision['decision']
    work['decision_evidence'] = str(path.relative_to(ROOT))
    work['report_evidence'] = str((E / 'report-assessment-v1.json').relative_to(ROOT))
    work['report_summary'] = 'Both complete unnormalized reports are byte-identical across four fresh CLI runs on the same 108 extracted files: CPArti 147892 bytes / 12 findings and HVP109A 128807 bytes / 197 findings. Production executable size increases 968 bytes. Standalone durations and RSS do not establish a matched performance or memory improvement.'
    work['source_review'] = 'Global Point2 equality, the original policy retry, later whole endpoint call, and point/radius/sweep order remain unchanged. Public normal-form docs now correctly describe the existing at-most-two 512-expansion passes.'
    work['next_step'] = next_step
    work['runtime_admitted'] = False
    ledger['current_work']['pending_native_geometry']['next_step'] = next_step
    ledger_path.write_text(json.dumps(ledger, indent=2) + '\n')
    print(json.dumps({'decision': decision['decision'], 'native_gates': decision['native_gates'],
                      'intended_ratio_range': decision['intended_arc_median_paired_ratio_range'],
                      'broad_all_five_slower_rows': len(decision['broad_all_five_slower_rows']),
                      'production_binary_byte_delta': decision['production_binary_byte_delta'], 'pending_native_entries_unchanged': 35}))


if __name__ == '__main__':
    main()
