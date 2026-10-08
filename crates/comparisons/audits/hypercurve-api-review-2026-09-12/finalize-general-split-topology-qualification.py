from pathlib import Path
import hashlib, json, subprocess

root = Path('/home/tim/Documents/GitHub/workspace')
audit = root / 'hypercurve-api-review-2026-09-12'
prefix = 'general-split-topology'
results = json.loads((audit / (prefix + '-full-results.json')).read_text())
assert len(results) == 49
assert not any(row['new_failures'] or row['timed_out'] for row in results)
assert all(row['returncode'] in {0, 101} for row in results)
for suffix in ['', '-hyperbrep']:
    for kind in ['-check', '-test-build']:
        assert (audit / (prefix + suffix + kind + '.exit')).read_text().strip() == '0'
rows = json.loads((audit / (prefix + '-source-before-tests.json')).read_text())
changed = [row['file'] for row in rows if not (root / row['file']).is_file()
           or hashlib.sha256((root / row['file']).read_bytes()).hexdigest() != row['sha256']]
assert not changed, changed
edited = subprocess.check_output(['git', 'diff', '--name-only'], cwd=root / 'hypercurve').decode().splitlines()
subprocess.run(['rustfmt', '--edition', '2024', '--config', 'skip_children=true', '--check']
               + [name for name in edited if name.endswith('.rs')], cwd=root / 'hypercurve', check=True)
subprocess.run(['git', 'diff', '--check'], cwd=root / 'hypercurve', check=True)
callers = json.loads((audit / (prefix + '-caller-audit.json')).read_text())
native_api_files = ['benches/rational_bezier.rs', 'src/rational_bezier_general.rs', 'tests/hypercurve_rational_bezier.rs']
for row in callers['repositories']:
    for match in row['matches']:
        assert row['repository'] == 'hypercurve'
        assert match.split(':')[0] in native_api_files, match
        assert 'arrangement_graph_view' in match, match
rlib = root / 'hypercurve/target/release/deps/libhypercurve-2c7ff6f3fbe07653.rlib'
source_checks = {'source_files': len(rows), 'changed_sources': changed,
                 'formatting': True, 'whitespace': True,
                 'source_manifest': prefix + '-source-before-tests.json'}
(audit / (prefix + '-source-checks.json')).write_text(json.dumps(source_checks, indent=2) + '\n')
qualification = {
    'status': 'validated; awaiting incremental commit',
    'goal_status': 'active',
    'base_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=root / 'hypercurve').decode().strip(),
    'change': [
        'Return traversal-ordered Curve2 pieces from general curve/path split topology, preserving selected parameters without native Bezier promotion.',
        'Share one retained arrangement graph and borrow it directly; prepare it within operation certainty tracking.',
        'Use common support charts for cuts and existing flat source restrictions for repeated subdivision.',
        'Remove the obsolete general materialization/view APIs and splitter, updating callers directly without compatibility shims.',
        'Reuse already prepared native trim spans and parameters, removing repeat promotion and aggregate materialization storage.',
        'Identify authored curves and traversal-ordered fragments in arrangement provenance.'
    ],
    'regressions': {
        'new_public_tests': [
            'generated_chord_topology_publishes_reusable_curve_pieces',
            'selected_tail_topology_keeps_reversed_and_nonunit_source_charts',
            'split_topology_preserves_both_sides_of_a_discontinuous_spline_knot'
        ],
        'expanded_public_tests': [
            'generated_chord_overlaps_retain_independent_and_selected_boundaries',
            'generated_chord_cuts_reenter_collinear_endpoint_intersections'
        ],
        'additional_topology_queries': 256,
        'additional_piece_reentry_intersections': 96,
        'setup_intersections_in_new_tests': 8,
        'policies': ['STRICT', 'APPROXIMATE_512'],
        'oracles': 'Independent exact endpoint coordinates, parameter evaluation, split-piece reentry, one-sided discontinuous knot limits, and authored-curve graph provenance.'
    },
    'qualification': {
        'results': prefix + '-full-results.json', 'targets': len(results),
        **{key: sum(len(row[key]) for row in results) for key in ['passed', 'failed', 'ignored', 'new_failures']},
        'hypercurve_passed': sum(len(row['passed']) for row in results if row['repo'] == 'hypercurve'),
        'hyperbrep_passed': sum(len(row['passed']) for row in results if row['repo'] == 'hyperbrep'),
        'remaining_failures': sorted(f for row in results for f in row['failed']),
        'timeouts': 0, 'previously_unqualified_expensive_cases': 8, 'full_suite_passing': False,
        'all_target_checks': True, 'immutable_sources': True, 'source_files': len(rows),
        'formatting_and_whitespace_checks': True, 'caller_audit': prefix + '-caller-audit.json',
        'rlib_sha256': hashlib.sha256(rlib.read_bytes()).hexdigest()
    },
    'evidence_audit': prefix + '-evidence-audit.md',
    'performance_claim': None,
    'remaining': [
        'General curve/region trimming beyond native span representative points.',
        'Selected-circle and analytic-parallel common pair dispatch.',
        'Finite exterior source domains and complete partially coincident non-injective/retraced parameter relations.',
        'Inverse branch transport for a later narrower chord domain in a chord/rational correspondence.',
        'Direct propagation of contact topology identities into arrangement vertices and further reuse/scheduling measurements.',
        'Nine pre-existing promotion failures and eight expensive unqualified cases.',
        'The broader full-family API, region-normalization and computational-closure architecture plan.'
    ]
}
(audit / (prefix + '-qualification.json')).write_text(json.dumps(qualification, indent=2) + '\n')
print(json.dumps(qualification['qualification'], indent=2))
