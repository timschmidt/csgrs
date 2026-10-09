"""Assemble the native-rectangle record after validation, cost and report audits."""
import hashlib
import json
from pathlib import Path
import re

record = Path(__file__).resolve().parent
load = lambda path: json.loads(path.read_bytes())
info = lambda path: dict(bytes=Path(path).stat().st_size, sha256=hashlib.sha256(Path(path).read_bytes()).hexdigest())
base = load(record / 'baseline.json')['production_commit']
for folder in ['validation', 'consumer-validation', 'expanded-fixed']:
    assert all(step['exit_code'] == 0 for step in load(record / folder / 'status.json'))
    assert load(record / folder / 'sources.json')['unchanged']
cost = load(record / 'cost/manifest.json')
replay = load(record / 'report-replay/manifest.json')
assert cost['source_unchanged'] and cost['fixture_exit_code'] == 0 and replay['source_unchanged']
delta = load(record / 'report-replay/baseline-delta-audit.json')
assert delta['status'] == 'passed'
rows_audit = load(record / 'cost/row-delta-audit.json')
assert rows_audit['status'] == 'passed'
fixture_audit = load(record / 'cost/fixture-report-audit.json')
fixture = (record / 'cost/fixture.log').read_text()
seconds = float(re.search(r'fixture_smoke_total ms=([\d.]+)', fixture)[1]) / 1000
rss = [row['peak_rss_kib'] for row in cost['observations']]
manifest = dict(
    base_revision=base, status='progress',
    scope='Native rectangle construction at the finite API boundary, with explicit missing/unavailable-size failure for KiCad primitive pads, including nonzero source underflow. All 39 properties, 14 targets and crate future work remain in scope.',
    contract=[
        'Finite center, size and degree inputs enter the scalar domain before half-size arithmetic, rotation and translation. Native regularized geometry is independent of collapsed or overflowing finite report corners.',
        'Signed dimensions denote the same regularized rectangle; a zero dimension is certified empty. Non-finite input, unavailable construction and exhausted shared budgets remain attached failures. Warm measurements retain the same guards.',
        'KiCad keeps optional finite size through custom-primitive selection. Admitted custom geometry needs no unused size; other primitives retain missing/unavailable size as source failure instead of manufacturing zero dimensions. A projected zero requires an exact source-zero certificate; nonzero underflow remains a failure.',
        'The original 1e20 / width 16384 hard case is retained without shrinking or moving it. Its native area is exactly 268435456; adjacent imported centers share an exact closed edge. Tests also cover one-unit and subnormal dimensions, unrepresentable corners, exact dyadic inputs, rotations, zero/signed dimensions, preview and budget scopes.',
    ],
    baseline=dict(record='baseline.json', passed=3, failed=9, reconstructed_source_files=135,
                  full_reports='baseline-reports/manifest.json', library='baseline-library.json'),
    validation=dict(status='passed', status_file='validation/status.json', source_hashes='validation/sources.json',
                    drc_passed=1528, drc_library=1523, readme_integration=2, drc_doctests=3,
                    new_regressions=26, rectangle_regressions=20, kicad_regressions=6,
                    strict_all_feature_all_target_clippy=True, strict_rustdoc=True, workspace_format_check=True),
    focused_validation=dict(record='expanded-fixed/status.json', rectangles=20, kicad=42,
                             initial_record='candidate-initial/status.json', initial_rectangles=12, existing_geometry=59, existing_affine=12),
    consumer_validation=dict(status='passed', status_file='consumer-validation/status.json', passed=14,
                             strict_all_feature_all_target_clippy=True, snapshots='snapshot-versions.json'),
    corrections=['expanded-compile/correction.json', 'integration-failure/correction.json', 'feature-race/correction.json', 'underflow-correction/correction.json', 'report-replay/comparison-correction.json'],
    cost=dict(status='passed', manifest='cost/manifest.json', rows=cost['rows'], trials=5,
              previous_workloads=193, unchanged_inputs_and_iterations=True, unchanged_previous_rows=188,
              row_delta_audit='cost/row-delta-audit.json', auxiliary_public_probes='cost/auxiliary-probes/manifest.json',
              new_medians_ns={k:v for k,v in cost['medians_ns'].items() if k.startswith('operation_rectangle_')},
              peak_rss_kib=[min(rss),max(rss)], fixture_seconds=seconds,
              fixture_peak_rss_kib=int(re.search(r'Maximum resident set size \(kbytes\): (\d+)', fixture)[1]),
              fixture_gate_seconds=120, limitations=cost['conditions']),
    report_replay=dict(manifest='report-replay/manifest.json', reports=replay['reports'],
                       complete_candidate_bytes_identical=True, baseline_version_fields_per_board=12,
                       initial_version_only_comparison='report-replay/initial-comparison.json',
                       exact_delta_audit='report-replay/baseline-delta-audit.json', scope=delta['scope']),
    fixture_finding_audit='cost/fixture-report-audit.json',
    remaining=[
        'Other finite primitives (rounded/chamfered rectangles, trapezoids, lines, sampled strokes) and parser token-to-float paths remain open, including primitive size arithmetic and authored-center rounding before the rectangle API.',
        'Forty legacy CopperSpatialIndex constructor sites, remaining shape/drill/distribution consumers, scalar/net-class thresholds, report-chord widths and density proxies still need audit.',
        'Native expressions can retain unavailable comparisons. This checkpoint does not prove universal operation resource bounds or complete scalar trigonometric identities.',
        'The report comparison covers twelve selected checks on both preserved full boards plus a retained synthetic seven-record IPC sidecar; it does not establish universal or path-independent report determinism.',
        'All 39 properties, all 14 targets and crate future work remain in scope. No push or CI configuration change; the complete pre-push workflow matrix has not been run.',
    ],
    replay=['reproduce_baseline.py BASE_CHECKOUT OUTPUT', 'reproduce_baseline_reports.py BASE_CHECKOUT OUTPUT',
            'reproduce_checkpoint_validation.py OUTPUT_ROOT',
            'reproduce_cost.py VALIDATION_OUTPUT COST_OUTPUT', 'reproduce_auxiliary_probes.py COST_OUTPUT',
            'audit_cost_rows.py COST_OUTPUT',
            'audit_fixture_reports.py COST_OUTPUT', 'reproduce_report_replay.py COST_OUTPUT REPORT_OUTPUT',
            'audit_report_deltas.py REPORT_OUTPUT', 'freeze_logs.py', 'build_manifest.py', 'audit_artifacts.py'],
    runtime=dict(cost='/tmp/drc-native-rectangles-cost', report_replay='/tmp/drc-native-rectangles-report-replay',
                 baseline_libraries='/tmp/drc-native-rectangles-baseline-libraries', inputs='/tmp/drc-native-rectangles-report-inputs'),
    other_input_hashes={str(p):info(p)['sha256'] for p in [Path('Cargo.toml'), Path('Cargo.lock'), Path('AGENTS.md'),
                        Path('crates/hyperdrc/Cargo.toml'), Path('crates/hypercircuit/Cargo.toml'),
                        Path('crates/hyperdrc/README.md'), Path('crates/hyperdrc/PERFORMANCE.md'),
                        Path('crates/hyperdrc/src/geometry/README.md'), Path('.github/workflows/workspace.yml')]},
    artifact_integrity_audit='artifact-audit.json',
    evidence_files={str(p.relative_to(record)):info(p) for p in sorted(record.rglob('*')) if p.is_file() and p not in [record/'manifest.json', record/'artifact-audit.json']},
)
(record / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
print('Wrote native-rectangle checkpoint manifest.')
