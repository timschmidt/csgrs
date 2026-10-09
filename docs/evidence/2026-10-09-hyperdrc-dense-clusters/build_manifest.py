"""Assemble the checkpoint record after completed validation and evidence audits."""
import hashlib
import json
from pathlib import Path
import re

record = Path(__file__).resolve().parent
load = lambda path: json.loads(path.read_bytes())
info = lambda path: dict(bytes=Path(path).stat().st_size, sha256=hashlib.sha256(Path(path).read_bytes()).hexdigest())
base = load(record / 'baseline.json')['production_commit']
for folder in ['validation', 'consumer-validation']:
    assert all(step['exit_code'] == 0 for step in load(record / folder / 'status.json'))
    assert load(record / folder / 'sources.json')['unchanged']
cost = load(record / 'cost/manifest.json')
replay = load(record / 'report-replay/manifest.json')
assert cost['source_unchanged'] and cost['fixture_exit_code'] == 0 and replay['source_unchanged']
delta = load(record / 'report-replay/baseline-delta-audit.json')
assert delta['status'] == 'passed'
assert all(row['all_other_bytes_unchanged'] for row in delta['reports'].values())
rss = [r['peak_rss_kib'] for r in cost['observations']]
fixture = (record / 'cost/fixture.log').read_text()
seconds = float(re.search(r'fixture_smoke_total ms=([\d.]+)', fixture)[1]) / 1000
fixture_rss = int(re.search(r'Maximum resident set size \(kbytes\): (\d+)', fixture)[1])
manifest = dict(
    base_revision=base, status='progress',
    scope='All four dense-pad readiness checks: local fiducials, escape vias, via/pad copper spacing and pad mask-web spacing. Native pitch qualifies clusters and native arithmetic derives their average center. All 39 properties, 14 targets and crate future work remain in scope.',
    contract=[
        'Native point enclosures schedule authored-center candidates. Native shape enclosures schedule copper-boundary candidates independently of authored centers. Unknown or unprojectable optional enclosures retain candidates; required measurements and markers retain typed uncertainty.',
        'Cluster pitch and clearance selection require certified qualifying witnesses. Unknown alternative measurements or optional minimum ordering do not erase a proved witness; diagnostics say certified pitch/clearance unless the minimum itself is certified.',
        'Native fiducial dimensions use six equivalent inequalities without unnecessary width/height ordering or division. A certified exclusion proves ineligibility despite unknown alternatives. Unknown eligibility otherwise remains required uncertainty.',
        'Two certified fiducials prove sufficient registration regardless of unresolved alternatives; any certified escape via proves existence. A count below two must be exact. Layer eligibility precedes required matching.',
        'All four checks run under STRICT and the shared budget guard, including empty and warm paths. Required budget failures survive certified alternatives. Six-place diagnostics use the explicit fixed-decimal display view.',
    ],
    baseline=dict(record='baseline.json', passed=2, failed=19, reconstructed_source_files=126,
                  full_reports='baseline-reports/manifest.json',
                  reused_library_provenance='baseline-reports/reused-cost-manifest.json'),
    validation=dict(status='passed', status_file='validation/status.json', source_hashes='validation/sources.json',
                    drc_passed=1454, drc_library=1449, readme_integration=2, drc_doctests=3,
                    new_regressions=38, initial_regressions=21,
                    strict_all_feature_all_target_clippy=['hyperdrc'], strict_rustdoc=['hyperdrc'],
                    workspace_format_check=True, source_frozen_through_cost_and_report_replay=True),
    consumer_validation=dict(status='passed', status_file='consumer-validation/status.json', passed=14,
                             strict_all_feature_all_target_clippy=True, snapshots='snapshot-versions.json',
                             six_snapshots_only_version_14_to_15=True),
    cost=dict(status='passed', manifest='cost/manifest.json', rows=cost['rows'], trials=5,
              previous_one_hundred_thirty_nine_witness_rows_unchanged=True,
              new_medians_ns={k:v for k,v in cost['medians_ns'].items() if k.startswith('operation_dense_')},
              peak_rss_kib=[min(rss),max(rss)], fixture_seconds=seconds,
              fixture_peak_rss_kib=fixture_rss, fixture_gate_seconds=120, limitations=cost['conditions']),
    report_replay=dict(manifest='report-replay/manifest.json', reports=replay['reports'],
                       complete_candidate_bytes_identical=True, baseline_version_fields_per_board=4,
                       baseline_diagnostic_changes={label:len(row['diagnostic_changes']) for label,row in delta['reports'].items()},
                       exact_delta_audit='report-replay/baseline-delta-audit.json',
                       initial_version_only_gate='report-replay/initial-comparison.json',
                       scope=delta['scope']),
    fixture_finding_audit='cost/fixture-report-audit.json', constructor_inventory='constructor-inventory.json',
    staged_validation=['candidate-initial.json', 'expanded-validation/status.json', 'expanded-fixed/manifest.json'],
    retained_fixture_limitation='expanded-validation/fixture-correction.json',
    evidence_driver_failure='draft/cost-private-bounds/manifest.json',
    remaining=[
        '43 legacy CopperSpatialIndex constructor sites remain. Thermal copper-area readiness still matches authored power-feature/zone centers. IPC testpoint side parity/accessibility, diameter, spacing and edge logic, other shape/width/density proxies, source adapters and full property coverage remain open.',
        'A new native-grid fixture exposed finite rectangle construction rounding at 1e20: a 16,384 width collapses before native admission and retains an insufficient-vertices failure. The original staged regression and failure log remain; direct native unit-width rectangles test the grid. Cost probes retain the original typed construction failure. Source construction is not repaired here.',
        'General Real alternate/scientific formatting still seeks a leading bit for unresolved zero. Explicit fixed-decimal diagnostics do not establish universal magnitude, precision or expression resource bounds.',
        'No universal or path-independent report determinism claim. Complete Report comparisons cover these four checks on two preserved full KiCad fixtures. The separate smoke fixture retains its original checks and inputs.',
        'Shared STRICT and budget guards do not establish total operation resource bounds. All 39 properties, all 14 targets and crate future work remain in scope.',
        'No push or CI configuration change. The complete repository pre-push workflow matrix has not been run.',
    ],
    replay=['reproduce_baseline.py BASE_CHECKOUT OUTPUT', 'reproduce_baseline_reports.py BASE_CHECKOUT OUTPUT',
            'reproduce_validation.py OUTPUT', 'reproduce_consumer_validation.py OUTPUT',
            'reproduce_cost.py VALIDATION_OUTPUT COST_OUTPUT', 'audit_fixture_reports.py COST_OUTPUT',
            'reproduce_report_replay.py COST_OUTPUT REPORT_OUTPUT', 'audit_report_deltas.py REPORT_OUTPUT',
            'audit_constructor_inventory.py',
            'freeze_logs.py', 'build_manifest.py', 'audit_artifacts.py'],
    runtime=dict(validation=str(record / 'validation'), consumer_validation=str(record / 'consumer-validation'),
                 cost='/tmp/drc-dense-clusters-cost', report_replay='/tmp/drc-dense-clusters-report-replay',
                 baseline_libraries='/tmp/drc-dense-clusters-baseline-libraries', inputs='/tmp/drc-dense-clusters-report-inputs'),
    other_input_hashes={str(p):info(p)['sha256'] for p in [Path('Cargo.toml'), Path('Cargo.lock'), Path('AGENTS.md'),
                        Path('crates/hyperdrc/Cargo.toml'), Path('crates/hypercircuit/Cargo.toml'),
                        Path('crates/hyperdrc/README.md'), Path('crates/hyperdrc/PERFORMANCE.md'),
                        Path('.github/workflows/workspace.yml')]},
    artifact_integrity_audit='artifact-audit.json',
    evidence_files={str(p.relative_to(record)):info(p) for p in sorted(record.rglob('*')) if p.is_file() and p not in [record/'manifest.json', record/'artifact-audit.json']},
)
(record / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
print('Wrote dense-cluster checkpoint manifest.')
