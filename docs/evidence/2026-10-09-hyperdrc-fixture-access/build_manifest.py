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
    scope='Thermal copper-area authored-center matching and all three IPC fixture checks: net-name coverage, accessibility and unrelated-copper keepout. Shared native circle envelopes also serve existing drill consumers. All 39 properties, 14 targets and crate future work remain in scope.',
    contract=[
        'Thermal zone-center matching uses native point enclosures and certified radius predicates after layer/net eligibility. A certified alternative proves proximity; unresolved candidates cannot prove absence. This is the retained placement proxy, not a measured heat-spreading area.',
        'IPC net-name coverage projects only the first three reported markers for each missing critical net. Covered nets and unused fourth markers need no projection. Required unavailable report markers become typed uncertainty instead of panic or NaN.',
        'Accessibility retains source diameters and original indices in native circle candidates. Spacing uses a certified strict radius predicate and native hypot only when a finding needs a gap diagnostic. Missing diameters remain explicit metadata findings; present invalid or unrepresentable sizes cannot silently disappear.',
        'Effective probe radii are alternatives for the maximum of the declared and minimum diameters. Side matching also keeps both spacing-floor alternatives. Membership does not require unrelated maximum-order certificates. A mismatch needs certified opposite-side presence and certified requested-side absence; either an expected-side witness or certified opposite-side absence settles the non-mismatch case.',
        'Copper clearance uses native region enclosures independent of authored feature markers. Same-net and certified empty geometry do not require unused probe predicates. Required source/classification/distance failures cannot also emit an asserted copper hit.',
        'All four checks retain STRICT and shared-budget failure, including empty and warm paths. Fixed-decimal diagnostics remain display projections; checked report markers are required only for findings.',
    ],
    baseline=dict(record='baseline.json', passed=4, failed=20, reconstructed_source_files=129,
                  full_reports='baseline-reports/manifest.json',
                  reused_library_provenance='baseline-reports/reused-cost-manifest.json'),
    validation=dict(status='passed', status_file='validation/status.json', source_hashes='validation/sources.json',
                    drc_passed=1502, drc_library=1497, readme_integration=2, drc_doctests=3,
                    new_regressions=48, initial_regressions=24,
                    strict_all_feature_all_target_clippy=['hyperdrc'], strict_rustdoc=['hyperdrc'],
                    workspace_format_check=True, source_frozen_through_cost_and_report_replay=True),
    consumer_validation=dict(status='passed', status_file='consumer-validation/status.json', passed=14,
                             strict_all_feature_all_target_clippy=True, snapshots='snapshot-versions.json',
                             six_snapshots_only_version_15_to_16=True),
    cost=dict(status='passed', manifest='cost/manifest.json', rows=cost['rows'], trials=5,
              previous_one_hundred_sixty_six_witness_rows_unchanged=True,
              new_medians_ns={k:v for k,v in cost['medians_ns'].items() if k.startswith('operation_fixture_')},
              peak_rss_kib=[min(rss),max(rss)], fixture_seconds=seconds,
              fixture_peak_rss_kib=fixture_rss, fixture_gate_seconds=120, limitations=cost['conditions']),
    report_replay=dict(manifest='report-replay/manifest.json', reports=replay['reports'],
                       complete_candidate_bytes_identical=True, baseline_version_fields_per_board=4,
                       baseline_diagnostic_changes={label:len(row['diagnostic_changes']) for label,row in delta['reports'].items()},
                       exact_delta_audit='report-replay/baseline-delta-audit.json',
                       initial_version_only_gate='report-replay/initial-comparison.json',
                       scope=delta['scope']),
    fixture_finding_audit='cost/fixture-report-audit.json', constructor_inventory='constructor-inventory.json',
    staged_validation=['candidate-initial.json', 'expanded-validation/status.json', 'expanded-fixed/status.json'],
    focused_validation=dict(record='expanded-fixed/status.json', new_regressions=48, existing_ipc=14, existing_thermal=4, existing_drills=21),
    lint_correction='expanded-validation/correction.json',
    inventory_correction='draft/inventory-correction.json',
    remaining=[
        '40 legacy CopperSpatialIndex constructor sites remain. Native shape predicates/projections, other drill/distribution logic, width/density proxies, scalar thresholds, source adapters and complete property coverage remain open. Constructor counts are syntactic, not behavioral coverage.',
        'The finite rectangle construction failure at 1e20 remains open. Its original dense-cluster regression, failure log and corrected native-grid fixture remain retained; this campaign also keeps the actual failed source in its copper capability probe.',
        'General Real alternate/scientific formatting still seeks a leading bit for unresolved zero. Explicit fixed-decimal diagnostics do not establish universal magnitude, precision or expression resource bounds.',
        'Complete Report comparisons cover these four checks on two full preserved KiCad fixtures plus a retained synthetic seven-record IPC sidecar. This is not universal or path-independent report determinism or a claim about an original board-authored IPC export. The separate smoke fixture retains its original checks and inputs.',
        'Shared STRICT and budget guards do not establish total operation resource bounds. All 39 properties, all 14 targets and crate future work remain in scope.',
        'No push or CI configuration change. The complete repository pre-push workflow matrix has not been run.',
    ],
    replay=['reproduce_focused.py OUTPUT', 'reproduce_baseline.py BASE_CHECKOUT OUTPUT', 'reproduce_baseline_reports.py BASE_CHECKOUT OUTPUT',
            'reproduce_validation.py OUTPUT', 'reproduce_consumer_validation.py OUTPUT',
            'reproduce_cost.py VALIDATION_OUTPUT COST_OUTPUT', 'audit_fixture_reports.py COST_OUTPUT',
            'reproduce_report_replay.py COST_OUTPUT REPORT_OUTPUT', 'audit_report_deltas.py REPORT_OUTPUT',
            'audit_constructor_inventory.py',
            'freeze_logs.py', 'build_manifest.py', 'audit_artifacts.py'],
    runtime=dict(validation=str(record / 'validation'), consumer_validation=str(record / 'consumer-validation'),
                 cost='/tmp/drc-fixture-access-cost', report_replay='/tmp/drc-fixture-access-report-replay',
                 baseline_libraries='/tmp/drc-fixture-access-baseline-libraries', inputs='/tmp/drc-fixture-access-report-inputs'),
    other_input_hashes={str(p):info(p)['sha256'] for p in [Path('Cargo.toml'), Path('Cargo.lock'), Path('AGENTS.md'),
                        Path('crates/hyperdrc/Cargo.toml'), Path('crates/hypercircuit/Cargo.toml'),
                        Path('crates/hyperdrc/README.md'), Path('crates/hyperdrc/PERFORMANCE.md'),
                        Path('.github/workflows/workspace.yml')]},
    artifact_integrity_audit='artifact-audit.json',
    evidence_files={str(p.relative_to(record)):info(p) for p in sorted(record.rglob('*')) if p.is_file() and p not in [record/'manifest.json', record/'artifact-audit.json']},
)
(record / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
print('Wrote fixture-access checkpoint manifest.')
