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
    scope='Six authored-center readiness consumers: connector return path, decoupling proximity, RF via fences, plating intent, mounting-hole grounding and mounting-hole plating intent. The shared checked mounting-size selector additionally propagates uncertainty to mounting-hole copper keepout and edge clearance; their remaining geometry paths are not certified by this checkpoint. All 39 properties, 14 targets and crate future work remain in scope.',
    contract=[
        'Authored center matching uses native outward enclosures to schedule candidates and checked squared distances to decide closed-radius membership. Unavailable optional enclosures retain candidates; required unresolved predicates and required unavailable report markers become typed uncertainty.',
        'All six center consumers retain STRICT and shared-budget failures, including empty and warm paths. Any certified matching alternative proves existence; otherwise unresolved candidates cannot become certified absence. Layer, net and copper-kind eligibility precede required matching.',
        'NPTH radius construction validates diameter. Mounting-size eligibility compares diameter to both one and the clearance hint without requiring an order between thresholds; a certified too-small bound proves ineligibility. An explicit ground/chassis net proves plated mounting intent without optional geometric evidence.',
        'Per-query native radii reuse the point enclosure index without changing its completeness contract. Copper shape carriers do not determine these authored-center proxies, except the connector rule\'s required native edge selection. Required fixed-decimal diagnostics remain explicit display projections.',
    ],
    baseline=dict(record='baseline.json', passed=5, failed=28, reconstructed_source_files=122,
                  full_reports='baseline-reports/manifest.json',
                  reused_library_provenance='baseline-reports/reused-cost-manifest.json'),
    validation=dict(status='passed', status_file='validation/status.json', source_hashes='validation/sources.json',
                    drc_passed=1416, drc_library=1411, readme_integration=2, drc_doctests=3,
                    new_regressions=49, initial_regressions=33,
                    strict_all_feature_all_target_clippy=['hyperdrc'], strict_rustdoc=['hyperdrc'],
                    workspace_format_check=True, source_frozen_through_cost_and_report_replay=True),
    readme_validation='readme-validation/manifest.json',
    consumer_validation=dict(status='passed', status_file='consumer-validation/status.json', passed=14,
                             strict_all_feature_all_target_clippy=True, snapshots='snapshot-versions.json',
                             six_snapshots_only_version_13_to_14=True),
    cost=dict(status='passed', manifest='cost/manifest.json', rows=cost['rows'], trials=5,
              previous_one_hundred_ten_witness_rows_unchanged=True,
              new_medians_ns={k:v for k,v in cost['medians_ns'].items() if k.startswith('operation_authored_')},
              peak_rss_kib=[min(rss),max(rss)], fixture_seconds=seconds,
              fixture_peak_rss_kib=fixture_rss, fixture_gate_seconds=120, limitations=cost['conditions']),
    report_replay=dict(manifest='report-replay/manifest.json', reports=replay['reports'],
                       complete_candidate_bytes_identical=True, baseline_version_fields_per_board=8,
                       baseline_diagnostic_changes={'cparti-fpga':6,'hvp109a':0},
                       exact_delta_audit='report-replay/baseline-delta-audit.json',
                       initial_version_only_gate='report-replay/initial-comparison.json',
                       scope=delta['scope']),
    fixture_finding_audit='cost/fixture-report-audit.json', constructor_inventory='constructor-inventory.json',
    staged_validation=['candidate-initial.json', 'candidate-expanded.json'],
    remaining=[
        '48 legacy CopperSpatialIndex constructor sites remain, including derived-center cluster proxies and copper-shape queries. Their required predicates/projections, other drill/distribution logic, width/density proxies, source adapters and pad/track/slot/custom geometry remain unaudited or incomplete.',
        'The two additional mounting-hole keepout/edge checks gain only the checked shared selector; their other geometry, projections and budget fast paths remain open.',
        'General Real alternate/scientific formatting still seeks a leading bit for unresolved zero. Explicit fixed-decimal display is used at the repaired connector, decoupling and mounting diagnostic sites; universal magnitude, precision and expression resource bounds remain open.',
        'No universal or path-independent report determinism claim. The complete Report comparison covers these eight affected checks and two preserved full KiCad fixtures. The separate smoke fixture retains its original checks and inputs.',
        'Shared STRICT and budget guards do not establish total operation resource bounds. All 39 properties, all 14 targets and crate future work remain in scope.',
        'No push or CI configuration change. The complete repository pre-push workflow matrix has not been run.',
    ],
    replay=['reproduce_baseline.py BASE_CHECKOUT OUTPUT', 'reproduce_baseline_reports.py BASE_CHECKOUT OUTPUT',
            'reproduce_validation.py OUTPUT', 'reproduce_consumer_validation.py OUTPUT',
            'reproduce_readme_validation.py OUTPUT',
            'reproduce_cost.py VALIDATION_OUTPUT COST_OUTPUT', 'audit_fixture_reports.py COST_OUTPUT',
            'reproduce_report_replay.py COST_OUTPUT REPORT_OUTPUT', 'audit_report_deltas.py REPORT_OUTPUT',
            'audit_constructor_inventory.py',
            'freeze_logs.py', 'build_manifest.py', 'audit_artifacts.py'],
    runtime=dict(validation='/tmp/drc-authored-centers-validation', consumer_validation='/tmp/drc-authored-centers-consumer',
                 cost='/tmp/drc-authored-centers-cost', report_replay='/tmp/drc-authored-centers-report-replay',
                 baseline_libraries='/tmp/drc-authored-centers-baseline-libraries', inputs='/tmp/drc-authored-centers-report-inputs'),
    other_input_hashes={str(p):info(p)['sha256'] for p in [Path('Cargo.toml'), Path('Cargo.lock'), Path('AGENTS.md'),
                        Path('crates/hyperdrc/Cargo.toml'), Path('crates/hypercircuit/Cargo.toml'),
                        Path('crates/hyperdrc/README.md'), Path('crates/hyperdrc/PERFORMANCE.md'),
                        Path('.github/workflows/workspace.yml')]},
    artifact_integrity_audit='artifact-audit.json',
    evidence_files={str(p.relative_to(record)):info(p) for p in sorted(record.rglob('*')) if p.is_file() and p not in [record/'manifest.json', record/'artifact-audit.json']},
)
(record / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
print('Wrote authored-center checkpoint manifest.')
