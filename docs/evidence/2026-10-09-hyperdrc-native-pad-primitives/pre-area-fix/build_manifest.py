"""Assemble the pad-primitive checkpoint after validation and report review."""
import hashlib
import json
from pathlib import Path
import re

record = Path(__file__).resolve().parent
load = lambda path: json.loads(path.read_bytes())
info = lambda path: dict(bytes=Path(path).stat().st_size,sha256=hashlib.sha256(Path(path).read_bytes()).hexdigest())
for folder in ['expanded','consumer-validation','validation']:
    assert all(s['exit_code'] == 0 for s in load(record/folder/'status.json'))
    assert load(record/folder/'sources.json')['unchanged']
cost = load(record/'cost/manifest.json')
assert cost['source_unchanged'] and cost['fixture_exit_code'] == 0
assert load(record/'cost/row-audit.json')['status'] == 'passed'
report = load(record/'report-replay/manifest.json')
delta = load(record/'report-replay/baseline-delta-audit.json')
assert report['source_unchanged'] and delta['status'] == 'passed'
fixture = (record/'cost/fixture.log').read_text()
seconds = float(re.search(r'fixture_smoke_total ms=([\d.]+)',fixture)[1])/1000
rss = [r['peak_rss_kib'] for r in cost['observations']]
result = dict(base_revision=load(record/'baseline.json')['production_commit'],status='progress',
    scope='Native rounded/chamfered rectangle and KiCad trapezoid construction at finite primitive API boundaries. All 39 properties, 14 targets and crate future work remain in scope.',
    contract=[
        'Finite inputs enter native scalar arithmetic before half-size calculation, radius/chamfer clamping, corner offsets, rotation and translation. Report coordinates cannot reconstruct the native outline.',
        'Rounded rectangles retain native line/arc boundaries independently of report samples, including limiting capsules/circles. Chamfers remove only selected corner triangles and omit repeated limiting vertices.',
        'Trapezoids use the primary KiCad half-delta and corner-sign convention; excessive deltas remain source failures and limiting triangles remain valid. Two old report-view expectations were corrected to that convention.',
        'Zero dimensions denote certified emptiness; non-finite parameters, unresolved construction and shared-budget exhaustion remain failures. Warm measurements and enclosing previews retain STRICT behavior.',
        'The original x=1e20 and size 16384 hard scale remains intact for every family, including three KiCad parser handoff tests. Subnormal dimensions/radii and corners beyond the finite report range retain native geometry.',
    ],
    references='reference-sources.json',
    baseline=dict(record='baseline.json',passed=4,failed=16,reconstructed_source_files=138,library='baseline-library.json',full_reports='baseline-reports/reuse.json'),
    validation=dict(record='validation/status.json',drc_passed=1559,library_tests=1554,readme_tests=2,doctests=3,
                    new_regressions=31,pad_regressions=28,parser_handoff_regressions=3,source_files=141,
                    strict_all_feature_all_target_clippy=True,strict_rustdoc=True,workspace_format_check=True),
    focused=dict(record='expanded/status.json',pads=28,existing_geometry=59,existing_rectangles=20,kicad=45,
                 initial_record='candidate-initial/status.json',initial_pads=20,initial_kicad=42),
    consumer=dict(record='consumer-validation/status.json',passed=14,source_files=282,strict_all_feature_all_target_clippy=True,
                  snapshots='snapshot-versions.json',version_change=[17,18]),
    cost=dict(manifest='cost/manifest.json',rows=cost['rows'],trials=5,previous_rows=205,
              previous_inputs_iterations_and_witnesses_unchanged=True,new_construction_rows=20,new_warm_rows=6,
              row_audit='cost/row-audit.json',new_medians_ns={k:v for k,v in cost['medians_ns'].items() if k.startswith('operation_pad_')},
              peak_rss_kib=[min(rss),max(rss)],fixture_seconds=seconds,fixture_gate_seconds=120,
              fixture_peak_rss_kib=int(re.search(r'Maximum resident set size \(kbytes\): (\d+)',fixture)[1]),limitations=cost['conditions']),
    reports=dict(manifest='report-replay/manifest.json',results=report['reports'],complete_candidate_bytes_repeat=True,
                 initial_comparison='report-replay/initial-comparison.json',delta_audit='report-replay/baseline-delta-audit.json',scope=delta['scope']),
    fixture_text_audit='cost/fixture-report-audit.json',remaining_source_audit='remaining-source-audit.json',
    remaining=[
        'KiCad ratio products, source-token/placement parsing and other shape adapters still have finite boundaries. Exact primitive construction cannot recover source information already rounded by callers.',
        'Circle size halving, oval endpoint/body construction, line normalization and sampled arc/Bezier strokes require native source conversion. Malformed optional source tokens still need a complete uncertainty audit.',
        'Forty legacy copper-index constructor sites, remaining shape/drill/distribution logic, scalar/net-class thresholds, report-chord widths and density proxies remain open.',
        'General scalar identities, total operation resource bounds, bounded report allocation and universal/path-independent report identity remain unproven.',
        'This checkpoint does not complete any property/target or crate future-work scope. No push or CI configuration change; the full repository pre-push workflow matrix was not run.',
    ],
    replay=['reproduce_baseline.py BASE_CHECKOUT OUTPUT','reproduce_focused.py OUTPUT','reproduce_final_validation.py OUTPUT_ROOT',
            'reproduce_cost.py VALIDATION_OUTPUT COST_OUTPUT','audit_cost_rows.py COST_OUTPUT','audit_fixture_reports.py COST_OUTPUT',
            'reproduce_report_replay.py COST_OUTPUT REPORT_OUTPUT','audit_report_deltas.py REPORT_OUTPUT','freeze_logs.py','build_manifest.py','audit_artifacts.py'],
    runtime=dict(cost='/tmp/drc-native-pad-primitives-cost',reports='/tmp/drc-native-pad-primitives-report-replay',inputs='/tmp/drc-native-rectangles-report-inputs',baseline_library='/tmp/drc-native-pad-primitives-baseline-libraries/libhyperdrc.rlib'),
    other_input_hashes={p:info(p)['sha256'] for p in ['Cargo.toml','Cargo.lock','AGENTS.md','.github/workflows/workspace.yml','crates/hyperdrc/Cargo.toml','crates/hypercircuit/Cargo.toml','crates/hyperdrc/README.md','crates/hyperdrc/PERFORMANCE.md','crates/hyperdrc/src/geometry/README.md']},
    artifact_integrity_audit='artifact-audit.json',
    evidence_files={str(p.relative_to(record)):info(p) for p in sorted(record.rglob('*')) if p.is_file() and p not in [record/'manifest.json',record/'artifact-audit.json']})
(record/'manifest.json').write_text(json.dumps(result,indent=2)+'\n')
print('Wrote native pad-primitive checkpoint manifest.')
