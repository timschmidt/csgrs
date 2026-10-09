"""Verify source reconstruction, retained results, report bytes and full goal scope."""
import ast
import gzip
import hashlib
import json
from pathlib import Path
import re
import statistics
import subprocess

record = Path(__file__).resolve().parent
load = lambda path: json.loads(path.read_bytes())
digest = lambda data: hashlib.sha256(data).hexdigest()
info = lambda path: dict(bytes=Path(path).stat().st_size, sha256=digest(Path(path).read_bytes()))
manifest = load(record / 'manifest.json')
base = manifest['base_revision']
pattern = re.compile(r'^test result: (ok|FAILED)\. (\d+) passed; (\d+) failed; (\d+) ignored;', re.M)

def results(path, passed, failed=0):
    rows = pattern.findall(path.read_text())
    assert rows, path
    assert [sum(int(row[i]) for row in rows) for i in [1,2,3]] == [passed,failed,0], path
    return dict(passed=passed, failed=failed, ignored=0)

source_counts = {}
for folder, names in [
    ('validation', ['fmt','clippy','test','docs','fixture-build']),
    ('consumer-validation', ['fmt','consumer-clippy','consumer-drc','consumer-curves']),
    ('expanded-fixed', ['clippy','focused','kicad']),
]:
    steps = load(record / folder / 'status.json')
    assert [step['name'] for step in steps] == names
    assert all(step['exit_code'] == 0 for step in steps)
    sources = load(record / folder / 'sources.json')
    assert sources['unchanged']
    for path, expected in sources['source_hashes'].items():
        assert info(path)['sha256'] == expected, path
    source_counts[folder] = len(sources['source_hashes'])
tests = {
    'drc': results(record/'validation/test.log',1528),
    'consumer_drc': results(record/'consumer-validation/consumer-drc.log',9),
    'consumer_curves': results(record/'consumer-validation/consumer-curves.log',5),
    'rectangles': results(record/'expanded-fixed/focused.log',20),
    'kicad': results(record/'expanded-fixed/kicad.log',42),
    'initial_rectangles': results(record/'candidate-initial/focused.log',12),
    'initial_geometry': results(record/'candidate-initial/existing-geometry.log',59),
    'initial_affine': results(record/'candidate-initial/existing-affine.log',12),
    'integration_failure': results(record/'integration-failure/test.log',1515,2),
    'feature_race': results(record/'feature-race/test.log',1525,2),
    'before_underflow': results(record/'before-underflow-validation/test.log',1527),
    'baseline': results(record/'baseline.log',3,9),
}
baseline = load(record/'baseline.json')
assert baseline['production_commit'] == base and baseline['exit_code'] == 101 and baseline['unchanged']
for path, expected in baseline['source_hashes'].items():
    if path.endswith('/geometry/rectangle_regressions.rs'):
        data = (record/'baseline-regressions.rs').read_bytes()
    else:
        data = subprocess.check_output(['git','show',f'{base}:{path}'])
        if path.endswith('/geometry.rs'):
            anchor = baseline['injection_anchor'].encode()
            assert data.count(anchor) == 1
            data = data.replace(anchor, anchor+baseline['test_module_registration'].encode())
    assert digest(data) == expected, path
assert len(baseline['source_hashes']) == 135
for folder in ['candidate-initial','expanded-compile','integration-failure','consumer-initial','feature-race','before-underflow-validation','before-underflow-focused','before-underflow-consumer']:
    assert load(record/folder/'sources.json')['unchanged']
assert [s['exit_code'] for s in load(record/'expanded-compile/status.json')] == [0,101]
assert [s['exit_code'] for s in load(record/'integration-failure/status.json')] == [0,0,101]
assert [s['exit_code'] for s in load(record/'feature-race/status.json')] == [0,0,101]
assert load(record/'feature-race/sources.json')['source_hashes'] == load(record/'before-underflow-validation/sources.json')['source_hashes']
assert (record/'feature-race/test.log').read_text().count('error[E0460]') == 2
underflow = load(record/'underflow-correction/manifest.json')
assert underflow['build_exit_code'] == 0 and underflow['exit_code'] == 101
assert underflow['source'] == info(record/'underflow-correction/probe.rs')
after_underflow = load(record/'cost/auxiliary-probes/manifest.json')['underflow-after']
assert after_underflow['source'] == underflow['source']
assert after_underflow['build_exit_code'] == after_underflow['exit_code'] == 0
assert (record/'cost/auxiliary-probes/underflow-after.log').read_text().splitlines() == [
    '(size inf 1): 1 retained copper records',
    '(size 1e-400 1): 1 retained copper records',
    '(size 1 1e-400): 1 retained copper records',
]
assert digest((record/'underflow-correction/kicad-before-fix.rs').read_bytes()) == load(record/'before-underflow-validation/sources.json')['source_hashes']['crates/hyperdrc/src/kicad.rs']
before_fix = (record/'expanded-compile/regressions-before-fix.rs').read_bytes()
assert digest(before_fix) == load(record/'expanded-compile/sources.json')['source_hashes']['crates/hyperdrc/src/geometry/rectangle_regressions.rs']
assert digest(before_fix.replace(b'hypercurve::provisional(rect_polygon(', b'hypercurve::provisional(|| rect_polygon(')) == load(record/'integration-failure/sources.json')['source_hashes']['crates/hyperdrc/src/geometry/rectangle_regressions.rs']
assert load(record/'integration-failure/sources.json')['source_hashes']['crates/hyperdrc/src/kicad.rs'] == digest(subprocess.check_output(['git','show',f'{base}:crates/hyperdrc/src/kicad.rs']))

prior = record.parent/'2026-10-09-hyperdrc-fixture-access'
reused = load(record/'baseline-reports/reused-cost-manifest.json')
for path, expected in reused['source_hashes'].items():
    assert digest(subprocess.check_output(['git','show',f'{base}:{path}'])) == expected
before = load(record/'baseline-reports/manifest.json')
assert before['exit_code'] == 0 and before['driver'] == info(record/'full_report_replay.rs')
library = load(record/'baseline-library.json')
assert library['base_revision'] == base
assert library['sha256'] == 'f45b8e8fcc37daab5f8cfc598dc9d597ed34c94f8d03c33253594a3b4ddc1f5e'
assert before['linked_libraries'][library['path']] == {k:library[k] for k in ['bytes','sha256']}
for label,row in before['reports'].items():
    raw = gzip.decompress((record/f'baseline-reports/{label}.json.gz').read_bytes())
    assert len(raw) == row['bytes'] and digest(raw) == row['sha256']
for path, expected in before['archives'].items():
    assert info(path) == expected

for path,row in load(record/'log-storage.json').items():
    archive = (record/row['raw_archive']).read_bytes()
    raw = gzip.decompress(archive)
    assert digest(archive) == row['archive_sha256']
    assert len(raw) == row['raw_bytes'] and digest(raw) == row['raw_sha256']
    clean = b'\n'.join(line.rstrip(b' \t\r') for line in raw.split(b'\n'))
    assert clean == (record/path).read_bytes() and digest(clean) == row['display_sha256']
snapshots = load(record/'snapshot-versions.json')
assert len(snapshots) == 6
for row in snapshots:
    old = subprocess.check_output(['git','show',f"{base}:{row['path']}"])
    assert old.count(b'"check_version": 16') == row['changed_check_versions'] == 160
    assert Path(row['path']).read_bytes() == old.replace(b'"check_version": 16', b'"check_version": 17')
    assert digest(old) == row['previous_sha256'] and info(row['path'])['sha256'] == row['current_sha256']

cost = load(record/'cost/manifest.json')
assert cost['source_unchanged'] and cost['fixture_exit_code'] == 0
assert cost['source_hashes'] == load(record/'validation/sources.json')['source_hashes']
for key, stem in [('probe_source','operation'),('affine_probe_source','affine'),('native_drill_probe_source','native_drill'),('ipc_probe_source','ipc'),('layer_order_probe_source','layer_order'),('point_consumer_probe_source','point_consumer'),('authored_center_probe_source','authored_center'),('dense_cluster_probe_source','dense_cluster'),('fixture_access_probe_source','fixture_access'),('rectangle_probe_source','rectangle')]:
    assert cost[key] == info(record/'cost'/f'{stem}_probe.rs')
for path in (prior/'cost').glob('*.rs'):
    if path.name not in ['operation_probe.rs','dense_cluster_probe.rs','fixture_access_probe.rs']:
        assert (record/'cost'/path.name).read_bytes() == path.read_bytes()
rows = load(record/'cost/rows.json')
assert dict(count=len(rows), **info(record/'cost/rows.json')) == cost['rows'] == manifest['cost']['rows']
assert len(rows) == 205
observations = []
for trial in range(1,6):
    log = (record/f'cost/operation-{trial}.log').read_text()
    parsed = []
    for line in log.splitlines():
        if line.startswith('operation_'):
            name,iterations,nanoseconds,witness = line.split('\t')
            parsed.append(dict(name=name,iterations=int(iterations),ns_per_query=int(nanoseconds)/int(iterations),witness=witness))
    assert [{k:v for k,v in row.items() if k != 'ns_per_query'} for row in parsed] == rows
    assert re.search(r'Exit status: 0\s*$',log)
    observations.append(dict(rows=parsed,peak_rss_kib=int(re.search(r'Maximum resident set size \(kbytes\): (\d+)',log)[1])))
assert observations == cost['observations']
assert {row['name']:statistics.median(run['rows'][i]['ns_per_query'] for run in observations) for i,row in enumerate(rows)} == cost['medians_ns']
subprocess.run(['python3',str(record/'audit_cost_rows.py'),str(record/'cost')],check=True,stdout=subprocess.PIPE)
fixture = (record/'cost/fixture.log').read_text()
seconds = float(re.search(r'fixture_smoke_total ms=([\d.]+)',fixture)[1])/1000
assert seconds == manifest['cost']['fixture_seconds'] < 120
assert re.search(r'Exit status: 0\s*$',fixture)
fixture_audit = load(record/'cost/fixture-report-audit.json')
assert fixture_audit['ordered_check_counts_unchanged'] and fixture_audit['normalized_reports_unchanged']
assert fixture_audit['diff_lines'] == fixture_audit['finding_diff_lines'] == 0
assert not fixture_audit['duplicate_marker_changes']
assert all(step['exit_code'] == 0 for step in load(record/'checkpoint-validation.json'))

subprocess.run(['python3',str(record/'audit_report_deltas.py'),str(record/'report-replay')],check=True,stdout=subprocess.PIPE)
replay = load(record/'report-replay/manifest.json')
delta = load(record/'report-replay/baseline-delta-audit.json')
assert delta['status'] == 'passed'
assert load(record/'report-replay/initial-comparison.json') == dict(
    status='failed',reports={'cparti-fpga':True,'hvp109a':False},
    expectation='Complete baseline report bytes except exactly twelve implementation-version fields 16 to 17 per board.')
assert load(record/'report-replay/comparison-correction.json')['initial_exit_code'] == 1
assert 'AssertionError: Complete reports require explicit delta review' in (record/'report-replay/initial-replay.log').read_text()
assert 'assert initial["status"] == "passed"' in (record/'report-replay/initial-replay-script.py').read_text()
assert replay['source_unchanged'] and replay['source_hashes'] == cost['source_hashes']
assert len(replay['trials']) == 2 and all(t['exit_code'] == 0 for t in replay['trials'])
assert replay['cost_library_build'] == info(record/'cost/library-build.jsonl')
for label,row in replay['reports'].items():
    reports = []
    for path,expected in row['compressed_artifacts'].items():
        assert info(record/'report-replay'/path) == expected
        reports.append(gzip.decompress((record/'report-replay'/path).read_bytes()))
    assert len(reports) == 2 and reports[0] == reports[1]
    assert len(reports[0]) == row['bytes'] and digest(reports[0]) == row['sha256']
    assert row['identical_complete_bytes'] and row['baseline_version_fields'] == 12
for path,expected in {**cost['binaries'],**replay['binaries'],**replay['linked_libraries']}.items():
    if Path(path).exists(): assert info(path) == expected,path

for path,expected in manifest['evidence_files'].items():
    assert info(record/path) == expected,path
for path,expected in manifest['other_input_hashes'].items():
    assert info(path)['sha256'] == expected,path
for path in record.glob('*.py'): ast.parse(path.read_text(),filename=str(path))
for path in record.rglob('*.json'): load(path)
old = json.loads(subprocess.check_output(['git','show',f'{base}:docs/properties-status.json']))
new = load(Path('docs/properties-status.json'))
assert old['scope'] == new['scope']
for key,count in [('properties',39),('targets',14)]:
    assert len(new[key]) == count
    assert [(r['id'],r['status']) for r in old[key]] == [(r['id'],r['status']) for r in new[key]]
assert [r['path'] for r in old['future_work']['sources']] == [r['path'] for r in new['future_work']['sources']]
result = dict(status='passed',test_results=tests,validated_source_counts=source_counts,
              baseline_reconstructed_source_count=135,all_205_cost_rows_stable=True,
              old_193_workloads_preserved_with_explicit_delta_audit=True,
              complete_candidate_reports_identical=True,baseline_report_deltas_audited=True,
              six_snapshots_only_version_changed=True,unchanged_fixture_seconds=seconds,
              full_39_property_14_target_future_work_scope_preserved=True)
(record/'artifact-audit.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
